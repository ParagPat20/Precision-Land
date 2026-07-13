import sys
import os
import glob
import time
import threading
import traceback

# Add STServo SDK to the path
_SDK_ROOT_1 = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "STServo_Python"))
_SDK_ROOT_2 = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for path in [_SDK_ROOT_1, _SDK_ROOT_2]:
    if path not in sys.path:
        sys.path.append(path)

_SDK_IMPORT_ERROR = None
try:
    from STservo_sdk import *
except ImportError:
    try:
        from STServo_Python.STservo_sdk import *
    except ImportError as e:
        _SDK_IMPORT_ERROR = e

# --- CONFIGURATION CONSTANTS FOR LOCK/UNLOCK SEQUENCE ---
# Speeds & Accel
ST_SPEED = 2400
ST_ACC = 200
SC_SPEED = 1500

# Locking Targets
LOCK_POS_1 = 350
LOCK_POS_2 = 480
LOCK_POS_3 = 740

# Unlocking Targets
UNLOCK_POS_1 = 2100
UNLOCK_POS_2 = 650
UNLOCK_POS_3 = 550
UNLOCK_CHECK_3 = 560  # Threshold check for ID 3

# Absolute Physical Mechanical Limits to prevent over-travel or losing linkage handlers
SERVO_LIMITS = {
    1: (150, 2100),  # Servo 1 (ST)
    2: (450, 960),   # Servo 2 (SC)
    3: (530, 750)    # Servo 3 (SC)
}
# --------------------------------------------------------


def resolve_servo_port(manual_port=None):
    """
    Resolve the STServo serial bus for Linux/RPi first, while keeping Windows
    development usable. Set JECH_SERVO_PORT or pass --servo-port to override.
    """
    if manual_port:
        return manual_port

    env_port = os.environ.get("JECH_SERVO_PORT")
    if env_port:
        return env_port

    if os.name == "nt":
        return "COM21"

    by_id_patterns = [
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110734-if00",
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00",
        "/dev/serial/by-id/*1a86*USB*Single*Serial*",
        "/dev/serial/by-id/*CH340*",
        "/dev/serial/by-id/*ch341*",
        "/dev/serial/by-id/*QinHeng*",
        "/dev/serial/by-id/*CP210*",
        "/dev/serial/by-id/*Silicon_Labs*",
        "/dev/serial/by-id/*FTDI*",
        "/dev/serial/by-id/*USB*Serial*",
    ]
    candidates = []
    for pattern in by_id_patterns:
        candidates.extend(sorted(glob.glob(pattern)))
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    candidates.extend(["/dev/serial0", "/dev/ttyAMA0"])

    seen = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        name = os.path.basename(candidate).lower()
        if "pixhawk" in name or "ardupilot" in name or "prolific" in name:
            continue
        if os.path.exists(candidate):
            return candidate

    return "/dev/serial0"

class ServoController:
    """
    Manages ST3215 and SC09 servos for Precision Landing.
    The working servo_tool.py setup uses the STS packet handler for all IDs,
    so this controller follows that bus protocol unless changed later.
    ID 1: ST3215
    ID 2 & 3: SC09
    """
    def __init__(self, vehicle, port_name=None, baudrate=1000000, is_mission_active_cb=None):
        if _SDK_IMPORT_ERROR is not None:
            raise ImportError(f"STservo_sdk is not available: {_SDK_IMPORT_ERROR}")

        self.vehicle = vehicle
        self.port_name = resolve_servo_port(port_name)
        self.baudrate = baudrate
        self.is_mission_active_cb = is_mission_active_cb
        
        self.portHandler = None
        self.stsHandler = None
        self.scsHandler = None
        self.packetHandler = None
        self.connected = False
        
        self.st3215_id = 1
        self.sc09_ids = [2, 3]
        self.st_config = {"min": 0, "max": 4095, "home": 0}
        self.sc09_configs = {2: {"min": 0, "max": 1023}, 3: {"min": 0, "max": 1023}}
        self.servo_protocols = {1: "sts", 2: "scscl", 3: "scscl"}
        
        self._running = False
        self._thread = None
        self._io_lock = threading.Lock()
        
        # State tracking to avoid redundant writes
        self.st3215_locked = False
        self.sc09_locked = False

        # Lock / Unlock sequence state variables
        self.servo6_raw = 0
        self.last_servo6_raw_rx_time = 0
        self.last_stream_request_time = 0
        self.sequence_active = False
        self.last_triggered_state = 'lock'
        self.last_state = 'lock'

        # Register MAVLink message listener for Servo Output Channel 6
        if self.vehicle:
            try:
                self.vehicle.add_message_listener('SERVO_OUTPUT_RAW', self._servo_output_listener)
                print("[SERVO] Registered SERVO_OUTPUT_RAW listener for Channel 6.")
            except Exception as e:
                print(f"[SERVO] Warning: Failed to add SERVO_OUTPUT_RAW message listener: {e}")

        self._connect()

    def _connect(self):
        try:
            self.portHandler = PortHandler(self.port_name)
            self.stsHandler = sts(self.portHandler)
            self.scsHandler = scscl(self.portHandler)
            self.packetHandler = self.stsHandler
            
            if not self.portHandler.openPort():
                print(f"[SERVO] Error: Failed to open port {self.port_name}")
                return
                
            if not self.portHandler.setBaudRate(self.baudrate):
                print(f"[SERVO] Error: Failed to set baudrate to {self.baudrate}")
                return
                
            print(f"[SERVO] Successfully connected to {self.port_name} at {self.baudrate} bps")
            self.connected = True
        except Exception as e:
            print(f"[SERVO] Connection exception: {e}")
            self.connected = False

    def _handler_for(self, sid):
        return self.scsHandler if self.servo_protocols.get(int(sid)) == "scscl" else self.stsHandler

    def _is_sts(self, sid):
        return self.servo_protocols.get(int(sid), "sts") == "sts"

    def _is_st3215(self, sid):
        return int(sid) == self.st3215_id

    def _servo_ids(self):
        return [self.st3215_id] + list(self.sc09_ids)

    def _config_for(self, sid):
        sid = int(sid)
        if self._is_sts(sid):
            return self.st_config
        return self.sc09_configs.get(sid, {"min": 0, "max": 1023})

    def _home_position_for(self, sid):
        cfg = self._config_for(sid)
        if self._is_st3215(sid):
            return 2048
        return (int(cfg.get("min", 0)) + int(cfg.get("max", 1023))) // 2

    def _clamp_position(self, sid, position):
        cfg = self._config_for(sid)
        default_max = 4095 if self._is_st3215(sid) else 1023
        low = int(cfg.get("min", 0))
        high = int(cfg.get("max", default_max))
        if low > high:
            low, high = high, low
        return max(low, min(high, int(position)))

    def _normalize_config(self, st_config=None, sc09_configs=None):
        st_src = st_config or {}
        sc_src = sc09_configs or {}
        self.st_config = {
            "min": int(st_src.get("min", self.st_config.get("min", 0))),
            "max": int(st_src.get("max", self.st_config.get("max", 4095))),
            "home": int(st_src.get("home", self.st_config.get("home", 0))),
        }
        for sid in self.sc09_ids:
            src = sc_src.get(sid, sc_src.get(str(sid), self.sc09_configs.get(sid, {})))
            self.sc09_configs[sid] = {
                "min": int(src.get("min", self.sc09_configs.get(sid, {}).get("min", 0))),
                "max": int(src.get("max", self.sc09_configs.get(sid, {}).get("max", 1023))),
            }

    def get_config(self):
        return {
            "st3215": dict(self.st_config),
            "sc09_2": dict(self.sc09_configs.get(2, {"min": 0, "max": 1023})),
            "sc09_3": dict(self.sc09_configs.get(3, {"min": 0, "max": 1023})),
            "protocols": dict(self.servo_protocols),
        }

    def _result_ok(self, sid, operation, result, error, handler=None):
        handler = handler or self._handler_for(sid)
        if result != COMM_SUCCESS:
            print(f"[SERVO] ID {sid} {operation} failed: {handler.getTxRxResult(result)}")
            return False
        if error:
            print(f"[SERVO] ID {sid} {operation} servo error: {handler.getRxPacketError(error)}")
            return False
        return True

    def _write1(self, sid, address, value, operation):
        handler = self._handler_for(sid)
        result, error = handler.write1ByteTxRx(sid, address, value)
        return self._result_ok(sid, operation, result, error, handler)

    def _write2(self, sid, address, value, operation):
        handler = self._handler_for(sid)
        result, error = handler.write2ByteTxRx(sid, address, value)
        return self._result_ok(sid, operation, result, error, handler)

    def _unlock_eprom(self, sid):
        handler = self._handler_for(sid)
        result, error = handler.unLockEprom(sid)
        return self._result_ok(sid, "EEPROM unlock", result, error, handler)

    def _lock_eprom(self, sid):
        handler = self._handler_for(sid)
        result, error = handler.LockEprom(sid)
        return self._result_ok(sid, "EEPROM lock", result, error, handler)

    def _read1(self, sid, address):
        handler = self._handler_for(sid)
        value, result, error = handler.read1ByteTxRx(sid, address)
        if not self._result_ok(sid, f"read address {address}", result, error, handler):
            return None
        return value

    def _read2(self, sid, address):
        handler = self._handler_for(sid)
        value, result, error = handler.read2ByteTxRx(sid, address)
        if not self._result_ok(sid, f"read address {address}", result, error, handler):
            return None
        return value

    def initialize_servos(self, st_config=None, sc09_configs=None):
        """
        Sets the home position offset, max and min positions for all servos.
        st_config: dict with 'min', 'max', 'home'
        sc09_configs: dict mapping sid to dict with 'min', 'max'
        """
        if not self.connected:
            print("[SERVO] Cannot initialize: Not connected.")
            return

        self._normalize_config(st_config, sc09_configs)

        print(f"[SERVO] Initializing Servos...")
        for sid in [self.st3215_id] + self.sc09_ids:
            try:
                with self._io_lock:
                    handler = self._handler_for(sid)
                    is_sts = self._is_sts(sid)
                    torque_addr = STS_TORQUE_ENABLE if is_sts else SCSCL_TORQUE_ENABLE
                    min_addr = STS_MIN_ANGLE_LIMIT_L if is_sts else SCSCL_MIN_ANGLE_LIMIT_L
                    max_addr = STS_MAX_ANGLE_LIMIT_L if is_sts else SCSCL_MAX_ANGLE_LIMIT_L
                    
                    if self._is_st3215(sid):
                        sid_min = self.st_config.get("min", 0)
                        sid_max = self.st_config.get("max", 4095)
                        home_offset = self.st_config.get("home", 0)
                    else:
                        sc_cfg = self.sc09_configs.get(sid, {"min": 0, "max": 1023})
                        sid_min = sc_cfg.get("min", 0)
                        sid_max = sc_cfg.get("max", 1023)

                    if not self._write1(sid, torque_addr, 0, "disable torque"):
                        continue
                    if not self._unlock_eprom(sid):
                        continue

                    if not self._write2(sid, min_addr, sid_min, "write min angle"):
                        self._lock_eprom(sid)
                        continue
                    if not self._write2(sid, max_addr, sid_max, "write max angle"):
                        self._lock_eprom(sid)
                        continue

                    if self._is_st3215(sid):
                        val = handler.sts_toscs(home_offset, 11)
                        if not self._write2(sid, STS_OFS_L, val, "write home offset"):
                            self._lock_eprom(sid)
                            continue

                    time.sleep(0.1)
                    if not self._lock_eprom(sid):
                        continue

                    # Initialize servos directly to their Unlock sequence positions on startup
                    if sid == 1:
                        target_pos = UNLOCK_POS_1
                        speed_val = 2400
                    elif sid == 2:
                        target_pos = UNLOCK_POS_2
                        speed_val = 1500
                    elif sid == 3:
                        target_pos = UNLOCK_POS_3
                        speed_val = 1500
                    else:
                        target_pos = self._home_position_for(sid)
                        speed_val = 500

                    if is_sts:
                        if not self._write1(sid, STS_MODE, 0, "set position mode"):
                            continue
                        result, error = handler.WritePosEx(sid, target_pos, speed_val, 50)
                    else:
                        result, error = handler.WritePos(sid, target_pos, 0, speed_val)
                    if not self._result_ok(sid, "move to unlocking position", result, error, handler):
                        continue

                print(f"[SERVO] Servo ID {sid} initialized.")
            except Exception as e:
                print(f"[SERVO] Error initializing servo ID {sid}: {e}")
                # Prevent "Port is in use!" subsequent errors
                if hasattr(self, "portHandler") and self.portHandler:
                    self.portHandler.is_using = False

        # Startup sequence: Wait 2 seconds, then execute the locked sequence
        def startup_locking_thread():
            time.sleep(2.0)
            print("[SERVO] Startup delay complete. Executing locked sequence...")
            self.perform_locking()

        threading.Thread(target=startup_locking_thread, daemon=True, name="ServoStartupLocking").start()

    def set_torque(self, sid, enable):
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        sid = int(sid)
        state = 1 if enable else 0
        try:
            with self._io_lock:
                torque_addr = STS_TORQUE_ENABLE if self._is_sts(sid) else SCSCL_TORQUE_ENABLE
                ok = self._write1(sid, torque_addr, state, "set torque")
            return {"ok": ok, "id": sid, "torque": bool(enable)}
        except Exception as e:
            print(f"[SERVO] Error setting torque for ID {sid}: {e}")
            return {"ok": False, "id": sid, "error": str(e)}

    def set_all_torque(self, enable):
        results = [self.set_torque(sid, enable) for sid in self._servo_ids()]
        return {"ok": all(item.get("ok") for item in results), "results": results}

    def move_servo(self, sid, position, speed=None, acc=50):
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        sid = int(sid)
        position = self._clamp_position(sid, position)
        speed = int(speed if speed is not None else (2400 if self._is_sts(sid) else 500))
        acc = int(acc)
        handler = self._handler_for(sid)
        try:
            with self._io_lock:
                if self._is_sts(sid):
                    if not self._write1(sid, STS_MODE, 0, "set position mode"):
                        return {"ok": False, "id": sid, "error": "failed to set position mode"}
                    result, error = handler.WritePosEx(sid, position, speed, acc)
                else:
                    result, error = handler.WritePos(sid, position, 0, speed)
                ok = self._result_ok(sid, "move", result, error, handler)
            return {"ok": ok, "id": sid, "position": position, "speed": speed, "acc": acc}
        except Exception as e:
            if self.portHandler:
                self.portHandler.is_using = False
            print(f"[SERVO] Error moving ID {sid}: {e}")
            return {"ok": False, "id": sid, "error": str(e)}

    def move_home(self, sid=None):
        ids = self._servo_ids() if sid in (None, "all") else [int(sid)]
        results = [self.move_servo(item, self._home_position_for(item)) for item in ids]
        return {"ok": all(item.get("ok") for item in results), "results": results}

    def reset_home_position(self, sid=None):
        """
        Reset saved home calibration. ST3215 supports a home offset register;
        SC09 home is defined as the midpoint of its configured min/max range.
        """
        ids = self._servo_ids() if sid in (None, "all") else [int(sid)]
        results = []
        for item in ids:
            if self._is_st3215(item):
                self.st_config["home"] = 0
                try:
                    with self._io_lock:
                        self._write1(item, STS_TORQUE_ENABLE, 0, "disable torque")
                        unlocked = self._unlock_eprom(item)
                        ok = False
                        if unlocked:
                            ok = self._write2(item, STS_OFS_L, 0, "reset home offset")
                            time.sleep(0.1)
                            self._lock_eprom(item)
                    move = self.move_servo(item, self._home_position_for(item))
                    results.append({"ok": ok and move.get("ok", False), "id": item, "home": 0, "move": move})
                except Exception as e:
                    if self.portHandler:
                        self.portHandler.is_using = False
                    results.append({"ok": False, "id": item, "error": str(e)})
            else:
                move = self.move_servo(item, self._home_position_for(item))
                results.append({"ok": move.get("ok", False), "id": item, "home": self._home_position_for(item), "move": move})
        return {"ok": all(item.get("ok") for item in results), "results": results}

    def read_status(self, sid=None):
        ids = self._servo_ids() if sid in (None, "all") else [int(sid)]
        return {"ok": True, "servos": [self._read_status_one(item) for item in ids], "config": self.get_config()}

    def _read_status_one(self, sid):
        sid = int(sid)
        is_sts = self._is_sts(sid)
        handler = self._handler_for(sid)
        torque_addr = STS_TORQUE_ENABLE if is_sts else SCSCL_TORQUE_ENABLE
        voltage_addr = STS_PRESENT_VOLTAGE if is_sts else SCSCL_PRESENT_VOLTAGE
        temp_addr = STS_PRESENT_TEMPERATURE if is_sts else SCSCL_PRESENT_TEMPERATURE
        load_addr = STS_PRESENT_LOAD_L if is_sts else SCSCL_PRESENT_LOAD_L
        current_addr = STS_PRESENT_CURRENT_L if is_sts else SCSCL_PRESENT_CURRENT_L
        moving_addr = STS_MOVING if is_sts else SCSCL_MOVING
        try:
            with self._io_lock:
                pos, spd, result, error = handler.ReadPosSpeed(sid)
                ok = self._result_ok(sid, "read position/speed", result, error, handler)
                payload = {
                    "ok": ok,
                    "id": sid,
                    "model": "ST3215" if is_sts else "SC09",
                    "position": pos if ok else None,
                    "speed": spd if ok else None,
                    "home_position": self._home_position_for(sid),
                    "torque": self._read1(sid, torque_addr),
                    "voltage_v": None,
                    "temperature_c": self._read1(sid, temp_addr),
                    "load": self._read2(sid, load_addr),
                    "current": self._read2(sid, current_addr),
                    "moving": self._read1(sid, moving_addr),
                }
                volts = self._read1(sid, voltage_addr)
                payload["voltage_v"] = None if volts is None else volts / 10.0
                if self._is_st3215(sid):
                    ofs = self._read2(sid, STS_OFS_L)
                    payload["home_offset"] = None if ofs is None else handler.sts_tohost(ofs, 11)
                if is_sts:
                    payload["mode"] = self._read1(sid, STS_MODE)
                return payload
        except Exception as e:
            if self.portHandler:
                self.portHandler.is_using = False
            return {"ok": False, "id": sid, "error": str(e)}

    def request_servo_output_raw_stream(self):
        if not self.vehicle:
            return
        try:
            print("[SERVO] Explicitly requesting SERVO_OUTPUT_RAW streams from Flight Controller...")
            # Method 1: MAV_CMD_SET_MESSAGE_INTERVAL (preferred in MAVLink 2)
            # Message ID 36 is SERVO_OUTPUT_RAW
            # 100000 microseconds = 10Hz (100ms interval)
            msg1 = self.vehicle.message_factory.command_long_encode(
                0, 0,    # target system, target component
                511,     # MAV_CMD_SET_MESSAGE_INTERVAL
                0,       # confirmation
                36,      # param 1: Message ID (36 for SERVO_OUTPUT_RAW)
                100000,  # param 2: Interval in microseconds
                0, 0, 0, 0, 0 # param 3-7
            )
            self.vehicle.send_mavlink(msg1)
            
            # Method 2: Legacy REQUEST_DATA_STREAM (fallback for MAVLink 1)
            # Stream ID 3 is MAV_DATA_STREAM_RAW_CONTROLLER (contains SERVO_OUTPUT_RAW)
            # 10Hz, start/stop = 1 (start)
            msg2 = self.vehicle.message_factory.request_data_stream_encode(
                0, 0,    # target system, target component
                3,       # MAV_DATA_STREAM_RAW_CONTROLLER
                10,      # rate (Hz)
                1        # start/stop (1 = start)
            )
            self.vehicle.send_mavlink(msg2)
        except Exception as e:
            print(f"[SERVO] Error requesting MAVLink streams: {e}")

    def _servo_output_listener(self, vehicle, name, message):
        self.servo6_raw = getattr(message, 'servo6_raw', 0)
        self.last_servo6_raw_rx_time = time.time()

    def _clamp_wiggle(self, sid, target, wiggle_dir):
        limits = SERVO_LIMITS.get(sid)
        if not limits:
            return target + (100 * wiggle_dir)
        wiggle_target = target + (100 * wiggle_dir)
        return max(limits[0], min(limits[1], wiggle_target))

    def robust_move_st_single(self, sid, target, speed, acc, timeout=10.0):
        print(f"[SERVO] Moving Servo {sid} to {target} (timeout {timeout}s)...")
        start_time = time.time()
        
        with self._io_lock:
            self._write1(sid, STS_TORQUE_ENABLE, 1, "enable torque")
            self.stsHandler.WritePosEx(sid, target, speed, acc)
        
        last_pos = -1
        start_pos = None
        stuck_count = 0
        wiggle_dir = 1
        
        while time.time() - start_time < timeout:
            time.sleep(0.3)
            
            with self._io_lock:
                pos, res, error = self.stsHandler.ReadPos(sid)
            
            if res == COMM_SUCCESS:
                if start_pos is None:
                    start_pos = pos
                print(f"  [ID {sid}] Current Pos: {pos} | Target: {target} (Start: {start_pos})")
                
                is_reached = False
                if start_pos is not None:
                    if target < start_pos:
                        is_reached = (pos <= target + 30)
                    else:
                        is_reached = (pos >= target - 30)
                else:
                    is_reached = (abs(pos - target) <= 30)
                    
                if is_reached:
                    print(f"  -> Servo {sid} reached target!")
                    return True
                    
                if abs(pos - last_pos) < 3:
                    stuck_count += 1
                    if stuck_count >= 2:
                        wiggle_target = self._clamp_wiggle(sid, target, wiggle_dir)
                        print(f"  [ID {sid}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                        with self._io_lock:
                            self._write1(sid, STS_TORQUE_ENABLE, 1, "enable torque")
                            self.stsHandler.WritePosEx(sid, int(wiggle_target), speed, acc)
                        wiggle_dir *= -1
                        stuck_count = 0
                else:
                    stuck_count = 0
                last_pos = pos
            else:
                print(f"  [ID {sid}] Failed to read position (possibly resetting)...")
        print(f"  -> Timeout reached for Servo {sid}! Did not reach {target}.")
        return False

    def robust_move_sc_single(self, sid, target, speed, timeout=10.0):
        print(f"[SERVO] Moving SC Servo {sid} to {target} (timeout {timeout}s)...")
        start_time = time.time()
        
        with self._io_lock:
            self._write1(sid, SCSCL_TORQUE_ENABLE, 1, "enable torque")
            self.scsHandler.WritePos(sid, target, 0, speed)
        
        last_pos = -1
        start_pos = None
        stuck_count = 0
        wiggle_dir = 1
        
        while time.time() - start_time < timeout:
            time.sleep(0.3)
            
            with self._io_lock:
                pos, res, error = self.scsHandler.ReadPos(sid)
            
            if res == COMM_SUCCESS:
                if start_pos is None:
                    start_pos = pos
                print(f"  [ID {sid}] Current Pos: {pos} | Target: {target} (Start: {start_pos})")
                
                is_reached = False
                if start_pos is not None:
                    if target < start_pos:
                        is_reached = (pos <= target + 15)
                    else:
                        is_reached = (pos >= target - 15)
                else:
                    is_reached = (abs(pos - target) <= 15)
                    
                if is_reached:
                    print(f"  -> SC Servo {sid} reached target!")
                    return True
                    
                if abs(pos - last_pos) < 3:
                    stuck_count += 1
                    if stuck_count >= 2:
                        wiggle_target = self._clamp_wiggle(sid, target, wiggle_dir)
                        print(f"  [ID {sid}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                        with self._io_lock:
                            self._write1(sid, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(sid, int(wiggle_target), 0, speed)
                        wiggle_dir *= -1
                        stuck_count = 0
                else:
                    stuck_count = 0
                last_pos = pos
            else:
                print(f"  [ID {sid}] Failed to read position (possibly resetting)...")
                stuck_count = 5 # Force resend next successful read
                
        print(f"  -> Timeout reached for SC Servo {sid}! Did not reach {target}.")
        return False

    def robust_move_sc_pair(self, sid2, target2, sid3, target3, speed, timeout=15.0, check_target3=None, check_dir3='>='):
        print(f"[SERVO] Moving Servo {sid2} to {target2} and Servo {sid3} to {target3} together...")
        start_time = time.time()
        reached2 = False
        reached3 = False
        
        last_pos2 = -1
        last_pos3 = -1
        start_pos2 = None
        start_pos3 = None
        stuck_count2 = 0
        stuck_count3 = 0
        wiggle_dir2 = 1
        wiggle_dir3 = 1
        
        # Send initial commands
        with self._io_lock:
            self._write1(sid2, SCSCL_TORQUE_ENABLE, 1, "enable torque")
            self.scsHandler.WritePos(sid2, target2, 0, speed)
            self._write1(sid3, SCSCL_TORQUE_ENABLE, 1, "enable torque")
            self.scsHandler.WritePos(sid3, target3, 0, speed)
            
        while time.time() - start_time < timeout:
            time.sleep(0.3)
            
            if not reached2:
                with self._io_lock:
                    pos2, res2, error2 = self.scsHandler.ReadPos(sid2)
                if res2 == COMM_SUCCESS:
                    if start_pos2 is None:
                        start_pos2 = pos2
                    print(f"  [ID {sid2}] Current Pos: {pos2} | Target: {target2} (Start: {start_pos2})")
                    
                    is_reached2 = False
                    if start_pos2 is not None:
                        if target2 < start_pos2:
                            is_reached2 = (pos2 <= target2 + 15)
                        else:
                            is_reached2 = (pos2 >= target2 - 15)
                    else:
                        is_reached2 = (abs(pos2 - target2) <= 15)
                        
                    if is_reached2:
                        print(f"  -> Servo {sid2} reached target!")
                        reached2 = True
                    else:
                        if abs(pos2 - last_pos2) < 3:
                            stuck_count2 += 1
                            if stuck_count2 >= 3:
                                wiggle_target = self._clamp_wiggle(sid2, target2, wiggle_dir2)
                                print(f"  [ID {sid2}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                                with self._io_lock:
                                    self._write1(sid2, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                                    self.scsHandler.WritePos(sid2, int(wiggle_target), 0, speed)
                                wiggle_dir2 *= -1
                                stuck_count2 = 0
                        else:
                            stuck_count2 = 0
                    last_pos2 = pos2
                else:
                    print(f"  [ID {sid2}] Read failed (possibly resetting)...")
                    stuck_count2 = 5
                    
            if not reached3:
                with self._io_lock:
                    pos3, res3, error3 = self.scsHandler.ReadPos(sid3)
                if res3 == COMM_SUCCESS:
                    if start_pos3 is None:
                        start_pos3 = pos3
                    if check_target3 is not None:
                        print(f"  [ID {sid3}] Current Pos: {pos3} | Target: {target3} (Checking {check_dir3} {check_target3})")
                        if check_dir3 == '>=' and pos3 >= check_target3:
                            print(f"  -> Servo {sid3} crossed threshold {check_target3}!")
                            reached3 = True
                        elif check_dir3 == '<=' and pos3 <= check_target3:
                            print(f"  -> Servo {sid3} crossed threshold {check_target3}!")
                            reached3 = True
                    else:
                        print(f"  [ID {sid3}] Current Pos: {pos3} | Target: {target3} (Start: {start_pos3})")
                        is_reached3 = False
                        if start_pos3 is not None:
                            if target3 < start_pos3:
                                is_reached3 = (pos3 <= target3 + 20)
                            else:
                                is_reached3 = (pos3 >= target3 - 20)
                        else:
                            is_reached3 = (abs(pos3 - target3) <= 20)
                            
                        if is_reached3:
                            print(f"  -> Servo {sid3} reached target!")
                            reached3 = True
                            
                    if not reached3:
                        if abs(pos3 - last_pos3) < 3:
                            stuck_count3 += 1
                            if stuck_count3 >= 3:
                                wiggle_target = self._clamp_wiggle(sid3, target3, wiggle_dir3)
                                print(f"  [ID {sid3}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                                with self._io_lock:
                                    self._write1(sid3, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                                    self.scsHandler.WritePos(sid3, int(wiggle_target), 0, speed)
                                wiggle_dir3 *= -1
                                stuck_count3 = 0
                        else:
                            stuck_count3 = 0
                    last_pos3 = pos3
                else:
                    print(f"  [ID {sid3}] Read failed (possibly resetting)...")
                    stuck_count3 = 5
                    
            if reached2 and reached3:
                return True
                
        print(f"  -> Timeout reached for SC servos! Did not fully complete.")
        return False

    def perform_locking(self):
        self.sequence_active = True
        try:
            print("\n--- STARTING NATIVE LOCKING SEQUENCE ---")
            print(f"Step 1: Servo 1 (ST) -> {LOCK_POS_1}")
            self.robust_move_st_single(1, LOCK_POS_1, ST_SPEED, ST_ACC)
            
            print("\nWaiting 1s for mechanical settlement...")
            time.sleep(1.0)
            
            print(f"\nStep 2: Servo 2 & 3 (SC) -> {LOCK_POS_2} & {LOCK_POS_3}")
            self.robust_move_sc_pair(2, LOCK_POS_2, 3, LOCK_POS_3, SC_SPEED, check_target3=LOCK_POS_3, check_dir3='>=')
            print("\nLocking sequence complete!")
            self.last_state = 'lock'
        except Exception as e:
            print(f"[SERVO] Locking sequence failed: {e}")
            traceback.print_exc()
        finally:
            self.sequence_active = False

    def perform_unlocking(self):
        self.sequence_active = True
        try:
            print("\n--- STARTING NATIVE UNLOCKING SEQUENCE ---")
            print(f"Step 1: Servo 3 (SC) -> {UNLOCK_POS_3}")
            self.robust_move_sc_single(3, UNLOCK_POS_3, SC_SPEED)
            
            print(f"\nStep 2: Servo 2 (SC) -> {UNLOCK_POS_2}")
            self.robust_move_sc_single(2, UNLOCK_POS_2, SC_SPEED)
            
            print(f"\nStep 3: Servo 1 (ST) -> {UNLOCK_POS_1}")
            self.robust_move_st_single(1, UNLOCK_POS_1, ST_SPEED, ST_ACC)
            
            print("\nUnlocking sequence complete!")
            self.last_state = 'unlock'
        except Exception as e:
            print(f"[SERVO] Unlocking sequence failed: {e}")
            traceback.print_exc()
        finally:
            self.sequence_active = False

    def start_monitoring(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True, name="ServoMonitor")
        self._thread.start()
        print("[SERVO] Servo Output channel monitoring started.")

    def stop_monitoring(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _monitor_loop(self):
        print("[SERVO] Monitor loop started successfully.")
        while self._running:
            try:
                # Periodically request the stream if we haven't received a MAVLink update recently
                current_time = time.time()
                if (current_time - self.last_servo6_raw_rx_time > 3.0) and (current_time - self.last_stream_request_time > 5.0):
                    self.request_servo_output_raw_stream()
                    self.last_stream_request_time = current_time

                # Read target state from Servo Channel 6 (SERVO_OUTPUT_RAW)
                ch6 = 0
                if self.servo6_raw > 0:
                    ch6 = self.servo6_raw

                if ch6 > 0:
                    should_lock = ch6 > 1500
                    target_state = 'lock' if should_lock else 'unlock'
                    
                    if target_state != self.last_triggered_state and not self.sequence_active:
                        self.last_triggered_state = target_state
                        if target_state == 'lock':
                            print(f"[SERVO] Triggering LOCK sequence (Ch6/Servo6 Raw: {ch6})")
                            threading.Thread(target=self.perform_locking, daemon=True, name="LockSequenceThread").start()
                        else:
                            print(f"[SERVO] Triggering UNLOCK sequence (Ch6/Servo6 Raw: {ch6})")
                            threading.Thread(target=self.perform_unlocking, daemon=True, name="UnlockSequenceThread").start()
                
                # Active Background Holding Loop
                # If a sequence is NOT active, re-enforce the target state positions at 10Hz
                if not self.sequence_active and self.last_state in ['lock', 'unlock']:
                    with self._io_lock:
                        if self.last_state == 'lock':
                            # Servo 1 (ST)
                            self._write1(1, STS_TORQUE_ENABLE, 1, "enable torque")
                            self.stsHandler.WritePosEx(1, LOCK_POS_1, ST_SPEED, ST_ACC)
                            # Servo 2 (SC)
                            self._write1(2, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(2, LOCK_POS_2, 0, SC_SPEED)
                            # Servo 3 (SC)
                            self._write1(3, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(3, LOCK_POS_3, 0, SC_SPEED)
                        elif self.last_state == 'unlock':
                            # Servo 1 (ST)
                            self._write1(1, STS_TORQUE_ENABLE, 1, "enable torque")
                            self.stsHandler.WritePosEx(1, UNLOCK_POS_1, ST_SPEED, ST_ACC)
                            # Servo 2 (SC)
                            self._write1(2, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(2, UNLOCK_POS_2, 0, SC_SPEED)
                            # Servo 3 (SC)
                            self._write1(3, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(3, UNLOCK_POS_3, 0, SC_SPEED)

            except Exception as e:
                print(f"[SERVO] Monitor loop error: {e}")
                traceback.print_exc()
            
            time.sleep(0.1) # 10Hz monitoring
