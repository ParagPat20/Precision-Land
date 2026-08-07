import sys
import os
import glob
import time
import threading
import traceback
import math

def check_emergency_stop():
    """
    Non-blocking check for keyboard keypress (Spacebar, 'q', 's', ESC, or any key) on Windows/Linux.
    Allows instant emergency stop during active motor movement loops.
    """
    if os.name == 'nt':
        import msvcrt
        if msvcrt.kbhit():
            try:
                msvcrt.getch()
            except Exception:
                pass
            return True
    return False

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

# --- CONFIGURATION CONSTANTS FOR LOCK SEQUENCE ONLY ---
# Speeds & Accel
ST_SPEED = 2400
ST_ACC = 200
SC_SPEED = 1500

# Locking Targets
LOCK_POS_1 = "Continuous Rotation (Forward Speed 3000, 2 Rollover Laps -> Absolute Snap to 3000)"
LOCK_POS_2 = 550
LOCK_POS_3 = 715

# Unlocking Targets
UNLOCK_POS_1 = "Continuous Rotation (Backward Speed 3000, 2 Rollover Laps -> Absolute Snap to 3000)"
UNLOCK_POS_2 = 750
UNLOCK_POS_3 = 540
UNLOCK_CHECK_3 = 540  # Threshold check for ID 3

# Servo 1 (ST3215) Continuous Rotation & Absolute Target Parameters
ST_LOCK_SPEED_1 = 3000
ST_LOCK_ROTATIONS_1 = 2.0  # 2 rollover laps (crossing 4096 -> 0)
ST_LOCK_ABS_TARGET_1 = 3000
ST_UNLOCK_SPEED_1 = 3000
ST_UNLOCK_ROTATIONS_1 = 2.0  # 2 rollover laps (crossing 0 -> 4096)
ST_UNLOCK_ABS_TARGET_1 = 3000

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
    
    PWM Channel 6 mapping:
    - HIGH (> 1500): Unlocking sequence
    - LOW (<= 1500): Locking sequence
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

        # State file for persistence across Pi reboots
        self.state_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "servo_state.json"))
        
        # Lock sequence state variables
        self.servo6_raw = 0
        self.last_servo6_raw_rx_time = 0
        self.last_stream_request_time = 0
        self.sequence_active = False
        
        saved_st = self._load_saved_state()
        self.last_triggered_state = saved_st
        self.last_state = saved_st

        # Register MAVLink message listener for Servo Output Channel 6
        if self.vehicle:
            try:
                self.vehicle.add_message_listener('SERVO_OUTPUT_RAW', self._servo_output_listener)
                print("[SERVO] Registered SERVO_OUTPUT_RAW listener for Channel 6.")
            except Exception as e:
                print(f"[SERVO] Warning: Failed to add SERVO_OUTPUT_RAW message listener: {e}")

        self._connect()

    def _load_saved_state(self):
        try:
            if os.path.exists(self.state_file):
                import json
                with open(self.state_file, 'r') as f:
                    data = json.load(f)
                    st = data.get("last_state", "lock")
                    if st in ["lock", "unlock"]:
                        return st
        except Exception as e:
            print(f"[SERVO] Note: Failed to read state file {self.state_file}: {e}")
        return "lock"

    def _save_state(self, state):
        try:
            import json
            data = {"last_state": state, "updated_at": time.time()}
            with open(self.state_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"[SERVO] Warning: Failed to write state file {self.state_file}: {e}")

    def detect_physical_state(self):
        """
        Detects physical lock state on reboot by reading magnetic encoder feedback of SC servos (IDs 2 & 3).
        Falls back to saved state file if serial communication fails.
        """
        if not self.connected:
            return self._load_saved_state()

        try:
            with self._io_lock:
                pos2, res2, _ = self.scsHandler.ReadPos(2)
                pos3, res3, _ = self.scsHandler.ReadPos(3)

            if res2 == COMM_SUCCESS and res3 == COMM_SUCCESS:
                # Calculate proximity to LOCK positions vs UNLOCK positions
                dist_lock = abs(pos2 - LOCK_POS_2) + abs(pos3 - LOCK_POS_3)
                dist_unlock = abs(pos2 - UNLOCK_POS_2) + abs(pos3 - UNLOCK_POS_3)
                
                detected = 'lock' if dist_lock <= dist_unlock else 'unlock'
                print(f"[SERVO] Hardware encoder check on boot: Pos2={pos2}, Pos3={pos3} -> Detected state: '{detected}'")
                self._save_state(detected)
                return detected
        except Exception as e:
            print(f"[SERVO] Note: Encoder check fallback to state file: {e}")

        return self._load_saved_state()

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

                    # Enable holding torque & mode on startup without moving Servos 2 & 3 prematurely
                    if sid == 1:
                        if not self._write1(sid, STS_MODE, 1, "set wheel mode"):
                            continue
                        result, error = handler.WriteSpec(sid, 0, 50)
                    else:
                        result, error = handler.write1ByteTxRx(sid, SCSCL_TORQUE_ENABLE, 1)

                    if not self._result_ok(sid, "initialize torque state", result, error, handler):
                        continue

                print(f"[SERVO] Servo ID {sid} initialized.")
            except Exception as e:
                print(f"[SERVO] Error initializing servo ID {sid}: {e}")
                if hasattr(self, "portHandler") and self.portHandler:
                    self.portHandler.is_using = False

        # Detect current physical state on boot/restart using magnetic encoders & state file
        boot_state = self.detect_physical_state()
        self.last_state = boot_state
        self.last_triggered_state = boot_state

        # Startup sequence: Perform state-aware locking check
        def startup_locking_thread():
            print(f"[SERVO] Startup state check: Mechanism is '{self.last_state}'. Verifying lock state...")
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

    def rotate_st_continuous(self, sid=1, direction='f', speed=3000, rotations=1.0, abs_target=3000):
        """
        Rotates ST3215 servo (ID 1) in continuous Wheel Mode by counting 0-4096 rollover laps.
        When target rollover count is reached, it monitors the encoder until arriving at abs_target (e.g. 3000),
        then switches to Position Control Mode (33 -> 0) and locks absolute position with zero drift!
        - direction: 'f' (Forward/CW) or 'b' (Backward/CCW)
        - speed: speed magnitude (e.g. 3000)
        - rotations: target rollover count (1.0 = 1 full lap crossing 4096/0)
        - abs_target: absolute encoder position (0-4095)
        """
        sid = int(sid)
        is_forward = str(direction).lower() in ['f', 'for', 'forward', 'cw', '1']
        sign = 1 if is_forward else -1
        speed_val = abs(int(speed)) * sign
        dir_label = "Forward (CW)" if is_forward else "Backward (CCW)"
        
        target_rollovers = max(1, int(round(float(rotations))))
        
        print(f"[SERVO] Rotating Servo {sid} ({dir_label}): speed {abs(speed)}, target rollovers {target_rollovers}, final absolute target position {abs_target}...")
        
        with self._io_lock:
            self._write1(sid, STS_TORQUE_ENABLE, 1, "enable torque")
            pos_start, spd_start, res, error = self.stsHandler.ReadPosSpeed(sid)
            if res != COMM_SUCCESS:
                print(f"[SERVO] Error reading initial position for Servo {sid}: {self.stsHandler.getTxRxResult(res)}")
                return False
                
            self.stsHandler.WheelMode(sid)
            res_spec, err_spec = self.stsHandler.WriteSpec(sid, speed_val, 50)
            if res_spec != COMM_SUCCESS:
                print(f"[SERVO] Error starting rotation on Servo {sid}: {self.stsHandler.getTxRxResult(res_spec)}")
                return False

        last_pos = pos_start
        rollover_count = 0
        start_t = time.time()
        max_timeout = max(12.0, target_rollovers * 10.0)
        
        while True:
            if check_emergency_stop():
                print(f"[SERVO] EMERGENCY STOP TRIGGERED BY USER KEYPRESS! Halting Servo {sid}!")
                with self._io_lock:
                    self.stsHandler.write1ByteTxRx(254, 40, 0)
                    self.stsHandler.WriteSpec(sid, 0, 50)
                return False

            if (time.time() - start_t) > max_timeout:
                print(f"[SERVO] Safety timeout ({max_timeout:.1f}s) reached during ID {sid} rotation!")
                break
                
            time.sleep(0.005)
            
            with self._io_lock:
                curr_pos, _, res_read, _ = self.stsHandler.ReadPosSpeed(sid)
                
            if res_read == COMM_SUCCESS and 0 <= curr_pos <= 4095:
                # Detect boundary rollover
                if is_forward:
                    # Forward rollover: last_pos near 4096 (e.g. > 2500) and curr_pos near 0 (e.g. < 1500)
                    if last_pos > 2500 and curr_pos < 1500:
                        rollover_count += 1
                        print(f"[SERVO] Rollover #{rollover_count} detected (Forward 4096 -> 0)")
                    
                    # If target rollovers reached, check arrival at absolute target position
                    if rollover_count >= target_rollovers:
                        if abs_target is None or curr_pos >= (int(abs_target) - 60):
                            print(f"[SERVO] Rollover {rollover_count} complete & target absolute position reached: Pos {curr_pos} >= {abs_target}")
                            break
                else:
                    # Backward rollover: last_pos near 0 (e.g. < 1500) and curr_pos near 4096 (e.g. > 2500)
                    if last_pos < 1500 and curr_pos > 2500:
                        rollover_count += 1
                        print(f"[SERVO] Rollover #{rollover_count} detected (Backward 0 -> 4096)")
                    
                    # If target rollovers reached, check arrival at absolute target position
                    if rollover_count >= target_rollovers:
                        if abs_target is None or curr_pos <= (int(abs_target) + 60):
                            print(f"[SERVO] Rollover {rollover_count} complete & target absolute position reached: Pos {curr_pos} <= {abs_target}")
                            break
                            
                last_pos = curr_pos

        # Stop wheel rotation
        with self._io_lock:
            self.stsHandler.WriteSpec(sid, 0, 50)
            
        print(f"[SERVO] Servo {sid} continuous wheel rotation completed {rollover_count} rollover laps.")
        
        # Absolute Position Lock (Mode 0)
        if abs_target is not None:
            abs_target = int(abs_target)
            print(f"[SERVO] Snapping Servo {sid} to Absolute Target {abs_target} (switching to Position Mode 33->0)...")
            with self._io_lock:
                self._write1(sid, STS_MODE, 0, "set position mode")
                self._write1(sid, STS_TORQUE_ENABLE, 1, "enable torque")
                self.stsHandler.WritePosEx(sid, abs_target, 2400, 50)
            
            snap_start_t = time.time()
            final_p = -1
            while time.time() - snap_start_t < 2.5:
                time.sleep(0.05)
                with self._io_lock:
                    p, _, r_snap, _ = self.stsHandler.ReadPosSpeed(sid)
                if r_snap == COMM_SUCCESS:
                    final_p = p
                    if abs(p - abs_target) <= 30:
                        break
            print(f"[SERVO] Servo {sid} absolute position locked: Final Encoder Pos = {final_p} (Target = {abs_target})")

        return True

    def perform_locking(self, force=False):
        """
        Execute locking sequence.
        If force is False and mechanism is already locked, skips Servo 1 continuous rotation
        to prevent over-rewinding or straining the cable/motor.
        """
        self.sequence_active = True
        try:
            print("\n--- STARTING NATIVE LOCKING SEQUENCE ---")
            
            if not force:
                if self.last_state == 'lock':
                    print("[SERVO SAFETY] Mechanism is ALREADY LOCKED (last_state='lock'). Skipping lock sequence to prevent cable strain.")
                    with self._io_lock:
                        self._write1(1, STS_TORQUE_ENABLE, 1, "enable torque")
                        self.stsHandler.WriteSpec(1, 0, 50)
                    self.robust_move_sc_pair(2, LOCK_POS_2, 3, LOCK_POS_3, SC_SPEED, check_target3=LOCK_POS_3, check_dir3='>=')
                    return
                    
                with self._io_lock:
                    pos3, res3, _ = self.scsHandler.ReadPos(3)
                if res3 == COMM_SUCCESS and pos3 >= (LOCK_POS_3 - 50):
                    print(f"[SERVO SAFETY] Servo 3 live position ({pos3}) indicates lid is ALREADY CLOSED/LOCKED (>= {LOCK_POS_3 - 50})!")
                    print("[SERVO SAFETY] Skipping Servo 1 rotation to prevent over-tightening or cable breakage.")
                    self.robust_move_sc_pair(2, LOCK_POS_2, 3, LOCK_POS_3, SC_SPEED, check_target3=LOCK_POS_3, check_dir3='>=')
                    self.last_state = 'lock'
                    return

            print(f"Step 0: Pre-Lock Safety Check -> Ensuring Latches (Servo 2 & 3) are in UNLOCKED position ({UNLOCK_POS_2} & {UNLOCK_POS_3}) so they do not interfere with Servo 1...")
            self.robust_move_sc_single(3, UNLOCK_POS_3, SC_SPEED)
            self.robust_move_sc_single(2, UNLOCK_POS_2, SC_SPEED)
            
            print(f"\nStep 1: Servo 1 (ST) -> Continuous Rotation Forward Speed {ST_LOCK_SPEED_1}, Rotations {ST_LOCK_ROTATIONS_1}, Snap to {ST_LOCK_ABS_TARGET_1}")
            self.rotate_st_continuous(1, direction='f', speed=ST_LOCK_SPEED_1, rotations=ST_LOCK_ROTATIONS_1, abs_target=ST_LOCK_ABS_TARGET_1)
            
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

    def perform_unlocking(self, force=False):
        """
        Execute unlocking sequence.
        If force is False and mechanism is already unlocked, skips redundant moves.
        """
        self.sequence_active = True
        try:
            print("\n--- STARTING NATIVE UNLOCKING SEQUENCE ---")
            
            if self.last_state == 'unlock' and not force:
                print("[SERVO] Mechanism is ALREADY UNLOCKED. Skipping redundant unlock rotation.")
                with self._io_lock:
                    self._write1(1, STS_TORQUE_ENABLE, 1, "enable torque")
                    self.stsHandler.WriteSpec(1, 0, 50)
                self.robust_move_sc_single(3, UNLOCK_POS_3, SC_SPEED)
                self.robust_move_sc_single(2, UNLOCK_POS_2, SC_SPEED)
                return

            print(f"Step 1: Servo 3 (SC) -> {UNLOCK_POS_3}")
            self.robust_move_sc_single(3, UNLOCK_POS_3, SC_SPEED)
            
            print(f"\nStep 2: Servo 2 (SC) -> {UNLOCK_POS_2}")
            self.robust_move_sc_single(2, UNLOCK_POS_2, SC_SPEED)
            
            print(f"\nStep 3: Servo 1 (ST) -> Continuous Rotation Backward Speed {ST_UNLOCK_SPEED_1}, Rotations {ST_UNLOCK_ROTATIONS_1}, Snap to {ST_UNLOCK_ABS_TARGET_1}")
            self.rotate_st_continuous(1, direction='b', speed=ST_UNLOCK_SPEED_1, rotations=ST_UNLOCK_ROTATIONS_1, abs_target=ST_UNLOCK_ABS_TARGET_1)
            
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
        """Monitor loop - UNLOCK when PWM > 1500 (HIGH), LOCK when PWM <= 1500 (LOW)."""
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
                    # HIGH (> 1500) = UNLOCK, LOW (<= 1500) = LOCK
                    target_state = 'unlock' if ch6 > 1500 else 'lock'
                    
                    if target_state != self.last_triggered_state and not self.sequence_active:
                        self.last_triggered_state = target_state
                        if target_state == 'unlock':
                            print(f"[SERVO] Triggering UNLOCK sequence (Ch6/Servo6 Raw: {ch6} - HIGH)")
                            threading.Thread(target=self.perform_unlocking, daemon=True, name="UnlockSequenceThread").start()
                        else:
                            print(f"[SERVO] Triggering LOCK sequence (Ch6/Servo6 Raw: {ch6} - LOW)")
                            threading.Thread(target=self.perform_locking, daemon=True, name="LockSequenceThread").start()
                
                # Active Background Holding Loop
                # If a sequence is NOT active, re-enforce the target state positions at 10Hz
                if not self.sequence_active and self.last_state in ['lock', 'unlock']:
                    with self._io_lock:
                        # Servo 1 (ST) - Hold torque and zero speed in continuous wheel mode
                        self._write1(1, STS_TORQUE_ENABLE, 1, "enable torque")
                        self.stsHandler.WriteSpec(1, 0, 50)

                        if self.last_state == 'lock':
                            # Servo 2 (SC)
                            self._write1(2, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(2, LOCK_POS_2, 0, SC_SPEED)
                            # Servo 3 (SC)
                            self._write1(3, SCSCL_TORQUE_ENABLE, 1, "enable torque")
                            self.scsHandler.WritePos(3, LOCK_POS_3, 0, SC_SPEED)
                        elif self.last_state == 'unlock':
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
