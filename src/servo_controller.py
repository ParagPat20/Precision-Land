import sys
import os
import glob
import time
import threading
import traceback

# Add STServo SDK to the path
_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "STServo_Python"))
if _SDK_ROOT not in sys.path:
    sys.path.append(_SDK_ROOT)

_SDK_IMPORT_ERROR = None
try:
    from STservo_sdk import *
except ImportError:
    try:
        from STServo_Python.STservo_sdk import *
    except ImportError as e:
        _SDK_IMPORT_ERROR = e


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
        
        self._running = False
        self._thread = None
        self._io_lock = threading.Lock()
        
        # State tracking to avoid redundant writes
        self.st3215_locked = False
        self.sc09_locked = False

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
        if sid == self.st3215_id:
            return self.stsHandler
        if sid in self.sc09_ids:
            return self.scsHandler
        return self.stsHandler

    def _is_sts(self, sid):
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
        if self._is_sts(sid):
            return 2048
        return (int(cfg.get("min", 0)) + int(cfg.get("max", 1023))) // 2

    def _clamp_position(self, sid, position):
        cfg = self._config_for(sid)
        default_max = 4095 if self._is_sts(sid) else 1023
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
                    is_sts = sid == self.st3215_id
                    torque_addr = STS_TORQUE_ENABLE if is_sts else SCSCL_TORQUE_ENABLE
                    min_addr = STS_MIN_ANGLE_LIMIT_L if is_sts else SCSCL_MIN_ANGLE_LIMIT_L
                    max_addr = STS_MAX_ANGLE_LIMIT_L if is_sts else SCSCL_MAX_ANGLE_LIMIT_L
                    
                    if is_sts:
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

                    if is_sts:
                        val = handler.sts_toscs(home_offset, 11)
                        if not self._write2(sid, STS_OFS_L, val, "write home offset"):
                            self._lock_eprom(sid)
                            continue

                    time.sleep(0.1)
                    if not self._lock_eprom(sid):
                        continue

                    if is_sts:
                        if not self._write1(sid, STS_MODE, 0, "set position mode"):
                            continue
                        result, error = handler.WritePosEx(sid, self._home_position_for(sid), 2400, 50)
                    else:
                        result, error = handler.WritePos(sid, self._home_position_for(sid), 0, 500)
                    if not self._result_ok(sid, "move to center", result, error, handler):
                        continue

                print(f"[SERVO] Servo ID {sid} initialized.")
            except Exception as e:
                print(f"[SERVO] Error initializing servo ID {sid}: {e}")
                # Prevent "Port is in use!" subsequent errors
                if hasattr(self, "portHandler") and self.portHandler:
                    self.portHandler.is_using = False

    def set_torque(self, sid, enable):
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        sid = int(sid)
        state = 1 if enable else 0
        try:
            with self._io_lock:
                torque_addr = STS_TORQUE_ENABLE if sid == self.st3215_id else SCSCL_TORQUE_ENABLE
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
            if self._is_sts(item):
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
                if is_sts:
                    ofs = self._read2(sid, STS_OFS_L)
                    payload["home_offset"] = None if ofs is None else handler.sts_tohost(ofs, 11)
                    payload["mode"] = self._read1(sid, STS_MODE)
                return payload
        except Exception as e:
            if self.portHandler:
                self.portHandler.is_using = False
            return {"ok": False, "id": sid, "error": str(e)}

    def start_monitoring(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True, name="ServoMonitor")
        self._thread.start()
        print("[SERVO] Servo RC channel monitoring started.")

    def stop_monitoring(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _monitor_loop(self):
        while self._running:
            try:
                if self.vehicle and self.vehicle.channels:
                    # Check if mission is running
                    mission_running = False
                    if self.is_mission_active_cb:
                        mission_running = self.is_mission_active_cb()
                    else:
                        # Fallback: check if vehicle is in AUTO mode
                        mission_running = self.vehicle.mode.name == 'AUTO'
                    
                    if mission_running:
                        # Read channel 6 and 7
                        ch6 = self.vehicle.channels.get('6', 0)
                        ch7 = self.vehicle.channels.get('7', 0)
                        
                        # Channel 6 controls SC09 (IDs 2 & 3)
                        sc09_should_lock = ch6 > 1500
                        if sc09_should_lock != self.sc09_locked:
                            for sid in self.sc09_ids:
                                self.set_torque(sid, sc09_should_lock)
                            self.sc09_locked = sc09_should_lock
                            state_str = "LOCKED" if sc09_should_lock else "UNLOCKED"
                            print(f"[SERVO] SC09 Servos (IDs {self.sc09_ids}) {state_str} (CH6: {ch6})")
                            
                        # Channel 7 controls ST3215 (ID 1)
                        st3215_should_lock = ch7 > 1500
                        if st3215_should_lock != self.st3215_locked:
                            self.set_torque(self.st3215_id, st3215_should_lock)
                            self.st3215_locked = st3215_should_lock
                            state_str = "LOCKED" if st3215_should_lock else "UNLOCKED"
                            print(f"[SERVO] ST3215 Servo (ID {self.st3215_id}) {state_str} (CH7: {ch7})")
            except Exception as e:
                print(f"[SERVO] Monitor loop error: {e}")
                traceback.print_exc()
            
            time.sleep(0.1) # 10Hz monitoring
