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
        try:
            import msvcrt
            if msvcrt.kbhit():
                msvcrt.getch()
                return True
        except Exception:
            pass
    else:
        try:
            import select
            import termios
            import tty
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                if rlist:
                    sys.stdin.read(1)
                    return True
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass
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

# =========================================================================
# CENTRALIZED SERVO SEQUENCE CONFIGURATION
# Matches STServo_Python/servo_sequences.json
# =========================================================================
# Servo ID 1 & 2 (ST3215 Dual Lid Lifters)
DEFAULT_LID1_LOCK_POS   = 1450
DEFAULT_LID1_UNLOCK_POS = 4000
DEFAULT_LID2_LOCK_POS   = 3450
DEFAULT_LID2_UNLOCK_POS = 500
DEFAULT_LID_SPEED       = 2400
DEFAULT_LID_ACC         = 50
DEFAULT_LID_TOLERANCE   = 70

# Servo ID 3 & 4 (SC09 Locking Latches / Kadi)
DEFAULT_LATCH3_LOCK_POS   = 520
DEFAULT_LATCH3_UNLOCK_POS = 675
DEFAULT_LATCH4_LOCK_POS   = 700
DEFAULT_LATCH4_UNLOCK_POS = 550
DEFAULT_LATCH_SPEED       = 1500

DEFAULT_SEQUENCE_CONFIG = {
    "lock": {
        "st1_pos": DEFAULT_LID1_LOCK_POS,
        "st2_pos": DEFAULT_LID2_LOCK_POS,
        "st_speed": DEFAULT_LID_SPEED,
        "st_acc": DEFAULT_LID_ACC,
        "st_tol": DEFAULT_LID_TOLERANCE,
        "sc3_pos": DEFAULT_LATCH3_LOCK_POS,
        "sc4_pos": DEFAULT_LATCH4_LOCK_POS,
        "sc_speed": DEFAULT_LATCH_SPEED
    },
    "unlock": {
        "st1_pos": DEFAULT_LID1_UNLOCK_POS,
        "st2_pos": DEFAULT_LID2_UNLOCK_POS,
        "st_speed": DEFAULT_LID_SPEED,
        "st_acc": DEFAULT_LID_ACC,
        "st_tol": DEFAULT_LID_TOLERANCE,
        "sc3_pos": DEFAULT_LATCH3_UNLOCK_POS,
        "sc4_pos": DEFAULT_LATCH4_UNLOCK_POS,
        "sc_speed": DEFAULT_LATCH_SPEED
    }
}

# Absolute Physical Mechanical Limits to prevent over-travel
SERVO_LIMITS = {
    1: (0, 4095),   # Servo 1 (ST3215 Dual Lid 1)
    2: (0, 4095),   # Servo 2 (ST3215 Dual Lid 2)
    3: (0, 1023),   # Servo 3 (SC09 Latch 3)
    4: (0, 1023)    # Servo 4 (SC09 Latch 4)
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
        
        self.st_ids = [1, 2]
        self.sc_ids = [3, 4]
        self.st3215_id = 1
        self.sc09_ids = [3, 4]
        self.st_config = {"min": 0, "max": 4095, "home": 0}
        self.sc09_configs = {3: {"min": 0, "max": 1023}, 4: {"min": 0, "max": 1023}}
        self.servo_protocols = {1: "sts", 2: "sts", 3: "scscl", 4: "scscl"}
        
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

    def load_sequence_config(self):
        """
        Loads sequence settings from servo_sequences.json if present,
        otherwise uses configured default constants.
        """
        import json
        config = json.loads(json.dumps(DEFAULT_SEQUENCE_CONFIG))
        possible_paths = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "STServo_Python", "servo_sequences.json")),
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "servo_sequences.json")),
            os.path.abspath(os.path.join(os.path.dirname(__file__), "servo_sequences.json")),
            os.path.abspath("servo_sequences.json"),
            os.path.abspath("STServo_Python/servo_sequences.json"),
        ]
        for path in possible_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        data = json.load(f)
                        if "lock" in data and isinstance(data["lock"], dict):
                            config["lock"].update(data["lock"])
                        if "unlock" in data and isinstance(data["unlock"], dict):
                            config["unlock"].update(data["unlock"])
                        break
                except Exception as e:
                    print(f"[SERVO] Warning reading sequence config from {path}: {e}")
        return config

    def detect_physical_state(self):
        """
        Detects physical lock state on reboot by reading magnetic encoder feedback of all 4 servos.
        Falls back to saved state file if serial communication fails.
        """
        if not self.connected:
            return self._load_saved_state()

        try:
            seq_cfg = self.load_sequence_config()
            lock_c = seq_cfg["lock"]
            unlock_c = seq_cfg["unlock"]
            
            l_st1 = lock_c.get("st1_pos", DEFAULT_LID1_LOCK_POS)
            l_st2 = lock_c.get("st2_pos", DEFAULT_LID2_LOCK_POS)
            l_sc3 = lock_c.get("sc3_pos", DEFAULT_LATCH3_LOCK_POS)
            l_sc4 = lock_c.get("sc4_pos", DEFAULT_LATCH4_LOCK_POS)

            u_st1 = unlock_c.get("st1_pos", DEFAULT_LID1_UNLOCK_POS)
            u_st2 = unlock_c.get("st2_pos", DEFAULT_LID2_UNLOCK_POS)
            u_sc3 = unlock_c.get("sc3_pos", DEFAULT_LATCH3_UNLOCK_POS)
            u_sc4 = unlock_c.get("sc4_pos", DEFAULT_LATCH4_UNLOCK_POS)

            with self._io_lock:
                pos1, r1, _ = self.stsHandler.ReadPos(1)
                pos2, r2, _ = self.stsHandler.ReadPos(2)
                pos3, r3, _ = self.scsHandler.ReadPos(3)
                pos4, r4, _ = self.scsHandler.ReadPos(4)

            score_lock = 0
            score_unlock = 0
            valid_reads = 0

            if r1 == COMM_SUCCESS and 0 <= pos1 <= 4095:
                if abs(pos1 - l_st1) < abs(pos1 - u_st1): score_lock += 1
                else: score_unlock += 1
                valid_reads += 1

            if r2 == COMM_SUCCESS and 0 <= pos2 <= 4095:
                if abs(pos2 - l_st2) < abs(pos2 - u_st2): score_lock += 1
                else: score_unlock += 1
                valid_reads += 1

            if r3 == COMM_SUCCESS and 0 <= pos3 <= 1023:
                if abs(pos3 - l_sc3) < abs(pos3 - u_sc3): score_lock += 1
                else: score_unlock += 1
                valid_reads += 1

            if r4 == COMM_SUCCESS and 0 <= pos4 <= 1023:
                if abs(pos4 - l_sc4) < abs(pos4 - u_sc4): score_lock += 1
                else: score_unlock += 1
                valid_reads += 1

            if valid_reads >= 2:
                detected = 'lock' if score_lock >= score_unlock else 'unlock'
                print(f"[SERVO] Hardware encoder check on boot: P1={pos1}, P2={pos2}, P3={pos3}, P4={pos4} -> Detected: '{detected}'")
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
        return int(sid) in self.st_ids

    def _servo_ids(self):
        return [1, 2, 3, 4]

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
        for sid in self.sc_ids:
            src = sc_src.get(sid, sc_src.get(str(sid), self.sc09_configs.get(sid, {})))
            self.sc09_configs[sid] = {
                "min": int(src.get("min", self.sc09_configs.get(sid, {}).get("min", 0))),
                "max": int(src.get("max", self.sc09_configs.get(sid, {}).get("max", 1023))),
            }

    def get_config(self):
        return {
            "st3215_1": dict(self.st_config),
            "st3215_2": dict(self.st_config),
            "sc09_3": dict(self.sc09_configs.get(3, {"min": 0, "max": 1023})),
            "sc09_4": dict(self.sc09_configs.get(4, {"min": 0, "max": 1023})),
            "st3215": dict(self.st_config),
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

        print(f"[SERVO] Initializing Servos (IDs: {self._servo_ids()})...")
        for sid in self._servo_ids():
            try:
                with self._io_lock:
                    handler = self._handler_for(sid)
                    is_sts = self._is_sts(sid)
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

                    time.sleep(0.05)
                    if not self._lock_eprom(sid):
                        continue

                    # Put ST servos into Position Control Mode (Mode 0) and enable holding torque
                    if is_sts:
                        self._write1(sid, STS_MODE, 0, "set position mode")
                        result, error = handler.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 1)
                    else:
                        result, error = handler.write1ByteTxRx(sid, SCSCL_TORQUE_ENABLE, 1)

                    if not self._result_ok(sid, "initialize torque state", result, error, handler):
                        continue

                print(f"[SERVO] Servo ID {sid} initialized in Position Mode.")
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

    def move_dual_lid_sync(self, target1, target2, speed=DEFAULT_LID_SPEED, acc=DEFAULT_LID_ACC, tolerance=DEFAULT_LID_TOLERANCE, label="DUAL LID MOTION"):
        """
        Synchronously moves Servo 1 and Servo 2 simultaneously using SyncWrite (or RegWrite+Action fallback).
        Monitors both encoders in real time until both servos reach their targets within tolerance threshold.
        """
        target1 = int(target1)
        target2 = int(target2)
        speed = int(speed)
        acc = int(acc)
        tolerance = int(tolerance)

        print(f"\n[SERVO] === {label}: SERVO 1 -> {target1} | SERVO 2 -> {target2} (SPEED: {speed}, TOL: ±{tolerance}) ===")

        with self._io_lock:
            self._write1(1, STS_MODE, 0, "set position mode")
            self._write1(2, STS_MODE, 0, "set position mode")
            self._write1(1, STS_TORQUE_ENABLE, 1, "enable torque")
            self._write1(2, STS_TORQUE_ENABLE, 1, "enable torque")

            pos1_start, r1, _ = self.stsHandler.ReadPos(1)
            pos2_start, r2, _ = self.stsHandler.ReadPos(2)
            p1_str = f"{pos1_start}" if r1 == COMM_SUCCESS else "ERR"
            p2_str = f"{pos2_start}" if r2 == COMM_SUCCESS else "ERR"
            print(f"[SERVO] Starting Positions -> Servo 1: {p1_str} | Servo 2: {p2_str}")

            self.stsHandler.SyncWritePosEx(1, target1, speed, acc)
            self.stsHandler.SyncWritePosEx(2, target2, speed, acc)
            res_sync = self.stsHandler.groupSyncWrite.txPacket()
            self.stsHandler.groupSyncWrite.clearParam()

            if res_sync != COMM_SUCCESS:
                self.stsHandler.RegWritePosEx(1, target1, speed, acc)
                self.stsHandler.RegWritePosEx(2, target2, speed, acc)
                self.stsHandler.RegAction()

        start_t = time.time()
        max_wait = 12.0
        last_p1 = pos1_start if r1 == COMM_SUCCESS else 0
        last_p2 = pos2_start if r2 == COMM_SUCCESS else 0
        stall_count = 0

        while time.time() - start_t < max_wait:
            if check_emergency_stop():
                print("\n[SERVO EMERGENCY STOP] Halting both servos immediately!")
                with self._io_lock:
                    self._write1(1, STS_TORQUE_ENABLE, 0, "disable torque")
                    self._write1(2, STS_TORQUE_ENABLE, 0, "disable torque")
                return False

            time.sleep(0.04)
            with self._io_lock:
                pos1_now, r1, _ = self.stsHandler.ReadPos(1)
                pos2_now, r2, _ = self.stsHandler.ReadPos(2)

            p1_done = (r1 == COMM_SUCCESS and abs(pos1_now - target1) <= tolerance)
            p2_done = (r2 == COMM_SUCCESS and abs(pos2_now - target2) <= tolerance)

            if p1_done and p2_done:
                break

            # Mechanical seated lid / resistance check near target
            if r1 == COMM_SUCCESS and r2 == COMM_SUCCESS:
                if abs(pos1_now - last_p1) < 4 and abs(pos2_now - last_p2) < 4:
                    stall_count += 1
                    if stall_count >= 10 and abs(pos1_now - target1) <= (tolerance + 50) and abs(pos2_now - target2) <= (tolerance + 50):
                        print(f"\n[SERVO] [LID SEATED] Mechanical limit reached (ID 1: {pos1_now}, ID 2: {pos2_now}). Proceeding...")
                        break
                else:
                    stall_count = 0
                last_p1 = pos1_now
                last_p2 = pos2_now

        with self._io_lock:
            pos1_fin, _, _ = self.stsHandler.ReadPos(1)
            pos2_fin, _, _ = self.stsHandler.ReadPos(2)
        print(f"[SERVO] [SYNC REACHED] Final Positions: Servo 1 = {pos1_fin} (Target: {target1}) | Servo 2 = {pos2_fin} (Target: {target2})")
        return True

    def move_latches(self, target3, target4, speed=DEFAULT_LATCH_SPEED, timeout=5.0):
        """
        Moves SC09 latch servos (ID 3 & ID 4) simultaneously.
        """
        target3 = int(target3)
        target4 = int(target4)
        speed = int(speed)

        print(f"[SERVO] Moving Latches -> Servo 3: {target3} | Servo 4: {target4} (Speed: {speed})")

        with self._io_lock:
            self.scsHandler.write1ByteTxRx(3, SCSCL_TORQUE_ENABLE, 1)
            self.scsHandler.WritePos(3, target3, 0, speed)
            self.scsHandler.write1ByteTxRx(4, SCSCL_TORQUE_ENABLE, 1)
            self.scsHandler.WritePos(4, target4, 0, speed)

        start_t = time.time()
        while time.time() - start_t < timeout:
            time.sleep(0.08)
            with self._io_lock:
                pos3, r3, _ = self.scsHandler.ReadPos(3)
                pos4, r4, _ = self.scsHandler.ReadPos(4)
            p3_ok = (r3 == COMM_SUCCESS and abs(pos3 - target3) <= 25)
            p4_ok = (r4 == COMM_SUCCESS and abs(pos4 - target4) <= 25)
            if p3_ok and p4_ok:
                break

        with self._io_lock:
            p3_fin, _, _ = self.scsHandler.ReadPos(3)
            p4_fin, _, _ = self.scsHandler.ReadPos(4)
        print(f"[SERVO] Latches Position -> Servo 3: {p3_fin} (Target {target3}) | Servo 4: {p4_fin} (Target {target4})")
        return True

    def perform_locking(self, force=False):
        """
        Execute 4-servo locking sequence:
        Step 1: Move Dual Lid DOWN synchronously (Servo 1 & Servo 2)
        Step 2: Engage Latches (Servo 3 & Servo 4)
        """
        self.sequence_active = True
        try:
            print("\n--- STARTING 4-SERVO LOCKING SEQUENCE ---")
            cfg = self.load_sequence_config()["lock"]
            st1_pos = cfg.get("st1_pos", DEFAULT_LID1_LOCK_POS)
            st2_pos = cfg.get("st2_pos", DEFAULT_LID2_LOCK_POS)
            st_spd = cfg.get("st_speed", DEFAULT_LID_SPEED)
            st_acc = cfg.get("st_acc", DEFAULT_LID_ACC)
            st_tol = cfg.get("st_tol", DEFAULT_LID_TOLERANCE)
            sc3_pos = cfg.get("sc3_pos", DEFAULT_LATCH3_LOCK_POS)
            sc4_pos = cfg.get("sc4_pos", DEFAULT_LATCH4_LOCK_POS)
            sc_spd = cfg.get("sc_speed", DEFAULT_LATCH_SPEED)

            if not force and self.last_state == 'lock':
                print("[SERVO SAFETY] Mechanism is ALREADY LOCKED (last_state='lock'). Re-verifying latches...")
                self.move_latches(sc3_pos, sc4_pos, sc_spd, timeout=2.0)
                return

            # Step 1: Move Dual Lid DOWN synchronously
            print(f"\nStep 1: Dual Lid DOWN -> ID 1: {st1_pos} & ID 2: {st2_pos} (Speed: {st_spd}, Tol: ±{st_tol})")
            self.move_dual_lid_sync(st1_pos, st2_pos, speed=st_spd, acc=st_acc, tolerance=st_tol, label="LOCK: DUAL LID DOWN")
            time.sleep(0.5)

            # Step 2: Engage Latches (SC servos 3 & 4)
            print(f"\nStep 2: Engaging Latches -> ID 3: {sc3_pos} & ID 4: {sc4_pos} (Speed: {sc_spd})")
            self.move_latches(sc3_pos, sc4_pos, speed=sc_spd, timeout=3.0)
            time.sleep(0.5)

            print("\nLocking sequence complete!")
            self.last_state = 'lock'
            self._save_state('lock')
        except Exception as e:
            print(f"[SERVO] Locking sequence failed: {e}")
            traceback.print_exc()
        finally:
            self.sequence_active = False

    def perform_unlocking(self, force=False):
        """
        Execute 4-servo unlocking sequence:
        Step 1: Retract Latches (Servo 3 & Servo 4)
        Step 2: Move Dual Lid UP synchronously (Servo 1 & Servo 2)
        """
        self.sequence_active = True
        try:
            print("\n--- STARTING 4-SERVO UNLOCKING SEQUENCE ---")
            cfg = self.load_sequence_config()["unlock"]
            st1_pos = cfg.get("st1_pos", DEFAULT_LID1_UNLOCK_POS)
            st2_pos = cfg.get("st2_pos", DEFAULT_LID2_UNLOCK_POS)
            st_spd = cfg.get("st_speed", DEFAULT_LID_SPEED)
            st_acc = cfg.get("st_acc", DEFAULT_LID_ACC)
            st_tol = cfg.get("st_tol", DEFAULT_LID_TOLERANCE)
            sc3_pos = cfg.get("sc3_pos", DEFAULT_LATCH3_UNLOCK_POS)
            sc4_pos = cfg.get("sc4_pos", DEFAULT_LATCH4_UNLOCK_POS)
            sc_spd = cfg.get("sc_speed", DEFAULT_LATCH_SPEED)

            if not force and self.last_state == 'unlock':
                print("[SERVO SAFETY] Mechanism is ALREADY UNLOCKED (last_state='unlock'). Skipping redundant unlock moves.")
                return

            # Step 1: Retract Latches (SC servos 3 & 4)
            print(f"\nStep 1: Retracting Latches -> ID 3: {sc3_pos} & ID 4: {sc4_pos} (Speed: {sc_spd})")
            self.move_latches(sc3_pos, sc4_pos, speed=sc_spd, timeout=3.0)
            time.sleep(0.5)

            # Step 2: Move Dual Lid UP synchronously
            print(f"\nStep 2: Dual Lid UP -> ID 1: {st1_pos} & ID 2: {st2_pos} (Speed: {st_spd}, Tol: ±{st_tol})")
            self.move_dual_lid_sync(st1_pos, st2_pos, speed=st_spd, acc=st_acc, tolerance=st_tol, label="UNLOCK: DUAL LID UP")
            time.sleep(0.5)

            print("\nUnlocking sequence complete!")
            self.last_state = 'unlock'
            self._save_state('unlock')
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
                

            except Exception as e:
                print(f"[SERVO] Monitor loop error: {e}")
                traceback.print_exc()
            
            time.sleep(0.1) # 10Hz monitoring
