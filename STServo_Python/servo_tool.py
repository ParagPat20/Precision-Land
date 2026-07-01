#!/usr/bin/env python
#
# *********     ST/SC Servo Advanced All-in-One Dashboard      *********
#
# A comprehensive interactive CLI tool to control and monitor ST and SC series servos.
# Supports numeric menu choices, fuzzy command shell entries, and teach-playback recording.
#

import sys
import os
import time
import json
import glob

sys.path.append("..")
from STservo_sdk import *

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

# Default Settings
BAUDRATE = 1000000
DEVICENAME = resolve_servo_port()

# ANSI Colors for UI
C_RED = '\033[91m'
C_GREEN = '\033[92m'
C_YEL = '\033[93m'
C_BLUE = '\033[94m'
C_CYA = '\033[96m'
C_RST = '\033[0m'

# Caches for detected servo types and models to optimize communication
detected_types = {}  # sid -> 'ST' or 'SC'
detected_models = {} # sid -> model_number

# Keyboard interaction setup
try:
    if os.name == 'nt':
        import msvcrt
        def check_key():
            if msvcrt.kbhit():
                try:
                    return msvcrt.getch().decode('utf-8').lower()
                except:
                    return None
            return None
    else:
        import select
        import termios
        import tty
        def check_key():
            fd = sys.stdin.fileno()
            try:
                old_settings = termios.tcgetattr(fd)
            except:
                return None
            try:
                tty.setraw(fd)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                if rlist:
                    return sys.stdin.read(1).lower()
            except:
                return None
            finally:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                except:
                    pass
            return None
except:
    def check_key():
        return None

def print_header(title):
    print(f"\n{C_CYA}┌────────────────────────────────────────────────────────────┐{C_RST}")
    print(f"{C_CYA}│{C_YEL}{title.center(58)}{C_CYA}│{C_RST}")
    print(f"{C_CYA}└────────────────────────────────────────────────────────────┘{C_RST}")

def draw_boxed_header(active_id):
    print(f"{C_CYA}┌────────────────────────────────────────────────────────────┐{C_RST}")
    print(f"{C_CYA}│{C_YEL}           ST/SC SERVO ADVANCED CONTROL DASHBOARD           {C_CYA}│{C_RST}")
    print(f"{C_CYA}├────────────────────────────────────────────────────────────┤{C_RST}")
    
    if active_id is not None:
        stype = detected_types.get(active_id, 'ST')
        m_num = detected_models.get(active_id, 0)
        info_str = f"Active Servo: ID {active_id} | Type: {stype} | Model: {m_num}"
    else:
        info_str = "Active Servo: None (Select ID or type 'act <id>')"
        
    print(f"{C_CYA}│{C_GREEN}  {info_str:<56}{C_CYA}│{C_RST}")
    print(f"{C_CYA}└────────────────────────────────────────────────────────────┘{C_RST}")

def draw_menu():
    print(f"{C_BLUE}┌── Core Functions ──────────────────────────────────────────┐{C_RST}")
    print(f"{C_BLUE}│{C_RST}  1. Live Monitor Dashboard   2. Select Active Servo        {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}│{C_RST}  3. Scan for Servos (Fast)                                  {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}├── Movement & Control ──────────────────────────────────────┤{C_RST}")
    print(f"{C_BLUE}│{C_RST}  4. Absolute Move            5. Center Align (Mid Point)    {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}│{C_RST}  6. Continuous Rotation      7. Relative Jog (Step)         {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}│{C_RST}  8. Toggle Torque            9. Emergency Relax (All Off)   {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}├── Config & Teach-Play ─────────────────────────────────────┤{C_RST}")
    print(f"{C_BLUE}│{C_RST} 10. Read Device Config      11. Set Servo ID               {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}│{C_RST} 12. Set Angle Limits        13. Set Position Offset (ST)   {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}│{C_RST} 14. Teach-Playback Recorder                                 {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}├── Advanced & Debug ────────────────────────────────────────┤{C_RST}")
    print(f"{C_BLUE}│{C_RST} 15. Advanced Debugging      0. Exit Dashboard              {C_BLUE}│{C_RST}")
    print(f"{C_BLUE}└────────────────────────────────────────────────────────────┘{C_RST}")
    
    print(f"\n{C_YEL}┌── Direct Command Shell Reference ──────────────────────────┐{C_RST}")
    print(f"{C_YEL}│{C_RST}  act <id>                     │ move [id] <pos> [spd] [acc] {C_YEL}│{C_RST}")
    print(f"{C_YEL}│{C_RST}  ping [id/all]                │ rotate [id] <f/b> [spd/pwm] {C_YEL}│{C_RST}")
    print(f"{C_YEL}│{C_RST}  center [id]                  │ torque [id] <on/off/1/0>    {C_YEL}│{C_RST}")
    print(f"{C_YEL}│{C_RST}  relax                        │ diag [id]                   {C_YEL}│{C_RST}")
    print(f"{C_YEL}│{C_RST}  record <id1,id2,...> [file]  │ play [file] / mon [ids]     {C_YEL}│{C_RST}")
    print(f"{C_YEL}└────────────────────────────────────────────────────────────┘{C_RST}")

def get_int(prompt, default=None, min_val=None, max_val=None):
    """
    Robust input helper.
    - Pressing Enter returns the default (if provided), otherwise cancels (returns None).
    - Entering 'b' or 'back' cancels and returns None.
    - Validates bounds and data types.
    """
    while True:
        try:
            val = input(f"{prompt}").strip()
            if val.lower() in ['b', 'back', 'q', 'quit']:
                return None
            if not val:
                if default is not None:
                    return default
                else:
                    return None
            num = int(val)
            if min_val is not None and num < min_val:
                print(f"{C_RED}Value must be >= {min_val}. Try again.{C_RST}")
                continue
            if max_val is not None and num > max_val:
                print(f"{C_RED}Value must be <= {max_val}. Try again.{C_RST}")
                continue
            return num
        except ValueError:
            print(f"{C_RED}Invalid input. Enter a valid number, 'b' to go back, or press Enter.{C_RST}")

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

# Known models for ST and SC series servos
KNOWN_ST_MODELS = {3215, 3015, 3020, 4315, 4020, 80, 10, 777}
KNOWN_SC_MODELS = {15, 9, 230, 309, 1029, 1284}

# Calibration offsets (e.g., temperature offsets in Celsius)
# SC09 (SCS0009) is known to report temperatures about 15C too high due to internal self-heating / baseline calibration.
TEMP_OFFSETS = {
    3: -75,  # Temperature offset for ID 3 (SC09) to correct its faulty sensor baseline
}

def get_stall_torque(servo_type, model, voltage):
    """
    Returns estimated stall torque in kg.cm based on servo model and measured voltage.
    """
    if servo_type == 'ST':
        if model == 3215:
            # STS3215 has 12V version (30 kg.cm) and 7.4V version (19.5 kg.cm)
            if voltage >= 9.0:
                return 30.0 * (voltage / 12.0)
            else:
                return 19.5 * (voltage / 7.4)
        # Default ST fallback (usually STS3215 class or similar)
        return 19.5 * (voltage / 7.4)
    else: # 'SC' series
        if model == 9: # SCS0009 (SC09)
            return 2.3 * (voltage / 6.0)
        elif model == 15: # SCS15 / SC15
            return 15.0 * (voltage / 7.4)
        # Default SC fallback
        return 10.0 * (voltage / 6.0)

def detect_servo_type(byte0, byte1):
    """
    Decides if a servo is ST (little-endian) or SC (big-endian) based on model registers.
    """
    # Direct mappings for known models based on raw byte register values
    # STS3215: Register 3 = 9, Register 4 = 3 (Combined ST value = 777)
    if byte0 == 9 and byte1 == 3:
        return 'ST', 3215
    # SCS0009 (SC09): Register 3 = 5, Register 4 = 4 (Combined ST value = 1029)
    if byte0 == 5 and byte1 == 4:
        return 'SC', 9
        
    model_st = byte0 | (byte1 << 8)
    model_sc = byte1 | (byte0 << 8)
    
    if model_st in KNOWN_ST_MODELS:
        return 'ST', model_st
    if model_sc in KNOWN_SC_MODELS:
        return 'SC', model_sc
        
    # Heuristic ranges: ST models are usually 1000-9999; SC models are 1-999
    is_st_range = 1000 <= model_st <= 9999
    is_sc_range = 1 <= model_sc <= 999
    
    if is_st_range and not is_sc_range:
        return 'ST', model_st
    if is_sc_range and not is_st_range:
        return 'SC', model_sc
        
    # Byte heuristic fallback
    if byte0 <= 3 and (byte1 >= 10 or byte1 == 0):
        return 'SC', model_sc
    if byte1 >= 10 and byte1 <= 25:
        return 'ST', model_st
        
    # Final default
    if model_st > 1000:
        return 'ST', model_st
    else:
        return 'SC', model_sc

def ping_and_detect(sts_handler, sid):
    """
    Pings a servo and determines its series (ST/SC) and model number.
    """
    # Send base ping packet (this is standard across protocols)
    _, result, error = sts_handler.ping(sid)
    if result != COMM_SUCCESS:
        return None, None, result
        
    # Read address 3 (model number, 2 bytes) to detect endianness
    data_read, result, error = sts_handler.readTxRx(sid, 3, 2)
    if result != COMM_SUCCESS:
        return None, None, result
        
    servo_type, model = detect_servo_type(data_read[0], data_read[1])
    return servo_type, model, COMM_SUCCESS

def get_servo_type(sts_handler, sid):
    """
    Quick helper to retrieve or auto-detect a servo's type.
    """
    if sid in detected_types:
        return detected_types[sid]
    servo_type, model, res = ping_and_detect(sts_handler, sid)
    if res == COMM_SUCCESS:
        detected_types[sid] = servo_type
        detected_models[sid] = model
        return servo_type
    return 'ST'  # Default fallback

def select_port():
    """
    Scans available COM ports and prompts the user to select one.
    """
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print(f"{C_RED}No serial ports detected!{C_RST}")
        port = input("Enter port name manually (or 'q' to quit): ").strip()
        if port.lower() == 'q':
            sys.exit(0)
        return port
        
    print(f"\n{C_BLUE}--- Detected Serial Ports ---{C_RST}")
    for idx, p in enumerate(ports):
        print(f" {idx + 1}. {p.device} - {p.description}")
    print(f" {len(ports) + 1}. Enter custom port name")
    print(f" 0. Quit")
    
    while True:
        choice = input(f"Select a port (0-{len(ports) + 1}): ").strip()
        if choice == '0':
            sys.exit(0)
        if choice == str(len(ports) + 1):
            custom = input("Enter custom port name: ").strip()
            if custom:
                return custom
        try:
            val = int(choice)
            if 1 <= val <= len(ports):
                return ports[val - 1].device
        except ValueError:
            pass
        print(f"{C_RED}Invalid choice. Try again.{C_RST}")

def select_active_servo(sts_handler):
    """
    Interactive tool to change the current active servo ID.
    """
    print_header("Select Active Servo")
    print("Enter a Servo ID to make it the default for all dashboard commands.")
    print("The system will ping the ID and auto-detect if it is ST or SC series.\n")
    
    sid = get_int("Enter Servo ID (0-253) [or Enter/b to cancel]: ", min_val=0, max_val=253)
    if sid is None:
        return None
        
    servo_type, model, res = ping_and_detect(sts_handler, sid)
    if res == COMM_SUCCESS:
        detected_types[sid] = servo_type
        detected_models[sid] = model
        print(f"\n{C_GREEN}Success! Servo {sid} is online. Type: {servo_type}, Model: {model}{C_RST}")
        time.sleep(1.5)
        return sid
    else:
        print(f"\n{C_RED}Warning: Could not connect to Servo {sid}. ({sts_handler.getTxRxResult(res)}){C_RST}")
        confirm = input("Select it anyway? (y/n) [Default: n]: ").strip().lower()
        if confirm == 'y':
            detected_types[sid] = 'ST'  # Fallback assumption
            detected_models[sid] = 0
            return sid
        return None

def diagnose_torque_issue(sts_handler, sc_handler, sid):
    """
    Reads all relevant safety, limits, and error registers to diagnose why a servo's torque won't lock.
    """
    servo_type = get_servo_type(sts_handler, sid)
    handler = sts_handler if servo_type == 'ST' else sc_handler
    
    print(f"\n{C_YEL}┌────────────────────────────────────────────────────────────┐")
    print(f"│                  Torque Diagnostics (ID {sid:2d})                 │")
    print(f"└────────────────────────────────────────────────────────────┘{C_RST}")
    
    model = detected_models.get(sid, 0)
    if model == 0:
        # Try pinging
        model_num, res, _ = sts_handler.ping(sid)
        if res != COMM_SUCCESS:
            print(f"{C_RED}[ERROR] Servo {sid} is offline or not responding to Ping.{C_RST}")
            print("Guidance: Check physical connections, power supply, and baudrate.")
            return
            
    # Read key registers
    torque, res, _ = handler.read1ByteTxRx(sid, 40)
    if res != COMM_SUCCESS:
        print(f"{C_RED}[ERROR] Failed to read Torque Enable register (Reg 40).{C_RST}")
        return
        
    error_val, _, _ = handler.read1ByteTxRx(sid, 65)
    voltage, _, _ = handler.read1ByteTxRx(sid, 62)
    temp, _, _ = handler.read1ByteTxRx(sid, 63)
    unloading, _, _ = handler.read1ByteTxRx(sid, 19)
    
    # Max Torque / Torque Limit register address is 16 for SC/ST
    torque_limit, _, _ = handler.read2ByteTxRx(sid, 16)
    
    # Angle limits
    min_ang, _, _ = handler.read2ByteTxRx(sid, 9)
    max_ang, _, _ = handler.read2ByteTxRx(sid, 11)
    
    # Voltage/Temperature Limits
    max_volt, _, _ = handler.read1ByteTxRx(sid, 14)
    min_volt, _, _ = handler.read1ByteTxRx(sid, 15)
    max_temp_lim, _, _ = handler.read1ByteTxRx(sid, 13)
    
    print(f"  - Torque Enable State       : {torque} ({'Enabled/Locked' if torque > 0 else 'Disabled/Limp'})")
    print(f"  - Max Torque Limit (Reg 16) : {torque_limit}")
    print(f"  - Present Error (Reg 65)    : {error_val} (Bin: {bin(error_val)})")
    print(f"  - Measured Voltage          : {voltage/10.0:.1f}V (Limits: {min_volt/10.0:.1f}V - {max_volt/10.0:.1f}V)")
    print(f"  - Measured Temperature      : {temp}C (Limit: {max_temp_lim}C)")
    print(f"  - Unloading Cond. (Reg 19)  : {unloading} (Bin: {bin(unloading)})")
    print(f"  - Angle Limits              : Min={min_ang}, Max={max_ang}")
    
    issues_found = False
    
    # A. Active errors in Reg 65
    if error_val > 0:
        issues_found = True
        print(f"\n{C_RED}[FOUND] Active Error State:{C_RST}")
        if error_val & 0x01:
            print(f"  * Voltage Error: Input voltage ({voltage/10.0:.1f}V) is out of configured limits ({min_volt/10.0:.1f}V - {max_volt/10.0:.1f}V).")
        if error_val & 0x02:
            print("  * Sensor/Angle Error: Magnetic encoder / feedback sensor detected an out-of-range value.")
        if error_val & 0x04:
            print(f"  * Temperature Error: Servo temp ({temp}°C) exceeded maximum limit ({max_temp_lim}°C).")
            print(f"    This is why the torque will not lock. If the temperature reading is incorrect/faulty (e.g. ambient is cool but it reports high),")
            print(f"    you can raise the Max Temperature Limit to allow the servo to operate.")
            fix = input(f"    Would you like to raise the Max Temperature Limit (Reg 13) to {temp + 10}°C? (y/n): ").strip().lower()
            if fix == 'y':
                handler.unLockEprom(sid)
                res_w, _ = handler.write1ByteTxRx(sid, 13, temp + 10)
                handler.LockEprom(sid)
                if res_w == COMM_SUCCESS:
                    print(f"    {C_GREEN}[SUCCESS] Max Temperature Limit updated to {temp + 10}°C. Please power-cycle the servo to apply and clear the error.{C_RST}")
                else:
                    print(f"    {C_RED}[ERROR] Failed to write Max Temperature Limit.{C_RST}")
        if error_val & 0x08:
            print("  * Overload / Overcurrent Error: Motor load exceeded protection threshold, triggering automatic shutdown to prevent burnout.")
        
        print(f"\n{C_GREEN}Guidance to Clear Error:{C_RST}")
        print("  1. Remove any physical load/obstruction from the servo horn.")
        print("  2. Ensure your power supply voltage is stable and within the recommended range.")
        print("  3. Allow the servo to cool down if it feels hot.")
        print("  4. Power-cycle (turn off and turn back on) the servo to clear the error latch.")
        
    # B. Torque Limit is 0
    if torque_limit == 0:
        issues_found = True
        print(f"\n{C_RED}[FOUND] Torque Limit is set to 0:{C_RST}")
        print("  The 'Max Torque' register (Reg 16) is 0, which prevents the servo from outputting any power.")
        fix = input("  Would you like to reset the Max Torque limit to default (1023)? (y/n): ").strip().lower()
        if fix == 'y':
            handler.unLockEprom(sid)
            res_wl, _ = handler.write2ByteTxRx(sid, 16, 1023)
            handler.LockEprom(sid)
            if res_wl == COMM_SUCCESS:
                print(f"  {C_GREEN}[SUCCESS] Max Torque restored to 1023. Try enabling torque now.{C_RST}")
            else:
                print(f"  {C_RED}[ERROR] Failed to write Max Torque register.{C_RST}")
                
    # C. Angle limits misconfigured
    if min_ang == max_ang and min_ang != 0:
        issues_found = True
        print(f"\n{C_RED}[FOUND] Angle Limits Conflict:{C_RST}")
        print("  Min and Max angle limits are equal, which may cause torque lock rejection.")
        print("  Guidance: Use Option 12 to set proper angle limits.")

    if not issues_found:
        print(f"\nNo obvious register faults found. If the servo remains limp, check if there is an active protection state that requires a power cycle.")
        print("Guidance: Try power-cycling the servo to clear any latched hardware protection states.")

def scan_servos(sts_handler, scan_all=False):
    """
    Scans servo IDs. Default is checking IDs [1, 2, 3] to be extremely fast.
    """
    target_ids = range(254) if scan_all else [1, 2, 3]
    msg = "Scanning all IDs 0-253..." if scan_all else "Performing fast scan on default IDs [1, 2, 3]..."
    print_header("Scan All Servos")
    print(msg + " Please wait.")
    
    # Temporarily set lower timeout for faster scanning
    sts_handler.portHandler.setPacketTimeout(10)
    
    found = []
    for sid in target_ids:
        model_number, result, error = sts_handler.ping(sid)
        if result == COMM_SUCCESS:
            byte0 = model_number & 0xFF
            byte1 = (model_number >> 8) & 0xFF
            servo_type, model = detect_servo_type(byte0, byte1)
            detected_types[sid] = servo_type
            detected_models[sid] = model
            print(f"{C_GREEN}  >>> Found Servo ID: {sid:3d} | Type: {servo_type} | Model: {model}{C_RST}")
            found.append(sid)
            
    sts_handler.portHandler.setPacketTimeout(50) # Restore default timeout
    return found

def live_monitor_dashboard(sts_handler, sc_handler, sids):
    """
    Merged status reader and live sensor monitor.
    Provides real-time updates and allows interactive torque toggling.
    """
    print_header("Live Monitor Dashboard")
    print(f"Monitoring Servos: {sids}")
    print(f"Guidance: Shows realtime sensor feedback. Pos: Position, Spd: Speed, V: Voltage, T: Temp.")
    print(f"Controls: Press {C_GREEN}[T]{C_RST} to toggle Torque on active servo. Press {C_RED}[Q/ESC]{C_RST} to return to main menu.\n")
    
    header = f"{C_CYA}ID  | Type | Model | Pos   | Speed | Load(kg) | Current   | Voltage  | Temp | Torque | Moving{C_RST}"
    print(header)
    print("-" * 96)
    
    num_servos = len(sids)
    for _ in range(num_servos):
        print() # Allocating lines in terminal
        
    try:
        while True:
            # Move cursor up to overwrite previous lines
            sys.stdout.write(f"\033[{num_servos}A")
            
            for sid in sids:
                servo_type = get_servo_type(sts_handler, sid)
                handler = sts_handler if servo_type == 'ST' else sc_handler
                model = detected_models.get(sid, 0)
                
                pos, spd, res, err = handler.ReadPosSpeed(sid)
                if res == COMM_SUCCESS:
                    volts, _, _ = handler.read1ByteTxRx(sid, 62)
                    temp, _, _ = handler.read1ByteTxRx(sid, 63)
                    load, _, _ = handler.read2ByteTxRx(sid, 60)
                    curr, _, _ = handler.read2ByteTxRx(sid, 69)
                    moving, _, _ = handler.read1ByteTxRx(sid, 66)
                    torque, _, _ = handler.read1ByteTxRx(sid, 40)
                    error_val, _, _ = handler.read1ByteTxRx(sid, 65)
                    
                    is_moving = "Yes" if moving > 0 else "No"
                    
                    raw_torque_str = "ON" if torque > 0 else "OFF"
                    if error_val > 0:
                        errors = []
                        if error_val & 0x08: errors.append("LD")
                        elif error_val & 0x04: errors.append("TM")
                        elif error_val & 0x01: errors.append("VT")
                        elif error_val & 0x02: errors.append("SN")
                        else: errors.append("ER")
                        raw_torque_str = f"ERR:{errors[0]}"
                    
                    padded_torque = f"{raw_torque_str:6s}"
                    if error_val > 0:
                        torque_str = f"{C_RED}{padded_torque}{C_RST}"
                    elif torque > 0:
                        torque_str = f"{C_GREEN}{padded_torque}{C_RST}"
                    else:
                        torque_str = padded_torque
                    
                    # Fix Load/Current signs
                    # Load: bits 0-9 is magnitude, bit 10 is direction (0 = CW / positive, 1 = CCW / negative)
                    # Convert to physical percentage (-100.0% to 100.0%) by dividing by 10.0
                    load_pct = -(load & 0x3FF) / 10.0 if (load & 0x400) else (load & 0x3FF) / 10.0
                    
                    # Calculate estimated torque in kg.cm based on measured voltage
                    measured_voltage = volts / 10.0
                    stall_torque = get_stall_torque(servo_type, model, measured_voltage)
                    load_torque = (load_pct / 100.0) * stall_torque
                    
                    # Current: bit 15 is sign bit, bits 0-14 is magnitude
                    curr_val = -(curr & 0x7FFF) if (curr & 0x8000) else curr
                    
                    # Apply temperature offset calibration
                    calibrated_temp = temp + TEMP_OFFSETS.get(sid, 0)
                    
                    # Format positions nicely depending on servo type resolution
                    sys.stdout.write(
                        f"\033[K" # Clear line
                        f"{sid:3d} | {servo_type:4s} | {model:5d} | {pos:5d} | {spd:5d} | {load_torque:6.2f}kg | {curr_val:5d}mA  | {measured_voltage:6.1f}V  | {calibrated_temp:3d}C | {torque_str} | {is_moving}\n"
                    )
                else:
                    sys.stdout.write(f"\033[K{sid:3d} | {servo_type:4s} | Offline / Timeout Error\n")
            
            sys.stdout.flush()
            
            # Non-blocking key check
            key = check_key()
            if key in ['q', '\x1b']: # 'q' or ESC
                break
            elif key == 't':
                target_id = sids[0]
                servo_type = get_servo_type(sts_handler, target_id)
                handler = sts_handler if servo_type == 'ST' else sc_handler
                
                torque, _, _ = handler.read1ByteTxRx(target_id, 40)
                new_torque = 0 if torque > 0 else 1
                handler.write1ByteTxRx(target_id, 40, new_torque)
                
                # Show quick visual notification line
                sys.stdout.write(f"\033[K\n{C_YEL}Toggled Torque of ID {target_id} to {'ON' if new_torque else 'OFF'}{C_RST}")
                time.sleep(0.5)
                sys.stdout.write("\033[1A\033[K")
                
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    print(f"\n{C_YEL}Exiting Live Monitor...{C_RST}")

def center_align_servo(sts_handler, sc_handler, active_id):
    """
    Quickly centers the selected servo.
    """
    print_header("Center Align (Move to Center)")
    print("Guidance: Centers the servo at its middle rest angle.")
    print("  - ST Series: Center is 2048 (Range 0-4095)")
    print("  - SC Series: Center is 512 (Range 0-1023)\n")
    
    sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
    if sid is None: return
    
    servo_type = get_servo_type(sts_handler, sid)
    handler = sts_handler if servo_type == 'ST' else sc_handler
    
    center_pos = 512 if (servo_type == 'SC' and detected_models.get(sid) in [15, 9]) else 2048
    
    if servo_type == 'ST':
        spd = get_int("Speed (0-3000) [Default: 1500]: ", default=1500, min_val=0, max_val=3000)
        if spd is None: return
        acc = get_int("Acceleration (0-254) [Default: 50]: ", default=50, min_val=0, max_val=254)
        if acc is None: return
        
        handler.write1ByteTxRx(sid, 33, 0) # Position mode
        res, err = handler.WritePosEx(sid, center_pos, spd, acc)
    else:
        spd = get_int("Speed (0-1500) [Default: 800]: ", default=800, min_val=0, max_val=1500)
        if spd is None: return
        time_val = get_int("Time (0-254) [Default: 0]: ", default=0, min_val=0, max_val=254)
        if time_val is None: return
        
        # Ensure position control limits are set
        handler.unLockEprom(sid)
        handler.write2ByteTxRx(sid, 9, 0)
        handler.write2ByteTxRx(sid, 11, 1023)
        handler.LockEprom(sid)
        
        res, err = handler.WritePos(sid, center_pos, time_val, spd)
        
    if res == COMM_SUCCESS:
        print(f"{C_GREEN}Alignment command sent successfully.{C_RST}")
    else:
        print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")

def continuous_rotation(sts_handler, sc_handler, active_id):
    """
    Continuous rotation with a safety timer (runs for 5s then stops).
    """
    print_header("Continuous Rotation (5-Second Run)")
    print("Guidance: Configures and spins the servo like a motor for exactly 5 seconds.")
    print("  - ST Series: Speed is controlled via rate (-3000 to 3000).")
    print("  - SC Series: Speed is controlled via PWM duty cycle (-1000 to 1000).\n")
    
    sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
    if sid is None: return
    
    servo_type = get_servo_type(sts_handler, sid)
    handler = sts_handler if servo_type == 'ST' else sc_handler
    
    if servo_type == 'ST':
        spd = get_int("Speed (-3000 to 3000) [Default: 1000]: ", default=1000, min_val=-3000, max_val=3000)
        if spd is None: return
        
        handler.WheelMode(sid)
        res, err = handler.WriteSpec(sid, spd, 50)
    else:
        pwm = get_int("PWM Value (-1000 to 1000) [Default: 500]: ", default=500, min_val=-1000, max_val=1000)
        if pwm is None: return
        
        handler.PWMMode(sid)
        res, err = handler.WritePWM(sid, pwm)
        
    if res == COMM_SUCCESS:
        print(f"\n{C_GREEN}Running rotation... Press Ctrl+C to cancel early.{C_RST}")
        try:
            for i in range(5, 0, -1):
                sys.stdout.write(f"\rTime remaining: {i} seconds... ")
                sys.stdout.flush()
                time.sleep(1)
            print(f"\rTime remaining: 0 seconds... stopping.")
        except KeyboardInterrupt:
            print("\nInterrupted! Stopping immediately.")
            
        # Send stop command
        if servo_type == 'ST':
            handler.WriteSpec(sid, 0, 50)
        else:
            handler.WritePWM(sid, 0)
        print(f"{C_GREEN}Servo {sid} stopped.{C_RST}")
    else:
        print(f"{C_RED}Error running rotation: {handler.getTxRxResult(res)}{C_RST}")

def emergency_relax(sts_handler):
    """
    Broadcasts a torque disable (0) to all servos on the bus.
    """
    print_header("Emergency Relax (Disable All Torque)")
    print("Guidance: Send a broadcast command to turn off holding torque on all connected servos.")
    print("This will let you manually move the servos without disconnecting power.\n")
    
    confirm = input("Are you sure? (y/n) [Default: n]: ").strip().lower()
    if confirm != 'y':
        print("Cancelled.")
        return
        
    # Broadcast ID is 254 (0xFE). Writes to address 40 (Torque Enable)
    sts_handler.write1ByteTxRx(254, 40, 0)
    print(f"{C_GREEN}Broadcast torque disable sent. All servos relaxed successfully.{C_RST}")

def record_motion(sts_handler, sc_handler, sids, filename="recorded_motion.json"):
    """
    Disables torque on the specified servos, samples their physical positions at 10 Hz, 
    and saves the timing sequence to a JSON file.
    """
    print_header("Teach-Playback: Record Motion")
    print(f"Target Servos: {sids}")
    print("Releasing torque (disabling holding state) on target servos...")
    
    for sid in sids:
        stype = get_servo_type(sts_handler, sid)
        handler = sts_handler if stype == 'ST' else sc_handler
        handler.write1ByteTxRx(sid, 40, 0)
        
    print(f"\n{C_GREEN}Torque disabled! Servos are now loose. You can guide/pose them manually.{C_RST}")
    print(f"Press {C_YEL}ENTER{C_RST} or any key to stop recording.\n")
    
    frames = []
    start_time = time.time()
    
    try:
        while True:
            elapsed = time.time() - start_time
            frame_data = {}
            all_ok = True
            
            for sid in sids:
                stype = get_servo_type(sts_handler, sid)
                handler = sts_handler if stype == 'ST' else sc_handler
                pos, _, res, _ = handler.ReadPosSpeed(sid)
                if res == COMM_SUCCESS:
                    frame_data[str(sid)] = pos
                else:
                    all_ok = False
                    
            if all_ok:
                frames.append({
                    "time": elapsed,
                    "positions": frame_data
                })
                sys.stdout.write(f"\rRecording... Time: {elapsed:.1f}s | Frames: {len(frames)} | Pos: {frame_data}")
                sys.stdout.flush()
            else:
                sys.stdout.write(f"\r{C_RED}Warning: Communications failed or servo timeout at elapsed: {elapsed:.1f}s{C_RST}")
                sys.stdout.flush()
                
            key = check_key()
            if key is not None:
                break
                
            time.sleep(0.1)  # 10 Hz sampling rate
    except KeyboardInterrupt:
        pass
        
    print(f"\n\n{C_YEL}Recording stopped.{C_RST}")
    
    # Re-enable torque on stop to lock their current positions
    for sid in sids:
        stype = get_servo_type(sts_handler, sid)
        handler = sts_handler if stype == 'ST' else sc_handler
        handler.write1ByteTxRx(sid, 40, 1)
        
    if not frames:
        print(f"{C_RED}No frames captured. Canceled saving.{C_RST}")
        return
        
    data = {
        "servo_ids": sids,
        "servo_types": {str(sid): get_servo_type(sts_handler, sid) for sid in sids},
        "frames": frames
    }
    
    try:
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"{C_GREEN}Successfully recorded {len(frames)} frames to '{filename}'!{C_RST}")
    except Exception as e:
        print(f"{C_RED}Error saving recording file: {e}{C_RST}")

def play_motion(sts_handler, sc_handler, filename="recorded_motion.json"):
    """
    Locks torque and replays the position timeline saved in the JSON file.
    """
    print_header("Teach-Playback: Playback Motion")
    
    if not os.path.exists(filename):
        print(f"{C_RED}Recording file '{filename}' not found.{C_RST}")
        return
        
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"{C_RED}Error loading recording file: {e}{C_RST}")
        return
        
    sids = data.get("servo_ids", [])
    servo_types = data.get("servo_types", {})
    frames = data.get("frames", [])
    
    if not frames:
        print(f"{C_RED}File contains empty frames.{C_RST}")
        return
        
    print(f"Target Servos: {sids}")
    print(f"Sequence Length: {len(frames)} frames ({frames[-1]['time']:.1f} seconds)")
    print("Enabling torque on target servos...")
    
    for sid in sids:
        stype = servo_types.get(str(sid), 'ST')
        handler = sts_handler if stype == 'ST' else sc_handler
        handler.write1ByteTxRx(sid, 40, 1)
        
    print(f"\n{C_GREEN}Torque locked! Replaying motion... Press Ctrl+C to cancel.{C_RST}")
    
    start_time = time.time()
    try:
        for frame in frames:
            target_elapsed = frame["time"]
            positions = frame["positions"]
            
            # Match recording timestamp
            while (time.time() - start_time) < target_elapsed:
                time.sleep(0.005)
                
            for sid_str, pos in positions.items():
                sid = int(sid_str)
                stype = servo_types.get(sid_str, 'ST')
                handler = sts_handler if stype == 'ST' else sc_handler
                
                if stype == 'ST':
                    handler.WritePosEx(sid, pos, 3000, 0)
                else:
                    handler.WritePos(sid, pos, 0, 1500)
                    
            elapsed_now = time.time() - start_time
            sys.stdout.write(f"\rReplaying... Time: {elapsed_now:.1f}s / {frames[-1]['time']:.1f}s | Positions: {positions}")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print(f"\n{C_RED}Playback interrupted.{C_RST}")
        
    print(f"\n{C_GREEN}Playback complete!{C_RST}")

def advanced_debugging_menu(sts_handler, sc_handler, active_id):
    """
    Submenu containing less frequently used low-level tools.
    """
    while True:
        clear_screen()
        print_header("Advanced Debugging Menu")
        print(f"{C_YEL}Warning: Incorrect values can permanently damage servo configurations.{C_RST}")
        print("\n=== Guidance & Explanations ===")
        print(" - Raw Registers: Inspect or manually modify addresses in the servo memory map.")
        print("   Address layout differs between models; refer to register tables before writing.")
        print(" - Action Command (INST_ACTION): Executes instructions queued by 'regWrite'.")
        print("   Used to trigger synchronized motion across multiple servos.")
        
        print("\n 1. Read Raw Register (1/2/4 bytes)")
        print(" 2. Write Raw Register (1/2/4 bytes)")
        print(" 3. Execute Action Command (INST_ACTION)")
        print(" 4. Torque Diagnostics & Recovery")
        print("\n 0. Return to Main Menu")
        print(f"{C_CYA}{'-'*60}{C_RST}")
        
        choice = input("Select an option (0-4): ").strip()
        
        if choice == '0':
            break
            
        elif choice == '1':
            print_header("Read Raw Register")
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            addr = get_int("Enter Memory Address (0-255): ", min_val=0, max_val=255)
            if addr is None: continue
            
            size = get_int("Enter Byte Size (1, 2, or 4): ", min_val=1, max_val=4)
            if size not in [1, 2, 4]:
                print(f"{C_RED}Invalid size. Must be 1, 2, or 4.{C_RST}")
                time.sleep(1.5)
                continue
                
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            
            if size == 1: val, res, _ = handler.read1ByteTxRx(sid, addr)
            elif size == 2: val, res, _ = handler.read2ByteTxRx(sid, addr)
            elif size == 4: val, res, _ = handler.read4ByteTxRx(sid, addr)
            
            if res == COMM_SUCCESS:
                print(f"\n{C_GREEN}Value at Address {addr} ({size} bytes): {val} (0x{val:X}){C_RST}")
            else:
                print(f"\n{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
            input("\nPress Enter to continue...")
            
        elif choice == '2':
            print_header("Write Raw Register")
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            addr = get_int("Enter Memory Address (0-255): ", min_val=0, max_val=255)
            if addr is None: continue
            
            size = get_int("Enter Byte Size (1, 2, or 4): ", min_val=1, max_val=4)
            if size not in [1, 2, 4]:
                print(f"{C_RED}Invalid size. Must be 1, 2, or 4.{C_RST}")
                time.sleep(1.5)
                continue
                
            val = get_int("Enter Value to write: ")
            if val is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            
            if size == 1: res, _ = handler.write1ByteTxRx(sid, addr, val)
            elif size == 2: res, _ = handler.write2ByteTxRx(sid, addr, val)
            elif size == 4: res, _ = handler.write4ByteTxRx(sid, addr, val)
            
            if res == COMM_SUCCESS:
                print(f"\n{C_GREEN}Successfully wrote {val} (0x{val:X}) to Address {addr}{C_RST}")
            else:
                print(f"\n{C_RED}Error writing register: {handler.getTxRxResult(res)}{C_RST}")
            input("\nPress Enter to continue...")
            
        elif choice == '3':
            print_header("Execute Action Command")
            print("Sends a broadcast trigger to execute queued registered writes.")
            sid = get_int("Enter Servo ID (254 for Broadcast): ", default=254)
            if sid is None: continue
            
            res = sts_handler.action(sid)
            print(f"{C_GREEN}Action (INST_ACTION) command sent to ID {sid}.{C_RST}")
            time.sleep(1.5)
            
        elif choice == '4':
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is not None:
                diagnose_torque_issue(sts_handler, sc_handler, sid)
                input("\nPress Enter to continue...")

def fuzzy_match_command(word):
    """
    Fuzzy match to resolve typos, case insensitivity, and common abbreviations.
    """
    word = word.lower().strip()
    if word in ['move', 'mov', 'mve', 'moev', 'm']: return 'move'
    if word in ['active', 'act', 'actv', 'ac']: return 'active'
    if word in ['ping', 'png', 'pign', 'pi']: return 'ping'
    if word in ['center', 'cntr', 'centr', 'cen']: return 'center'
    if word in ['rotate', 'rot', 'rotat', 'ro']: return 'rotate'
    if word in ['torque', 'torq', 'torqu', 'tq', 'torqe']: return 'torque'
    if word in ['relax', 'relx', 'rx']: return 'relax'
    if word in ['record', 'rec', 'rc']: return 'record'
    if word in ['play', 'ply', 'pl']: return 'play'
    if word in ['monitor', 'mon', 'mn']: return 'monitor'
    if word in ['diag', 'diagnostic', 'diagnose', 'dg']: return 'diag'
    return word

def parse_and_run_command(cmd_str, sts_handler, sc_handler, active_id):
    """
    Parses a direct command entered by the user and executes it.
    Supports argument defaulting and fuzzy name mappings.
    Returns:
        new_active_id (int or None): If active ID changed, returns the new active ID.
        handled (bool): True if the command was recognized and processed, False otherwise.
    """
    parts = cmd_str.strip().split()
    if not parts:
        return active_id, False
        
    raw_cmd = parts[0]
    cmd = fuzzy_match_command(raw_cmd)
    args = parts[1:]
    
    # 1. active <id> / act <id>
    if cmd == 'active':
        if not args:
            print(f"{C_RED}Usage: active/act <id>{C_RST}")
            return active_id, True
        try:
            sid = int(args[0])
            if 0 <= sid <= 253:
                servo_type, model, res = ping_and_detect(sts_handler, sid)
                if res == COMM_SUCCESS:
                    detected_types[sid] = servo_type
                    detected_models[sid] = model
                    print(f"{C_GREEN}Active servo set to ID {sid} ({servo_type} - {model}).{C_RST}")
                    active_id = sid
                else:
                    print(f"{C_RED}Warning: Servo {sid} not responding. Active ID set anyway.{C_RST}")
                    detected_types[sid] = 'ST'
                    detected_models[sid] = 0
                    active_id = sid
            else:
                print(f"{C_RED}Servo ID must be 0-253.{C_RST}")
        except ValueError:
            print(f"{C_RED}Invalid Servo ID.{C_RST}")
        return active_id, True
        
    # 2. ping [id/all]
    elif cmd == 'ping':
        if args and args[0].lower() == 'all':
            found = scan_servos(sts_handler, scan_all=True)
            if found:
                print(f"\n{C_GREEN}Found {len(found)} servo(s): {found}{C_RST}")
                if active_id not in found and found:
                    active_id = found[0]
                    print(f"{C_GREEN}Active servo set to default ID {active_id}.{C_RST}")
            else:
                print(f"{C_RED}No servos found.{C_RST}")
        elif not args:
            print(f"Pinging default servos [1, 2, 3]...")
            found = []
            for sid in [1, 2, 3]:
                servo_type, model, res = ping_and_detect(sts_handler, sid)
                if res == COMM_SUCCESS:
                    detected_types[sid] = servo_type
                    detected_models[sid] = model
                    print(f"{C_GREEN}Servo {sid} online! Type: {servo_type} | Model: {model}{C_RST}")
                    found.append(sid)
                else:
                    print(f"{C_RED}Servo {sid} offline.{C_RST}")
        else:
            try:
                sid = int(args[0])
                servo_type, model, res = ping_and_detect(sts_handler, sid)
                if res == COMM_SUCCESS:
                    detected_types[sid] = servo_type
                    detected_models[sid] = model
                    print(f"{C_GREEN}Servo {sid} online! Type: {servo_type} | Model: {model}{C_RST}")
                else:
                    print(f"{C_RED}Servo {sid} did not respond: {sts_handler.getTxRxResult(res)}{C_RST}")
            except ValueError:
                print(f"{C_RED}Invalid Servo ID.{C_RST}")
        return active_id, True
        
    # 3. move [id] <pos> [speed] [acc/time]
    elif cmd == 'move':
        if not args:
            print(f"{C_RED}Usage: move [id] <pos> [speed] [acc/time]{C_RST}")
            return active_id, True
        try:
            speed_val = None
            acc_val = None
            
            if len(args) == 1:
                # Omitted ID: move <pos> -> uses active_id
                if active_id is None:
                    print(f"{C_RED}Error: Specify servo ID or set active servo first. Usage: move <id> <pos>{C_RST}")
                    return active_id, True
                sid = active_id
                pos = int(args[0])
            else:
                # ID and position both provided: move <id> <pos>
                sid = int(args[0])
                pos = int(args[1])
                speed_val = int(args[2]) if len(args) > 2 else None
                acc_val = int(args[3]) if len(args) > 3 else None
                
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            max_pos = 1023 if (servo_type == 'SC' and detected_models.get(sid) in [15, 9]) else 4095
            
            if not (0 <= pos <= max_pos):
                print(f"{C_RED}Position out of range (0-{max_pos}){C_RST}")
                return active_id, True
                
            if servo_type == 'ST':
                spd = speed_val if speed_val is not None else 2400
                acc = acc_val if acc_val is not None else 50
                
                handler.write1ByteTxRx(sid, 33, 0) # Pos mode
                res, err = handler.WritePosEx(sid, pos, spd, acc)
            else:
                spd = speed_val if speed_val is not None else 1000
                time_val = acc_val if acc_val is not None else 0
                
                handler.unLockEprom(sid)
                handler.write2ByteTxRx(sid, 9, 0)
                handler.write2ByteTxRx(sid, 11, max_pos)
                handler.LockEprom(sid)
                
                res, err = handler.WritePos(sid, pos, time_val, spd)
                
            if res == COMM_SUCCESS:
                print(f"{C_GREEN}Move command sent successfully to ID {sid} -> Position {pos}.{C_RST}")
            else:
                print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
        except ValueError:
            print(f"{C_RED}Invalid numeric arguments.{C_RST}")
        return active_id, True
        
    # 4. center [id]
    elif cmd == 'center':
        sid = active_id if not args else None
        if args:
            try: sid = int(args[0])
            except ValueError: pass
        if sid is None:
            print(f"{C_RED}Specify ID or set active servo first. Usage: center [id]{C_RST}")
            return active_id, True
            
        servo_type = get_servo_type(sts_handler, sid)
        handler = sts_handler if servo_type == 'ST' else sc_handler
        center_pos = 512 if (servo_type == 'SC' and detected_models.get(sid) in [15, 9]) else 2048
        
        if servo_type == 'ST':
            handler.write1ByteTxRx(sid, 33, 0)
            res, err = handler.WritePosEx(sid, center_pos, 1500, 50)
        else:
            handler.unLockEprom(sid)
            handler.write2ByteTxRx(sid, 9, 0)
            handler.write2ByteTxRx(sid, 11, 1023)
            handler.LockEprom(sid)
            res, err = handler.WritePos(sid, center_pos, 0, 800)
            
        if res == COMM_SUCCESS:
            print(f"{C_GREEN}Servo {sid} centered (position {center_pos}).{C_RST}")
        else:
            print(f"{C_RED}Error centering servo {sid}: {handler.getTxRxResult(res)}{C_RST}")
        return active_id, True
        
    # 5. rotate [id] <f/b> [speed/pwm]
    elif cmd == 'rotate':
        if not args:
            print(f"{C_RED}Usage: rotate [id] <f/b> [speed/pwm]{C_RST}")
            return active_id, True
        try:
            # Parse arguments
            if len(args) == 1:
                # Omitted ID: rotate <f/b>
                if active_id is None:
                    print(f"{C_RED}Error: Specify servo ID or set active servo first. Usage: rotate <id> <f/b>{C_RST}")
                    return active_id, True
                sid = active_id
                direction = args[0].lower()
                speed_arg = None
            elif len(args) >= 2:
                # Check if first arg is direction or ID
                if args[0].lower() in ['f', 'for', 'forward', 'b', 'back', 'backward']:
                    # Omitted ID: rotate <f/b> [speed/pwm]
                    if active_id is None:
                        print(f"{C_RED}Error: Specify servo ID or set active servo first.{C_RST}")
                        return active_id, True
                    sid = active_id
                    direction = args[0].lower()
                    speed_arg = int(args[1]) if len(args) > 1 else None
                else:
                    # ID and direction provided: rotate <id> <f/b> [speed/pwm]
                    sid = int(args[0])
                    direction = args[1].lower()
                    speed_arg = int(args[2]) if len(args) > 2 else None
                    
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            
            if direction not in ['f', 'for', 'forward', 'b', 'back', 'backward']:
                print(f"{C_RED}Direction must be f (forward) or b (backward).{C_RST}")
                return active_id, True
                
            sign = 1 if direction in ['f', 'for', 'forward'] else -1
            
            if servo_type == 'ST':
                speed_val = speed_arg if speed_arg is not None else 1000
                speed_val = abs(speed_val) * sign
                
                handler.WheelMode(sid)
                res, err = handler.WriteSpec(sid, speed_val, 50)
            else:
                pwm_val = speed_arg if speed_arg is not None else 500
                pwm_val = abs(pwm_val) * sign
                
                handler.PWMMode(sid)
                res, err = handler.WritePWM(sid, pwm_val)
                
            if res == COMM_SUCCESS:
                print(f"{C_GREEN}Rotating servo {sid} for 5 seconds...{C_RST}")
                try:
                    for i in range(5, 0, -1):
                        sys.stdout.write(f"\rTime remaining: {i} seconds... ")
                        sys.stdout.flush()
                        time.sleep(1)
                    print(f"\rTime remaining: 0 seconds... stopping.")
                except KeyboardInterrupt:
                    print("\nInterrupted!")
                
                if servo_type == 'ST':
                    handler.WriteSpec(sid, 0, 50)
                else:
                    handler.WritePWM(sid, 0)
                print(f"{C_GREEN}Stopped servo {sid}.{C_RST}")
            else:
                print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
        except ValueError:
            print(f"{C_RED}Invalid numeric arguments.{C_RST}")
        return active_id, True
        
    # 6. torque [id] <on/off/1/0>
    elif cmd == 'torque':
        if not args:
            print(f"{C_RED}Usage: torque [id] <on/off/1/0>{C_RST}")
            return active_id, True
        try:
            if len(args) == 1:
                # Omitted ID: torque <state> -> uses active_id
                if active_id is None:
                    print(f"{C_RED}Error: Specify servo ID or set active servo first. Usage: torque <id> <state>{C_RST}")
                    return active_id, True
                sid = active_id
                state_str = args[0].lower()
            else:
                sid = int(args[0])
                state_str = args[1].lower()
                
            if state_str in ['on', '1', 'enable']:
                state = 1
            elif state_str in ['off', '0', 'disable']:
                state = 0
            else:
                print(f"{C_RED}State must be on, off, 1, or 0.{C_RST}")
                return active_id, True
                
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            
            res, err = handler.write1ByteTxRx(sid, 40, state)
            if res == COMM_SUCCESS:
                time.sleep(0.1)
                t_val, _, _ = handler.read1ByteTxRx(sid, 40)
                if state == 1 and t_val == 0:
                    print(f"\n{C_RED}Warning: Torque command succeeded, but Servo {sid} rejected the lock and remains LIMP (0).{C_RST}")
                    diagnose_torque_issue(sts_handler, sc_handler, sid)
                else:
                    print(f"{C_GREEN}Torque on servo {sid} set to {'ON' if state else 'OFF'}.{C_RST}")
            else:
                print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
        except ValueError:
            print(f"{C_RED}Invalid numeric arguments.{C_RST}")
        return active_id, True
        
    # 6b. diag [id]
    elif cmd == 'diag':
        try:
            if args:
                sid = int(args[0])
            else:
                if active_id is None:
                    print(f"{C_RED}Error: Specify servo ID or set active servo first. Usage: diag <id>{C_RST}")
                    return active_id, True
                sid = active_id
                
            diagnose_torque_issue(sts_handler, sc_handler, sid)
        except ValueError:
            print(f"{C_RED}Invalid Servo ID.{C_RST}")
        return active_id, True
        
    # 7. relax / relax all
    elif cmd in ['relax', 'relaxall']:
        emergency_relax(sts_handler)
        return active_id, True
        
    # 8. monitor [ids] / mon [ids]
    elif cmd == 'monitor':
        if args:
            try:
                # parse commas or spaces
                sids = [int(x.strip()) for x in "".join(args).replace(',', ' ').split() if x.strip()]
            except ValueError:
                print(f"{C_RED}Invalid ID list.{C_RST}")
                return active_id, True
        else:
            sids = [active_id] if active_id is not None else []
            
        if sids:
            live_monitor_dashboard(sts_handler, sc_handler, sids)
        else:
            print(f"{C_RED}No active servo and no IDs provided.{C_RST}")
        return active_id, True
        
    # 9. record <ids> [filename]
    elif cmd == 'record':
        if not args:
            print(f"{C_RED}Usage: record <id1,id2,...> [filename]{C_RST}")
            return active_id, True
        try:
            ids_part = args[0]
            sids = [int(x.strip()) for x in ids_part.replace(',', ' ').split() if x.strip()]
            filename = args[1] if len(args) > 1 else "recorded_motion.json"
            
            if not sids:
                print(f"{C_RED}No valid IDs specified.{C_RST}")
                return active_id, True
                
            record_motion(sts_handler, sc_handler, sids, filename)
        except ValueError:
            print(f"{C_RED}Invalid ID list.{C_RST}")
        return active_id, True
        
    # 10. play [filename]
    elif cmd == 'play':
        filename = args[0] if args else "recorded_motion.json"
        play_motion(sts_handler, sc_handler, filename)
        return active_id, True
        
    return active_id, False

def main():
    portHandler = PortHandler(DEVICENAME)
    sts_handler = sts(portHandler)
    sc_handler = scscl(portHandler)
    
    clear_screen()
    draw_boxed_header(None)
    
    try:
        success = portHandler.openPort()
    except Exception:
        success = False
        
    if not success:
        print(f"{C_YEL}Could not open default port {DEVICENAME}. Let's select one.{C_RST}")
        while True:
            port_name = select_port()
            portHandler.setPortName(port_name)
            try:
                if portHandler.openPort():
                    break
            except Exception as e:
                print(f"{C_RED}Error: Failed to open port {port_name}: {e}{C_RST}")
            time.sleep(1.5)
            
    print(f"{C_GREEN}Successfully connected to {portHandler.getPortName()} at {portHandler.getBaudRate()} bps{C_RST}")
    
    active_id = None
    
    # Try to locate default servo ID 1 at start (fast ping)
    _, _, res = ping_and_detect(sts_handler, 1)
    if res == COMM_SUCCESS:
        active_id = 1
        
    print_menu_next = True
    while True:
        if print_menu_next:
            clear_screen()
            draw_boxed_header(active_id)
            draw_menu()
            print_menu_next = False
            
        choice_str = input("\nSelect number (0-15) or type command: ").strip()
        if not choice_str:
            print_menu_next = True
            continue
            
        if choice_str.lower() in ['menu', 'help', 'cls', 'clear']:
            print_menu_next = True
            continue
            
        # Try to parse as direct command first (accepts caps, typos, etc.)
        new_active, handled = parse_and_run_command(choice_str, sts_handler, sc_handler, active_id)
        if handled:
            active_id = new_active
            lower_choice = choice_str.lower()
            if any(x in lower_choice for x in ['mon', 'play', 'rec', 'help', 'menu', 'cls', 'clear']):
                print_menu_next = True
            continue
            
        # Fallback to menu number parsing
        choice = choice_str
        
        if choice == '1':
            default_ids = str(active_id) if active_id is not None else ""
            prompt = f"Enter Servo IDs to monitor (comma separated) [Default: {default_ids}]: "
            val = input(prompt).strip()
            if not val:
                if active_id is not None:
                    sids = [active_id]
                else:
                    print(f"{C_RED}No servos selected and no active servo default set.{C_RST}")
                    time.sleep(1.5)
                    continue
            else:
                try:
                    sids = [int(x.strip()) for x in val.split(',') if x.strip()]
                except ValueError:
                    print(f"{C_RED}Invalid ID list.{C_RST}")
                    time.sleep(1)
                    continue
                    
            if sids:
                live_monitor_dashboard(sts_handler, sc_handler, sids)
                print_menu_next = True
                
        elif choice == '2':
            new_id = select_active_servo(sts_handler)
            if new_id is not None:
                active_id = new_id
                
        elif choice == '3':
            confirm_all = input("Scan all 254 IDs? (y/n) [Default: n, scans 1,2,3]: ").strip().lower()
            scan_all = (confirm_all == 'y')
            found = scan_servos(sts_handler, scan_all=scan_all)
            if found:
                print(f"\n{C_GREEN}Found {len(found)} servo(s): {found}{C_RST}")
                sel = get_int("Select one of these as the active servo (Enter ID, or press Enter to skip): ", min_val=0, max_val=253)
                if sel in found:
                    active_id = sel
                    print(f"{C_GREEN}Active servo set to ID {active_id}.{C_RST}")
                    time.sleep(1)
            else:
                print(f"{C_RED}No servos found. Check connections/power.{C_RST}")
                time.sleep(2)
                
        elif choice == '4':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            max_pos = 1023 if (servo_type == 'SC' and detected_models.get(sid) in [15, 9]) else 4095
            center_val = 512 if max_pos == 1023 else 2048
            
            print(f"Guidance: Absolute Move shifts the servo to a specific coordinate.")
            print(f"Target Position range: 0 to {max_pos} (Center: {center_val})")
            pos = get_int(f"Target Position (0-{max_pos}) [Default: {center_val}]: ", default=center_val, min_val=0, max_val=max_pos)
            if pos is None: continue
            
            if servo_type == 'ST':
                spd = get_int("Speed (0-3000) [Default: 2400]: ", default=2400, min_val=0, max_val=3000)
                if spd is None: continue
                acc = get_int("Acceleration (0-254) [Default: 50]: ", default=50, min_val=0, max_val=254)
                if acc is None: continue
                
                sts_handler.write1ByteTxRx(sid, 33, 0) # Pos mode
                res, err = sts_handler.WritePosEx(sid, pos, spd, acc)
            else:
                spd = get_int("Speed (0-1500) [Default: 1000]: ", default=1000, min_val=0, max_val=1500)
                if spd is None: continue
                time_val = get_int("Time (0-254) [Default: 0]: ", default=0, min_val=0, max_val=254)
                if time_val is None: continue
                
                # Make sure angle limits are non-zero to enable position control
                sc_handler.unLockEprom(sid)
                sc_handler.write2ByteTxRx(sid, 9, 0)
                sc_handler.write2ByteTxRx(sid, 11, max_pos)
                sc_handler.LockEprom(sid)
                
                res, err = sc_handler.WritePos(sid, pos, time_val, spd)
                
            if res == COMM_SUCCESS:
                print(f"{C_GREEN}Move command sent successfully.{C_RST}")
            else:
                handler = sts_handler if servo_type == 'ST' else sc_handler
                print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
            
        elif choice == '5':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
            center_align_servo(sts_handler, sc_handler, active_id)
            
        elif choice == '6':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
            continuous_rotation(sts_handler, sc_handler, active_id)
            
        elif choice == '7':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            max_pos = 1023 if (servo_type == 'SC' and detected_models.get(sid) in [15, 9]) else 4095
            
            pos, _, res, _ = handler.ReadPosSpeed(sid)
            if res == COMM_SUCCESS:
                print(f"Current Position: {pos}")
                step = get_int("Step amount (e.g. 100 or -100): ", min_val=-max_pos, max_val=max_pos)
                if step is None: continue
                
                new_pos = max(0, min(max_pos, pos + step))
                print(f"Target Position: {new_pos}")
                
                if servo_type == 'ST':
                    spd = get_int("Speed (0-3000) [Default: 2400]: ", default=2400, min_val=0, max_val=3000)
                    if spd is None: continue
                    res, err = handler.WritePosEx(sid, new_pos, spd, 50)
                else:
                    spd = get_int("Speed (0-1500) [Default: 1000]: ", default=1000, min_val=0, max_val=1500)
                    if spd is None: continue
                    res, err = handler.WritePos(sid, new_pos, 0, spd)
                    
                if res == COMM_SUCCESS:
                    print(f"{C_GREEN}Jog command sent.{C_RST}")
                else:
                    print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
            else:
                print(f"{C_RED}Failed to read current position.{C_RST}")
            
        elif choice == '8':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            
            torque, res, _ = handler.read1ByteTxRx(sid, 40)
            if res == COMM_SUCCESS:
                print(f"Guidance: Disabling torque lets you rotate the horn manually. Enabling locks it.")
                print(f"Current Torque state: {'Enabled (Locked)' if torque > 0 else 'Disabled (Limp)'}")
                state = get_int("Enter new state (1: Enable, 0: Disable): ", min_val=0, max_val=1)
                if state is None: continue
                
                res, err = handler.write1ByteTxRx(sid, 40, state)
                if res == COMM_SUCCESS:
                    time.sleep(0.1)
                    t_val, _, _ = handler.read1ByteTxRx(sid, 40)
                    if state == 1 and t_val == 0:
                        print(f"\n{C_RED}Warning: Torque lock was rejected by the servo.{C_RST}")
                        diagnose_torque_issue(sts_handler, sc_handler, sid)
                        input("\nPress Enter to continue...")
                    else:
                        print(f"{C_GREEN}Torque state updated successfully.{C_RST}")
                else:
                    print(f"{C_RED}Error: {handler.getTxRxResult(res)}{C_RST}")
            else:
                print(f"{C_RED}Failed to read torque state: {handler.getTxRxResult(res)}{C_RST}")
            
        elif choice == '9':
            emergency_relax(sts_handler)
            
        elif choice == '10':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            
            min_ang, res, _ = handler.read2ByteTxRx(sid, 9)
            if res == COMM_SUCCESS:
                max_ang, _, _ = handler.read2ByteTxRx(sid, 11)
                torq, _, _ = handler.read1ByteTxRx(sid, 40)
                baud, _, _ = handler.read1ByteTxRx(sid, 6)
                
                print(f"\n{C_YEL}--- Device Config for Servo {sid} ({servo_type}) ---{C_RST}")
                print(f" Min Angle Limit : {min_ang}")
                print(f" Max Angle Limit : {max_ang}")
                print(f" Torque Enable   : {'Enabled (Locked)' if torq > 0 else 'Disabled (Limp)'}")
                print(f" Baudrate Index  : {baud} (0:1M, 1:0.5M, 2:250k, 3:128k, 4:115200, 5:76800, 6:57600, 7:38400)")
                
                if servo_type == 'ST':
                    ofs, _, _ = handler.read2ByteTxRx(sid, 31)
                    mode, _, _ = handler.read1ByteTxRx(sid, 33)
                    print(f" Position Offset : {handler.sts_tohost(ofs, 11)}")
                    print(f" Work Mode       : {mode} (0:Pos, 1:Wheel, 2:Step, 3:Multi-turn)")
                else:
                    is_pwm = (min_ang == 0 and max_ang == 0)
                    print(f" Work Mode       : {'Continuous Rotation (PWM Mode)' if is_pwm else 'Position Mode'}")
            else:
                print(f"{C_RED}Failed to read configuration: {handler.getTxRxResult(res)}{C_RST}")
            
        elif choice == '11':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Current Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            nid = get_int("Enter NEW Servo ID (0-253): ", min_val=0, max_val=253)
            if nid is None: continue
            if sid == nid:
                print(f"{C_RED}New ID must be different from current ID.{C_RST}")
                time.sleep(1.5)
                continue
                
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            lock_reg = 55 if servo_type == 'ST' else 48
            
            print(f"{C_YEL}Applying permanent ID change: ID {sid} -> ID {nid}...{C_RST}")
            handler.write1ByteTxRx(sid, 40, 0)
            handler.write1ByteTxRx(sid, lock_reg, 0) # Unlock EPROM
            
            res, _ = handler.write1ByteTxRx(sid, 5, nid)
            if res == COMM_SUCCESS:
                time.sleep(0.2)
                handler.write1ByteTxRx(nid, lock_reg, 1) # Lock EPROM
                
                # Check online status at new ID
                _, _, ping_res = ping_and_detect(sts_handler, nid)
                if ping_res == COMM_SUCCESS:
                    print(f"{C_GREEN}SUCCESS! Permanently set to ID: {nid}{C_RST}")
                    if sid in detected_types:
                        detected_types[nid] = detected_types.pop(sid)
                    if sid in detected_models:
                        detected_models[nid] = detected_models.pop(sid)
                    if active_id == sid:
                        active_id = nid
                else:
                    print(f"{C_RED}Verification failed. Servo did not respond at new ID {nid}.{C_RST}")
            else:
                print(f"{C_RED}Error writing ID: {handler.getTxRxResult(res)}{C_RST}")
                handler.write1ByteTxRx(sid, lock_reg, 1)
            
        elif choice == '12':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            handler = sts_handler if servo_type == 'ST' else sc_handler
            lock_reg = 55 if servo_type == 'ST' else 48
            max_limit = 1023 if (servo_type == 'SC' and detected_models.get(sid) in [15, 9]) else 4095
            
            print(f"Guidance: Restricts travel range. Set both to 0 to enable continuous rotation mode.")
            print(f"Range: 0 to {max_limit}")
            min_ang = get_int(f"Min Angle Limit (0-{max_limit}): ", min_val=0, max_val=max_limit)
            if min_ang is None: continue
            max_ang = get_int(f"Max Angle Limit (0-{max_limit}): ", min_val=0, max_val=max_limit)
            if max_ang is None: continue
            
            handler.write1ByteTxRx(sid, 40, 0)
            handler.write1ByteTxRx(sid, lock_reg, 0) # Unlock
            
            res1, _ = handler.write2ByteTxRx(sid, 9, min_ang)
            res2, _ = handler.write2ByteTxRx(sid, 11, max_ang)
            time.sleep(0.1)
            
            handler.write1ByteTxRx(sid, lock_reg, 1) # Lock
            
            if res1 == COMM_SUCCESS and res2 == COMM_SUCCESS:
                print(f"{C_GREEN}Angle limits updated successfully.{C_RST}")
            else:
                print(f"{C_RED}Failed to write limits.{C_RST}")
            
        elif choice == '13':
            if active_id is None:
                print(f"{C_RED}Please select an active servo first.{C_RST}")
                time.sleep(1.5)
                continue
                
            sid = get_int(f"Enter Servo ID [Default: {active_id}]: ", default=active_id)
            if sid is None: continue
            
            servo_type = get_servo_type(sts_handler, sid)
            if servo_type != 'ST':
                print(f"{C_RED}Hardware position offset calibration is not supported on SC series servos.{C_RST}")
                time.sleep(2)
                continue
                
            print("Guidance: Shifts the zero-point of the encoder. Value in range -2048 to 2048.")
            ofs = get_int("Offset value (-2048 to 2048): ", min_val=-2048, max_val=2048)
            if ofs is None: continue
            
            val = sts_handler.sts_toscs(ofs, 11)
            sts_handler.write1ByteTxRx(sid, 40, 0)
            sts_handler.write1ByteTxRx(sid, 55, 0)
            
            res, _ = sts_handler.write2ByteTxRx(sid, 31, val)
            time.sleep(0.1)
            
            sts_handler.write1ByteTxRx(sid, 55, 1)
            
            if res == COMM_SUCCESS:
                print(f"{C_GREEN}Offset permanently calibrated.{C_RST}")
            else:
                print(f"{C_RED}Error: {sts_handler.getTxRxResult(res)}{C_RST}")
            
        elif choice == '14':
            print_header("Teach-Playback Motion Recorder")
            print(" 1. Record New Motion Sequence")
            print(" 2. Playback Existing Motion Sequence")
            print("\n 0. Back to Main Menu")
            print(f"{C_CYA}{'-'*60}{C_RST}")
            sub_choice = input("Select an option (0-2): ").strip()
            
            if sub_choice == '1':
                default_ids = str(active_id) if active_id is not None else ""
                val = input(f"Enter Servo IDs to record (comma separated) [Default: {default_ids}]: ").strip()
                if not val:
                    if active_id is not None:
                        sids = [active_id]
                    else:
                        print(f"{C_RED}No servos selected.{C_RST}")
                        time.sleep(1.5)
                        continue
                else:
                    try:
                        sids = [int(x.strip()) for x in val.replace(',', ' ').split() if x.strip()]
                    except ValueError:
                        print(f"{C_RED}Invalid ID list.{C_RST}")
                        time.sleep(1.5)
                        continue
                filename = input("Enter save filename [Default: recorded_motion.json]: ").strip()
                if not filename:
                    filename = "recorded_motion.json"
                record_motion(sts_handler, sc_handler, sids, filename)
            elif sub_choice == '2':
                filename = input("Enter playback filename [Default: recorded_motion.json]: ").strip()
                if not filename:
                    filename = "recorded_motion.json"
                play_motion(sts_handler, sc_handler, filename)
            print_menu_next = True
            
        elif choice == '15':
            advanced_debugging_menu(sts_handler, sc_handler, active_id)
            print_menu_next = True
            
        elif choice == '0':
            print(f"\n{C_GREEN}Exiting Dashboard... Goodbye!{C_RST}")
            break
        else:
            print(f"{C_RED}Invalid option. Try again.{C_RST}")
            time.sleep(1)
            
    portHandler.closePort()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{C_RED}Program terminated. Exiting.{C_RST}")
        sys.exit(0)
