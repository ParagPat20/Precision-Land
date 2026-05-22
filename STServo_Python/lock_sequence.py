import os
import sys
import time

# Ensure we can import the SDK from the parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from STservo_sdk import *

# --- Cross-Platform Non-Blocking Input ---
if os.name == 'nt':
    import msvcrt
    def kbhit_safe():
        return msvcrt.kbhit()
    def getch_safe():
        return msvcrt.getch().decode('utf-8', errors='ignore')
else:
    import select
    import termios
    import tty
    def kbhit_safe():
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []
    def getch_safe():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
# -----------------------------------------

# --- CONFIGURATION ---
if os.name == 'nt':
    DEVICENAME = 'COM21'
else:
    DEVICENAME = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110734-if00'
BAUDRATE = 1000000

# Speeds & Accel
ST_SPEED = 2400
ST_ACC = max(0, min(254, 200)) # Clamped safely between 0-254
SC_SPEED = 1500

# Locking Targets
LOCK_POS_1 = 150
LOCK_POS_2 = 960
LOCK_POS_3 = 520

# Unlocking Targets
UNLOCK_POS_1 = 2000
UNLOCK_POS_2 = 790
UNLOCK_POS_3 = 690
UNLOCK_CHECK_3 = 630  # Threshold check for ID 3
# ---------------------

def setup_handlers():
    portHandler = PortHandler(DEVICENAME)
    sts_handler = sts(portHandler)
    sc_handler = scscl(portHandler)
    
    try:
        if not portHandler.openPort():
            print("Failed to open port. Make sure the dashboard is closed before running this script.")
            sys.exit(1)
        if not portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the baudrate")
            sys.exit(1)
    except Exception as e:
        print(f"Error opening port: {e}")
        print("Please close any other programs (like the servo dashboard) using COM21.")
        sys.exit(1)
        
    print(f"Connected to {portHandler.getPortName()} at {portHandler.getBaudRate()} bps")
    return sts_handler, sc_handler

def ping_servos(sts_handler, sc_handler):
    print("\nPinging servos to verify availability...")
    all_ok = True
    
    # Ping ID 1 (ST)
    model, res, _ = sts_handler.ping(1)
    if res == COMM_SUCCESS:
        print("  - Servo 1 (ST): ONLINE")
    else:
        print("  - Servo 1 (ST): OFFLINE")
        all_ok = False
        
    # Ping ID 2 (SC)
    model, res, _ = sc_handler.ping(2)
    if res == COMM_SUCCESS:
        print("  - Servo 2 (SC): ONLINE")
    else:
        print("  - Servo 2 (SC): OFFLINE")
        all_ok = False
        
    # Ping ID 3 (SC)
    model, res, _ = sc_handler.ping(3)
    if res == COMM_SUCCESS:
        print("  - Servo 3 (SC): ONLINE")
    else:
        print("  - Servo 3 (SC): OFFLINE")
        all_ok = False
        
    return all_ok

def robust_move_st_single(sts_handler, sid, target, speed, acc, timeout=10.0):
    print(f"Moving Servo {sid} to {target} (timeout {timeout}s)...")
    start_time = time.time()
    
    sts_handler.write1ByteTxRx(sid, 40, 1) 
    sts_handler.WritePosEx(sid, target, speed, acc)
    
    last_pos = -1
    start_pos = None
    stuck_count = 0
    wiggle_dir = 1
    tolerance = 30
    
    while time.time() - start_time < timeout:
        time.sleep(0.3)
        
        pos, res, _ = sts_handler.ReadPos(sid)
        if res == COMM_SUCCESS:
            if start_pos is None:
                start_pos = pos
            print(f"  [ID {sid}] Current Pos: {pos} | Target: {target} (Start: {start_pos})")
            
            is_reached = False
            if start_pos is not None:
                if target < start_pos:
                    is_reached = (pos <= target + tolerance)
                else:
                    is_reached = (pos >= target - tolerance)
            else:
                is_reached = (abs(pos - target) <= tolerance)
                
            if is_reached:
                print(f"  -> Servo {sid} reached target!")
                return True
                
            if abs(pos - last_pos) < 3:
                stuck_count += 1
                if stuck_count >= 2:
                    wiggle_target = target + (100 * wiggle_dir)
                    print(f"  [ID {sid}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                    sts_handler.write1ByteTxRx(sid, 40, 1) 
                    sts_handler.WritePosEx(sid, int(wiggle_target), speed, acc)
                    wiggle_dir *= -1
                    stuck_count = 0
            else:
                stuck_count = 0
            last_pos = pos
        else:
            print(f"  [ID {sid}] Failed to read position (possibly resetting)...")
            stuck_count = 5 # Force resend next successful read
            
    print(f"  -> Timeout reached for Servo {sid}! Did not reach {target}.")
    return False

def robust_move_sc_pair(sc_handler, sid2, target2, sid3, target3, speed, timeout=15.0, check_target3=None, check_dir3='>='):
    print(f"Moving Servo {sid2} to {target2} and Servo {sid3} to {target3} together...")
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
    sc_handler.write1ByteTxRx(sid2, 40, 1)
    sc_handler.WritePos(sid2, target2, 0, speed)
    sc_handler.write1ByteTxRx(sid3, 40, 1)
    sc_handler.WritePos(sid3, target3, 0, speed)
    
    while time.time() - start_time < timeout:
        time.sleep(0.3)
        
        if not reached2:
            pos2, res2, _ = sc_handler.ReadPos(sid2)
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
                            wiggle_target = target2 + (100 * wiggle_dir2)
                            print(f"  [ID {sid2}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                            sc_handler.write1ByteTxRx(sid2, 40, 1)
                            sc_handler.WritePos(sid2, int(wiggle_target), 0, speed)
                            wiggle_dir2 *= -1
                            stuck_count2 = 0
                    else:
                        stuck_count2 = 0
                last_pos2 = pos2
            else:
                print(f"  [ID {sid2}] Read failed (possibly resetting)...")
                stuck_count2 = 5
                
        if not reached3:
            pos3, res3, _ = sc_handler.ReadPos(sid3)
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
                            wiggle_target = target3 + (100 * wiggle_dir3)
                            print(f"  [ID {sid3}] JAM DETECTED! Jiggling target to {wiggle_target} to build momentum...")
                            sc_handler.write1ByteTxRx(sid3, 40, 1)
                            sc_handler.WritePos(sid3, int(wiggle_target), 0, speed)
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

def perform_locking(sts_handler, sc_handler):
    print("\n--- LOCKING SEQUENCE ---")
    print(f"Step 1: Servo 1 -> {LOCK_POS_1}")
    robust_move_st_single(sts_handler, 1, LOCK_POS_1, ST_SPEED, ST_ACC)
    
    print("\nWaiting 1s for mechanical settlement...")
    time.sleep(1.0)
    
    print(f"\nStep 2: Servo 2 -> {LOCK_POS_2} & Servo 3 -> {LOCK_POS_3}")
    robust_move_sc_pair(sc_handler, 2, LOCK_POS_2, 3, LOCK_POS_3, SC_SPEED, check_target3=LOCK_POS_3, check_dir3='<=')
    print("\nLocking sequence complete!")

def perform_unlocking(sts_handler, sc_handler):
    print("\n--- UNLOCKING SEQUENCE ---")
    print(f"Step 1: Servo 2 -> {UNLOCK_POS_2} & Servo 3 -> {UNLOCK_POS_3} (Checking if >= {UNLOCK_CHECK_3})")
    robust_move_sc_pair(sc_handler, 2, UNLOCK_POS_2, 3, UNLOCK_POS_3, SC_SPEED, check_target3=UNLOCK_CHECK_3, check_dir3='>=')
    
    print(f"\nStep 2: Servo 1 -> {UNLOCK_POS_1}")
    robust_move_st_single(sts_handler, 1, UNLOCK_POS_1, ST_SPEED, ST_ACC)
    print("\nUnlocking sequence complete!")

if __name__ == '__main__':
    print("Initializing Servos...")
    sts_handler, sc_handler = setup_handlers()
    
    # Ping servos before anything
    if not ping_servos(sts_handler, sc_handler):
        print("\nWARNING: One or more servos are offline. Sequence may fail or loop until timeout.")
        proceed = input("Proceed anyway? (y/n): ").strip().lower()
        if proceed != 'y':
            sys.exit(1)
            
    # Ensure torque is enabled initially
    sts_handler.write1ByteTxRx(1, 40, 1)
    sc_handler.write1ByteTxRx(2, 40, 1)
    sc_handler.write1ByteTxRx(3, 40, 1)
    
    print("\n┌─────────────────────────────┐")
    print("│      MECHANISM CONTROL      │")
    print("└─────────────────────────────┘")
    print("Press '1' to Execute Locking Sequence")
    print("Press '2' to Execute Unlocking Sequence")
    print("Press '3' or 'q' to Exit")
    print("\nWaiting for input... (Servos are actively held in place)")
    
    last_state = 'lock'
    perform_locking(sts_handler, sc_handler)
    
    while True:
        if kbhit_safe():
            choice = getch_safe().strip()
            
            if choice == '1':
                perform_locking(sts_handler, sc_handler)
                last_state = 'lock'
                print("\nWaiting for input... (Servos are actively held in place)")
            elif choice == '2':
                perform_unlocking(sts_handler, sc_handler)
                last_state = 'unlock'
                print("\nWaiting for input... (Servos are actively held in place)")
            elif choice in ['3', 'q', 'Q']:
                print("\nExiting and disabling torque...")
                sts_handler.write1ByteTxRx(1, 40, 0)
                sc_handler.write1ByteTxRx(2, 40, 0)
                sc_handler.write1ByteTxRx(3, 40, 0)
                break
                
        # Active Background Holding Loop (PID Compensator)
        # If a servo resets, this immediately re-enables torque and pushes the target again
        if last_state == 'lock':
            sts_handler.write1ByteTxRx(1, 40, 1)
            sts_handler.WritePosEx(1, LOCK_POS_1, ST_SPEED, ST_ACC)
            sc_handler.write1ByteTxRx(2, 40, 1)
            sc_handler.WritePos(2, LOCK_POS_2, 0, SC_SPEED)
            sc_handler.write1ByteTxRx(3, 40, 1)
            sc_handler.WritePos(3, LOCK_POS_3, 0, SC_SPEED)
        elif last_state == 'unlock':
            sts_handler.write1ByteTxRx(1, 40, 1)
            sts_handler.WritePosEx(1, UNLOCK_POS_1, ST_SPEED, ST_ACC)
            sc_handler.write1ByteTxRx(2, 40, 1)
            sc_handler.WritePos(2, UNLOCK_POS_2, 0, SC_SPEED)
            sc_handler.write1ByteTxRx(3, 40, 1)
            sc_handler.WritePos(3, UNLOCK_POS_3, 0, SC_SPEED)
            
        time.sleep(0.1) # Run holding loop at 10Hz
