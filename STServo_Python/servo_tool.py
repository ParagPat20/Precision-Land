#!/usr/bin/env python
#
# *********     STServo Advanced All-in-One Dashboard      *********
#
# A comprehensive interactive CLI tool to control STServos.
#

import sys
import os
import time

sys.path.append("..")
from STservo_sdk import *

# Default Settings
BAUDRATE = 1000000
DEVICENAME = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110734-if00'

# ANSI Colors for UI
C_RED = '\033[91m'
C_GREEN = '\033[92m'
C_YEL = '\033[93m'
C_BLUE = '\033[94m'
C_CYA = '\033[96m'
C_RST = '\033[0m'

def print_header(title):
    print(f"\n{C_CYA}{'='*50}{C_RST}")
    print(f"{C_YEL}{title.center(50)}{C_RST}")
    print(f"{C_CYA}{'='*50}{C_RST}")

def get_int(prompt, default=None):
    try:
        val = input(f"{prompt}").strip()
        if not val and default is not None:
            return default
        return int(val)
    except ValueError:
        print(f"{C_RED}Invalid input. Please enter a valid number.{C_RST}")
        return None

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    clear_screen()
    print_header("STServo Advanced Control Dashboard")
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = sts(portHandler)

    if not portHandler.openPort():
        print(f"{C_RED}Error: Failed to open port {DEVICENAME}{C_RST}")
        quit()
        
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"{C_RED}Error: Failed to set baudrate to {BAUDRATE}{C_RST}")
        quit()
        
    print(f"{C_GREEN}Successfully connected to {DEVICENAME} at {BAUDRATE} bps{C_RST}\n")
    
    while True:
        print(f"\n{C_BLUE}--- Core Functions ---{C_RST}")
        print(" 1. Scan for Servos (IDs 0-253)")
        print(" 2. Ping Specific Servo")
        print(" 3. Read Full Status (Pos, Spd, Volts, Temp, Load, Current)")
        print(" 4. Live Sensor Monitor (Supports Multiple Servos)")
        
        print(f"\n{C_BLUE}--- Movement Controls ---{C_RST}")
        print(" 5. Absolute Move (Position Control)")
        print(" 6. Relative Jog (Step Move Forward/Backward)")
        print(" 7. Move to Center / Rest Angle (Position 2048)")
        print(" 8. Continuous Rotation (Wheel Mode)")
        print(" 9. Toggle Torque (Enable/Disable Holding)")
        
        print(f"\n{C_BLUE}--- Configuration & Setup ---{C_RST}")
        print("10. Read Device Config (Limits, Mode, Torque)")
        print("11. Set Servo ID (Safe permanent EPROM save)")
        print("12. Set Angle Limits")
        print("13. Set Position Offset")
        
        print(f"\n{C_BLUE}--- Debugging & Advanced ---{C_RST}")
        print("14. Read Raw Register (1/2/4 bytes)")
        print("15. Write Raw Register (1/2/4 bytes)")
        print("16. Send Action / Reboot Command")
        
        print(f"\n{C_RED} 0. Exit Dashboard{C_RST}")
        print(f"{C_CYA}{'-'*50}{C_RST}")
        
        choice = input("Select an option (0-16): ").strip()
        
        if choice == '1':
            print_header("Scan All Servos")
            print("Scanning IDs 0-253... Please wait.")
            portHandler.setPacketTimeout(10)
            found = []
            for sts_id in range(254):
                model_number, result, _ = packetHandler.ping(sts_id)
                if result == COMM_SUCCESS:
                    print(f"{C_GREEN}  >>> Found Servo ID: {sts_id} (Model: {model_number}){C_RST}")
                    found.append(sts_id)
            if not found:
                print(f"{C_RED}  No servos found. Check connections/power.{C_RST}")
            portHandler.setPacketTimeout(50) 
            
        elif choice == '2':
            print_header("Ping Specific Servo")
            sid = get_int("Enter Servo ID: ")
            if sid is None: continue
            model, res, err = packetHandler.ping(sid)
            if res == COMM_SUCCESS:
                print(f"{C_GREEN}Success! Servo {sid} is online. Model: {model}{C_RST}")
            else:
                print(f"{C_RED}Failed to ping servo {sid}: {packetHandler.getTxRxResult(res)}{C_RST}")

        elif choice == '3':
            print_header("Read Full Status")
            sid = get_int("Enter Servo ID: ")
            if sid is None: continue
            
            pos, spd, res, err = packetHandler.ReadPosSpeed(sid)
            if res != COMM_SUCCESS:
                print(f"{C_RED}Failed to read: {packetHandler.getTxRxResult(res)}{C_RST}")
                continue
                
            volts, _, _ = packetHandler.read1ByteTxRx(sid, STS_PRESENT_VOLTAGE)
            temp, _, _ = packetHandler.read1ByteTxRx(sid, STS_PRESENT_TEMPERATURE)
            load, _, _ = packetHandler.read2ByteTxRx(sid, STS_PRESENT_LOAD_L)
            curr, _, _ = packetHandler.read2ByteTxRx(sid, STS_PRESENT_CURRENT_L)
            moving, _, _ = packetHandler.read1ByteTxRx(sid, STS_MOVING)
            
            is_moving = "Yes" if moving > 0 else "No"
            dir_load = "-" if (load & (1<<10)) else ""
            val_load = load & ~(1<<10)
            
            print(f"\n{C_YEL}--- Servo {sid} Complete Status ---{C_RST}")
            print(f" Position    : {pos}")
            print(f" Speed       : {packetHandler.sts_tohost(spd, 15)}")
            print(f" Load        : {dir_load}{val_load}")
            print(f" Current     : {curr} mA")
            print(f" Voltage     : {volts / 10.0:.1f} V")
            print(f" Temperature : {temp} °C")
            print(f" Is Moving   : {is_moving}")
            
        elif choice == '4':
            print_header("Live Sensor Monitor")
            ids_str = input("Enter Servo IDs (comma separated, e.g. 1,2,3): ").strip()
            try:
                sids = [int(x.strip()) for x in ids_str.split(',') if x.strip()]
            except ValueError:
                print(f"{C_RED}Invalid input.{C_RST}")
                continue
            if not sids: continue
            
            print(f"{C_YEL}Monitoring Servos {sids}... Press Ctrl+C to stop.{C_RST}")
            print("\n" * len(sids)) # Make room for the lines
            try:
                while True:
                    sys.stdout.write(f"\033[{len(sids)}A") # Move cursor up
                    for sid in sids:
                        pos, spd, res, err = packetHandler.ReadPosSpeed(sid)
                        volts, _, _ = packetHandler.read1ByteTxRx(sid, STS_PRESENT_VOLTAGE)
                        temp, _, _ = packetHandler.read1ByteTxRx(sid, STS_PRESENT_TEMPERATURE)
                        curr, _, _ = packetHandler.read2ByteTxRx(sid, STS_PRESENT_CURRENT_L)
                        if res == COMM_SUCCESS:
                            sys.stdout.write(f"\033[KID {sid:3d} | Pos: {pos:4d} | Spd: {packetHandler.sts_tohost(spd, 15):5d} | Cur: {curr:4d}mA | V: {volts/10.0:.1f}V | T: {temp}°C\n")
                        else:
                            sys.stdout.write(f"\033[KID {sid:3d} | Offline or No Response\n")
                    sys.stdout.flush()
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print(f"\n{C_YEL}Live Monitor stopped.{C_RST}")
                
        elif choice == '5':
            print_header("Absolute Move")
            sid = get_int("Enter Servo ID: ")
            pos = get_int("Target Position (0-4095): ")
            spd = get_int("Speed (0-3000) [Default 2400]: ", 2400)
            acc = get_int("Acceleration (0-254) [Default 50]: ", 50)
            if None in [sid, pos, spd, acc]: continue
            
            packetHandler.write1ByteTxRx(sid, STS_MODE, 0) # Pos mode
            res, err = packetHandler.WritePosEx(sid, pos, spd, acc)
            if res == COMM_SUCCESS: print(f"{C_GREEN}Move command sent.{C_RST}")
            else: print(f"{C_RED}Error: {packetHandler.getTxRxResult(res)}{C_RST}")
                
        elif choice == '6':
            print_header("Relative Jog")
            sid = get_int("Enter Servo ID: ")
            step = get_int("Step amount (e.g. 100 or -100): ")
            spd = get_int("Speed (0-3000) [Default 2400]: ", 2400)
            if None in [sid, step, spd]: continue
            
            pos, _, res, _ = packetHandler.ReadPosSpeed(sid)
            if res == COMM_SUCCESS:
                new_pos = max(0, min(4095, pos + step))
                print(f"Current Pos: {pos} -> Target Pos: {new_pos}")
                res, err = packetHandler.WritePosEx(sid, new_pos, spd, 50)
                if res == COMM_SUCCESS: print(f"{C_GREEN}Jog command sent.{C_RST}")
            else:
                print(f"{C_RED}Failed to read current position.{C_RST}")
                
        elif choice == '7':
            print_header("Move to Center / Rest Angle")
            sid = get_int("Enter Servo ID: ")
            spd = get_int("Speed (0-3000) [Default 2400]: ", 2400)
            acc = get_int("Acceleration (0-254) [Default 50]: ", 50)
            if None in [sid, spd, acc]: continue
            
            packetHandler.write1ByteTxRx(sid, STS_MODE, 0) # Pos mode
            res, err = packetHandler.WritePosEx(sid, 2048, spd, acc) # 2048 is center
            if res == COMM_SUCCESS: print(f"{C_GREEN}Move to center command sent.{C_RST}")
            else: print(f"{C_RED}Error: {packetHandler.getTxRxResult(res)}{C_RST}")

        elif choice == '8':
            print_header("Wheel Mode (Continuous Rotation)")
            print("Note: Servo spins continuously like a motor.")
            sid = get_int("Enter Servo ID: ")
            spd = get_int("Speed (-3200 to 3200, 0 to stop): ")
            if None in [sid, spd]: continue
            
            packetHandler.WheelMode(sid)
            res, err = packetHandler.WriteSpec(sid, spd, 50)
            if res == COMM_SUCCESS: print(f"{C_GREEN}Wheel command sent.{C_RST}")
            else: print(f"{C_RED}Error: {packetHandler.getTxRxResult(res)}{C_RST}")

        elif choice == '9':
            print_header("Toggle Torque")
            sid = get_int("Enter Servo ID: ")
            state = get_int("State (1:Enable, 0:Disable): ")
            if None in [sid, state]: continue
            res, err = packetHandler.write1ByteTxRx(sid, STS_TORQUE_ENABLE, state)
            if res == COMM_SUCCESS: print(f"{C_GREEN}Torque state updated.{C_RST}")
            else: print(f"{C_RED}Error: {packetHandler.getTxRxResult(res)}{C_RST}")

        elif choice == '10':
            print_header("Read Device Configuration")
            sid = get_int("Enter Servo ID: ")
            if sid is None: continue
            
            min_ang, _, _ = packetHandler.read2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L)
            max_ang, _, _ = packetHandler.read2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L)
            ofs, _, _ = packetHandler.read2ByteTxRx(sid, STS_OFS_L)
            mode, _, _ = packetHandler.read1ByteTxRx(sid, STS_MODE)
            torq, _, _ = packetHandler.read1ByteTxRx(sid, STS_TORQUE_ENABLE)
            baud, _, _ = packetHandler.read1ByteTxRx(sid, STS_BAUD_RATE)
            
            print(f"\n{C_YEL}--- Config for Servo {sid} ---{C_RST}")
            print(f" Min Angle Limit : {min_ang}")
            print(f" Max Angle Limit : {max_ang}")
            print(f" Position Offset : {packetHandler.sts_tohost(ofs, 11)}")
            print(f" Work Mode       : {mode} (0:Pos, 1:Wheel, 2:Step, 3:Multi-turn)")
            print(f" Torque Enable   : {torq}")
            print(f" Baudrate Index  : {baud}")

        elif choice == '11':
            print_header("Set Servo ID")
            sid = get_int("Enter Current Servo ID: ")
            nid = get_int("Enter NEW Servo ID (0-253): ")
            if None in [sid, nid] or sid == nid: continue
            
            print(f"{C_YEL}Applying change: ID {sid} -> ID {nid}...{C_RST}")
            packetHandler.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 0)
            
            packetHandler.write1ByteTxRx(sid, 48, 0)
            packetHandler.unLockEprom(sid)
            
            res, _ = packetHandler.write1ByteTxRx(sid, STS_ID, nid)
            if res != COMM_SUCCESS:
                print(f"{C_RED}Error writing ID: {packetHandler.getTxRxResult(res)}{C_RST}")
                packetHandler.write1ByteTxRx(sid, 48, 1)
                packetHandler.LockEprom(sid)
                continue
                
            time.sleep(0.1)
            packetHandler.write1ByteTxRx(nid, 48, 1)
            packetHandler.LockEprom(nid)
            
            _, res, _ = packetHandler.ping(nid)
            if res == COMM_SUCCESS: print(f"{C_GREEN}SUCCESS! Permanently set to ID: {nid}{C_RST}")
            else: print(f"{C_RED}Verification failed.{C_RST}")

        elif choice == '12':
            print_header("Set Angle Limits")
            sid = get_int("Enter Servo ID: ")
            min_ang = get_int("Min Angle (0-4095): ")
            max_ang = get_int("Max Angle (0-4095): ")
            if None in [sid, min_ang, max_ang]: continue
            
            packetHandler.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 0)
            packetHandler.write1ByteTxRx(sid, 48, 0)
            packetHandler.unLockEprom(sid)
            
            packetHandler.write2ByteTxRx(sid, STS_MIN_ANGLE_LIMIT_L, min_ang)
            packetHandler.write2ByteTxRx(sid, STS_MAX_ANGLE_LIMIT_L, max_ang)
            time.sleep(0.1)
            
            packetHandler.write1ByteTxRx(sid, 48, 1)
            packetHandler.LockEprom(sid)
            print(f"{C_GREEN}Limits updated.{C_RST}")

        elif choice == '13':
            print_header("Set Position Offset")
            sid = get_int("Enter Servo ID: ")
            ofs = get_int("Offset value (-2048 to 2048): ")
            if None in [sid, ofs]: continue
            
            val = packetHandler.sts_toscs(ofs, 11)
            packetHandler.write1ByteTxRx(sid, STS_TORQUE_ENABLE, 0)
            packetHandler.write1ByteTxRx(sid, 48, 0)
            packetHandler.unLockEprom(sid)
            
            packetHandler.write2ByteTxRx(sid, STS_OFS_L, val)
            time.sleep(0.1)
            
            packetHandler.write1ByteTxRx(sid, 48, 1)
            packetHandler.LockEprom(sid)
            print(f"{C_GREEN}Offset updated.{C_RST}")

        elif choice == '14':
            print_header("Read Raw Register")
            sid = get_int("Enter Servo ID: ")
            addr = get_int("Enter Register Address: ")
            size = get_int("Enter Byte Size (1, 2, or 4): ")
            if None in [sid, addr, size]: continue
            
            if size == 1: val, res, _ = packetHandler.read1ByteTxRx(sid, addr)
            elif size == 2: val, res, _ = packetHandler.read2ByteTxRx(sid, addr)
            elif size == 4: val, res, _ = packetHandler.read4ByteTxRx(sid, addr)
            else: print(f"{C_RED}Invalid size.{C_RST}"); continue
            
            if res == COMM_SUCCESS: print(f"{C_GREEN}Value at {addr} ({size} bytes): {val} (0x{val:X}){C_RST}")
            else: print(f"{C_RED}Error: {packetHandler.getTxRxResult(res)}{C_RST}")

        elif choice == '15':
            print_header("Write Raw Register")
            sid = get_int("Enter Servo ID: ")
            addr = get_int("Enter Register Address: ")
            size = get_int("Enter Byte Size (1, 2, or 4): ")
            val = get_int("Enter Value to write: ")
            if None in [sid, addr, size, val]: continue
            
            if size == 1: res, _ = packetHandler.write1ByteTxRx(sid, addr, val)
            elif size == 2: res, _ = packetHandler.write2ByteTxRx(sid, addr, val)
            elif size == 4: res, _ = packetHandler.write4ByteTxRx(sid, addr, val)
            else: print(f"{C_RED}Invalid size.{C_RST}"); continue
            
            if res == COMM_SUCCESS: print(f"{C_GREEN}Successfully wrote {val} to address {addr}{C_RST}")
            else: print(f"{C_RED}Error: {packetHandler.getTxRxResult(res)}{C_RST}")

        elif choice == '16':
            print_header("Send Action / Reboot")
            print("Note: This sends an INST_ACTION command.")
            sid = get_int("Enter Servo ID (254 for Broadcast): ")
            if sid is None: continue
            res = packetHandler.action(sid)
            print(f"{C_GREEN}Action command sent.{C_RST}")

        elif choice == '0':
            print(f"\n{C_GREEN}Exiting STServo Dashboard... Goodbye!{C_RST}")
            break
        else:
            print(f"{C_RED}Invalid option. Try again.{C_RST}")

    portHandler.closePort()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{C_RED}Program interrupted by user. Exiting.{C_RST}")
        sys.exit(0)
