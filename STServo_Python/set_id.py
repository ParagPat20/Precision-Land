#!/usr/bin/env python
#
# *********     Interactive Set ID      *********
#
# This script:
# 1. Scans for connected servos to detect current ID.
# 2. Prompts the user for a new ID.
# 3. Updates the servo ID and saves it to EPROM.
#

import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *

# Default setting
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM21'

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

def scan_for_servo():
    print("Scanning for connected servos (IDs 0-253)...")
    found_ids = []
    # Set a short timeout for scanning
    portHandler.setPacketTimeout(10) # 10ms is plenty for 1Mbps
    
    for sts_id in range(0, 254):
        # Print progress every 20 IDs
        if sts_id % 20 == 0:
            print("Checking IDs %d-%d..." % (sts_id, min(sts_id+19, 253)))
            
        model_number, result, error = packetHandler.ping(sts_id)
        if result == COMM_SUCCESS:
            print(">>> Found Servo! ID: %d (Model: %d)" % (sts_id, model_number))
            found_ids.append(sts_id)
            # If the user only has one servo, we can stop here or continue
            # Let's continue a bit to see if there are others, but stop if we find one 
            # to make it faster for the user.
            break 
            
    return found_ids

# Open port
if not portHandler.openPort():
    print("Failed to open the port")
    quit()

# Set port baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()

# 1. Detect Current ID
found = scan_for_servo()
if not found:
    print("No servos detected. Check connections, power, and baudrate (currently %d)." % BAUDRATE)
    portHandler.closePort()
    quit()

current_id = found[0]

# 2. User Input for New ID
print("\n--- ID Configuration ---")
print("Current ID: %d" % current_id)
val = input("Enter the NEW ID you want to set (0-253): ")

try:
    new_id = int(val)
    if new_id < 0 or new_id > 253:
        raise ValueError
except ValueError:
    print("Invalid ID. Please enter a number between 0 and 253.")
    portHandler.closePort()
    quit()

if new_id == current_id:
    print("New ID is the same as current ID. No changes made.")
    portHandler.closePort()
    quit()

# 3. Perform Change
print("Applying change: ID %d -> ID %d..." % (current_id, new_id))

# Disable Torque before changing ID (recommended for EPROM writes)
packetHandler.write1ByteTxRx(current_id, STS_TORQUE_ENABLE, 0)

# Unlock EPROM (writing 0 to both address 48 and 55 to cover all STS models)
packetHandler.write1ByteTxRx(current_id, 48, 0)
sts_comm_result, sts_error = packetHandler.unLockEprom(current_id)
if sts_comm_result != COMM_SUCCESS:
    print("Warning: Failed to unlock EPROM at addr 55: %s" % packetHandler.getTxRxResult(sts_comm_result))

# Write New ID
sts_comm_result, sts_error = packetHandler.write1ByteTxRx(current_id, STS_ID, new_id)
if sts_comm_result != COMM_SUCCESS:
    print("Failed to write new ID: %s" % packetHandler.getTxRxResult(sts_comm_result))
    # Try to lock it back just in case
    packetHandler.write1ByteTxRx(current_id, 48, 1)
    packetHandler.LockEprom(current_id)
    portHandler.closePort()
    quit()

# Give it a tiny bit of time for the EPROM to commit the write
import time
time.sleep(0.1)

# Lock EPROM (using the NEW ID, locking both 48 and 55)
packetHandler.write1ByteTxRx(new_id, 48, 1)
sts_comm_result, sts_error = packetHandler.LockEprom(new_id)
if sts_comm_result != COMM_SUCCESS:
    print("Note: Lock command returned %s (ID might have changed already)." % packetHandler.getTxRxResult(sts_comm_result))
else:
    print("Successfully locked EPROM.")

# 4. Verify
print("Verifying new ID...")
model_number, result, error = packetHandler.ping(new_id)
if result == COMM_SUCCESS:
    print("SUCCESS! Servo is now responding at ID: %d" % new_id)
else:
    print("Verification failed. Servo is not responding at new ID.")

# Close port
portHandler.closePort()
