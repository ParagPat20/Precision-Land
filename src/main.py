"""
Precision Landing System with Firebase Remote Dispatch Integration

NOTE: Call it with Mavlink 2.0 as follows:
MAVLINK20=1 python test_.....py

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

FIREBASE INTEGRATION:
-------------------
This system integrates with Firebase Realtime Database for remote mission dispatch and control.

Mission Flow:
1. Flutter app sends mission command with status 'PENDING' to Firebase
2. This script listens for PENDING commands and starts mission execution
3. Status updates: PENDING -> IN_PROGRESS -> COMPLETED/FAILED/ABORTED

Abort Handling:
- Flutter app can send ABORT_REQUESTED status at any time
- Firebase listener detects ABORT_REQUESTED in real-time
- Abort works even before mission starts or during execution
- On abort: switches to RTL mode and updates status to ABORTED

Status Management:
- PENDING: New mission received, ready to process
- IN_PROGRESS: Mission is executing
- COMPLETED: Mission finished successfully
- FAILED: Mission execution failed
- ABORTED: Mission was aborted (either by user or system)
- ABORT_REQUESTED: User requested abort (transient state)
- EXPIRED_STALE: Mission command was too old (>45s)

"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse
import os
from collections import deque  # For rolling stability buffer

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import numpy as np
from opencv.lib_aruco_pose import *
from led_controller import DroneLEDController
import threading
import firebase_admin
from firebase_admin import credentials, db
from core.mission_generator import DeliveryTemplate, LatLng

# --- Configuration ---
DRONE_ID = "Victoris"
# Path to serviceAccountKey.json (Assuming it's in the same folder as this script or adjusted path)
# Since this script is in rpi_mission_manager/Precision-Land/src/, and key is in rpi_mission_manager/
# We need to point up two levels.
CREDENTIALS_PATH = os.path.join(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))), "serviceAccountKey.json")
DATABASE_URL = "https://jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app"

# --- Firebase & Mission Logic ---
firebase_initialized = False
abort_requested = False  # Global abort flag to stop mission execution
active_mission_ref = None  # Reference to current mission command in Firebase
mission_active = False  # Track if a mission is currently active (IN_PROGRESS)

def telemetry_loop(cmd_ref):
    """
    Sends telemetry updates to Firebase while mission is active.
    Also listens for ABORT signals.
    Only sends GPS/telemetry data when mission status is IN_PROGRESS.
    Runs regardless of arming status once mission is uploaded.
    """
    global abort_requested, mission_active
    print("Starting Telemetry Loop...")
    mission_active = True  # Mark mission as active when telemetry loop starts
    
    # Continue sending telemetry while mission is active (not just when armed)
    while mission_active and not abort_requested:
        # 1. Check for Abort (check both global flag and Firebase status)
        try:
            current_status = cmd_ref.child('status').get()
            if current_status == 'ABORT_REQUESTED' or abort_requested:
                print("[ABORT] Received Abort Request! Stopping Mission...")
                abort_requested = True
                mission_active = False  # Stop telemetry immediately
                # Switch to RTL mode for safe return
                try:
                    vehicle.mode = VehicleMode("RTL")
                    print("[ABORT] Vehicle mode set to RTL")
                except Exception as e:
                    print(f"[ABORT] Error setting RTL mode: {e}")
                cmd_ref.update({
                    'status': 'ABORTED',
                    'aborted_at': int(time.time() * 1000)
                })
                print("[ABORT] Mission aborted, returning to launch...")
                return
            
            # Only send telemetry if mission is IN_PROGRESS
            if current_status != 'IN_PROGRESS':
                mission_active = False
                time.sleep(2)
                continue
                
        except Exception as e:
            print(f"Abort Check Error: {e}")

        # 2. Send Telemetry (only when mission is active)
        if mission_active:
            try:
                loc = vehicle.location.global_relative_frame
                if loc is not None and loc.lat is not None and loc.lon is not None:
                    telemetry = {
                        'lat': loc.lat,
                        'lng': loc.lon,
                        'alt': loc.alt,
                        'heading': vehicle.heading,
                        'mode': vehicle.mode.name,
                        'updated_at': int(time.time() * 1000)
                    }
                    cmd_ref.child('telemetry').update(telemetry)
                    print(f"[TELEMETRY] Sent to Firebase: lat={loc.lat:.6f}, lng={loc.lon:.6f}, alt={loc.alt:.1f}m, heading={vehicle.heading}°, mode={vehicle.mode.name}")
            except Exception as e:
                print(f"Telemetry Error: {e}")
            
        time.sleep(2) # 0.5Hz
    
    mission_active = False  # Mark mission as inactive when loop exits

def execute_mission_logic(mission_items, cmd_ref):
    """
    Executes the mission logic: uploads mission, sets mode, arms, and starts telemetry.
    Checks for abort requests throughout the process.
    """
    global abort_requested
    print("Uploading mission via DroneKit...")
    try:
        # Check for abort before starting
        if abort_requested:
            print("[ABORT] Abort requested before mission upload. Cancelling...")
            return False
            
        cmds = vehicle.commands
        cmds.clear()
        
        for item in mission_items:
            if abort_requested:
                print("[ABORT] Abort requested during mission upload. Cancelling...")
                return False
            cmd = Command(
                0, 0, 0, 
                item.frame,
                item.command_id,
                item.current, item.autocontinue,
                item.param1, item.param2, item.param3, item.param4,
                item.x, item.y, item.z
            )
            cmds.add(cmd)
            
        cmds.upload()
        print(f"Mission of {len(mission_items)} items Uploaded!")
        
        # Start telemetry loop immediately after mission upload (regardless of arming status)
        # This allows tracking mission progress even before arming
        print("Starting telemetry transmission to Firebase...")
        telemetry_thread = threading.Thread(target=telemetry_loop, args=(cmd_ref,), name="TelemetryThread", daemon=True)
        telemetry_thread.start()
        
        # Check for abort before setting mode
        if abort_requested:
            print("[ABORT] Abort requested after mission upload. Cancelling...")
            return False
        
        print("Setting Mode to AUTO...")
        vehicle.mode = VehicleMode("AUTO")
        
        # Check for abort before arming
        if abort_requested:
            print("[ABORT] Abort requested before arming. Cancelling...")
            return False
        
        print("Arming...")
        vehicle.armed = True
        
        # Wait up to 5s for arming, checking for abort during wait
        for _ in range(5):
            if abort_requested:
                print("[ABORT] Abort requested during arming wait. Switching to RTL...")
                try:
                    vehicle.mode = VehicleMode("RTL")
                    print("[ABORT] Vehicle mode set to RTL")
                except Exception as e:
                    print(f"[ABORT] Error setting RTL mode: {e}")
                return False
            if vehicle.armed:
                break
            time.sleep(1)
            
        print(f"ARMED Status: {vehicle.armed}")
        
        # Final check for abort after arming wait
        if abort_requested:
            print("[ABORT] Abort requested. Switching to RTL...")
            try:
                vehicle.mode = VehicleMode("RTL")
                print("[ABORT] Vehicle mode set to RTL")
            except Exception as e:
                print(f"[ABORT] Error setting RTL mode: {e}")
            return False
            
        return True
    except Exception as e:
        print(f"Mission Execution Error: {e}")
        return False

def run_mission_thread(command):
    """
    Runs a mission in a separate thread. Handles mission execution and status updates.
    """
    global abort_requested, active_mission_ref, mission_active
    cmd_id = command.get('id')
    timestamp = command.get('timestamp', 0)
    current_time = int(time.time() * 1000)
    
    ref = db.reference(f'missions/{DRONE_ID}/active_command')
    active_mission_ref = ref
    mission_active = False  # Reset mission active flag
    
    # 1. Stale Command Check (45 seconds = 45000 ms)
    if current_time - timestamp > 45000:
        print(f"[REJECT] Mission {cmd_id} is STALE ({(current_time - timestamp)/1000:.1f}s old). Ignoring.")
        mission_active = False
        ref.update({'status': 'EXPIRED_STALE'})
        active_mission_ref = None
        return

    # 2. Check for abort before processing
    try:
        current_status = ref.child('status').get()
        if current_status == 'ABORT_REQUESTED':
            print(f"[ABORT] Mission {cmd_id} was aborted before processing.")
            mission_active = False
            ref.update({'status': 'ABORTED', 'aborted_at': int(time.time() * 1000)})
            active_mission_ref = None
            return
    except Exception as e:
        print(f"Status check error: {e}")

    print(f"[{threading.current_thread().name}] Processing Mission {cmd_id}")
    payload = command.get('payload', {})
    
    target_lat = payload.get('target_lat')
    target_lng = payload.get('target_lng')
    
    if target_lat is None or target_lng is None:
        print("Invalid Target Location")
        mission_active = False
        ref.update({'status': 'FAILED', 'error': 'Invalid target location'})
        active_mission_ref = None
        return

    # Generate Mission
    print("Generating Mission...")
    try:
        template = DeliveryTemplate.get_default_template()
        # Get actual home location from vehicle
        if vehicle.location.global_frame.lat is not None:
            home_loc = LatLng(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        else:
            home_loc = LatLng(0.0, 0.0) 
             
        target_loc = LatLng(target_lat, target_lng)
        
        mission_items = template.generate_mission(
            home_location=home_loc,
            delivery_location=target_loc,
            override_values=payload
        )
    except Exception as e:
        print(f"Mission generation error: {e}")
        mission_active = False
        ref.update({'status': 'FAILED', 'error': str(e)})
        active_mission_ref = None
        return
    
    # Update status to IN_PROGRESS
    ref.update({'status': 'IN_PROGRESS', 'started_at': int(time.time() * 1000)})
    mission_active = True  # Mark mission as active
    
    # Execute mission (this will check for abort internally)
    success = execute_mission_logic(mission_items, ref)

    # Check final status
    try:
        final_status = ref.child('status').get()
        if final_status == 'ABORTED':
            print(f"Mission {cmd_id} ended (ABORTED).")
            mission_active = False
            active_mission_ref = None
            return
        elif success:
            mission_active = False  # Stop telemetry when mission completes
            ref.update({
                'status': 'COMPLETED',
                'completed_at': int(time.time() * 1000)
            })
        else:
            mission_active = False  # Stop telemetry when mission fails
            ref.update({
                'status': 'FAILED',
                'failed_at': int(time.time() * 1000)
            })
    except Exception as e:
        print(f"Status update error: {e}")
    finally:
        mission_active = False
        active_mission_ref = None

def check_mission(command):
    """
    Checks incoming Firebase commands and handles them appropriately.
    Handles PENDING missions and ABORT_REQUESTED status changes.
    """
    global abort_requested, active_mission_ref, mission_active
    
    if not command or not isinstance(command, dict):
        return
    
    status = command.get('status')
    
    # Handle abort requests (can come at any time)
    if status == 'ABORT_REQUESTED':
        print("[ABORT] Abort request received from Firebase!")
        abort_requested = True
        mission_active = False  # Stop sending telemetry immediately
        
        # If there's an active mission, update its status
        if active_mission_ref:
            try:
                active_mission_ref.update({
                    'status': 'ABORTED',
                    'aborted_at': int(time.time() * 1000)
                })
            except Exception as e:
                print(f"Error updating abort status: {e}")
        
        # Always try to switch to RTL mode (regardless of armed status)
        try:
            print("[ABORT] Switching vehicle to RTL mode...")
            vehicle.mode = VehicleMode("RTL")
            print("[ABORT] Vehicle mode set to RTL")
        except Exception as e:
            print(f"Error switching to RTL: {e}")
        
        return
    
    # Handle new PENDING missions
    if status == 'PENDING':
        # Reset abort flag and mission active flag for new mission
        abort_requested = False
        mission_active = False
        sender = command.get('sender_email', 'Unknown')
        print(f"Received PENDING Mission from {sender}")
        t = threading.Thread(target=run_mission_thread, args=(command,), name="MissionThread")
        t.start()

def firebase_listener_thread():
    """
    Firebase listener thread that connects to Firebase and listens for mission commands.
    Handles both initial state and real-time updates.
    """
    global firebase_initialized
    print("Attempting to connect to Firebase (Background Thread)...")
    while not firebase_initialized:
        try:
            if not firebase_admin._apps:
                cred = credentials.Certificate(CREDENTIALS_PATH)
                firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
            
            print("Firebase Connected!")
            firebase_initialized = True
            
            ref = db.reference(f'missions/{DRONE_ID}/active_command')
            
            # Initial check - see if there's already a command
            initial_command = ref.get()
            if initial_command:
                print("Found existing command in Firebase, processing...")
                check_mission(initial_command)
            
            # Listen for real-time updates (including abort requests)
            def on_firebase_event(event):
                """Handle Firebase real-time events"""
                try:
                    if event.data:
                        check_mission(event.data)
                    else:
                        # Command was deleted/cleared
                        print("[Firebase] Active command cleared.")
                except Exception as e:
                    print(f"Firebase event handler error: {e}")
            
            ref.listen(on_firebase_event)
            print("Firebase Listener Active - Listening for commands and abort requests...")
            
        except Exception as e:
            print(f"Firebase Connection Failed: {e}. Retrying in 10s...")
            time.sleep(10)

def heartbeat_thread():
    """
    Writes drone status (online, last_seen, internet) to Firebase so the UI can show
    "Drone online" and "Drone has internet" remotely. Runs every 15s once Firebase is connected.
    """
    while not firebase_initialized:
        time.sleep(2)
    status_ref = db.reference(f'missions/{DRONE_ID}/status')
    interval_sec = 15
    while True:
        try:
            status_ref.update({
                'online': True,
                'last_seen': int(time.time() * 1000),
                'internet': True,  # We have internet if this write succeeds
            })
        except Exception as e:
            print(f"[HEARTBEAT] Write failed (no internet?): {e}")
        time.sleep(interval_sec)


def init_firebase_listener():
    # Start the connection logic in a separate thread so it NEVER blocks the main script
    t = threading.Thread(target=firebase_listener_thread, name="FirebaseConnectionThread", daemon=True)
    t.start()
    # Start heartbeat so the app can show "Drone online" and "Drone has internet"
    h = threading.Thread(target=heartbeat_thread, name="DroneStatusHeartbeat", daemon=True)
    h.start()



parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '/dev/ttyACM0')
args = parser.parse_args()

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------     
# Define function to send landing_target mavlink message for mavlink based precision landing
# http://mavlink.org/messages/common#LANDING_TARGET
def send_land_message_v2(x_rad=0.0, y_rad=0.0, dist_m=0.0, x_m=0.0,y_m=0.0,z_m=0.0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        int(time_usec),          # time target data was processed, as close to sensor capture as possible
        int(target_num),         # target num, not used
        int(mavutil.mavlink.MAV_FRAME_BODY_FRD), # frame, not used
        float(x_rad),            # X-axis angular offset, in radians
        float(y_rad),            # Y-axis angular offset, in radians
        float(dist_m),       # distance, in meters
        0.0,                     # Target x-axis size, in radians
        0.0,                     # Target y-axis size, in radians
        float(x_m),              # x position in frame
        float(y_m),              # y position in frame
        float(z_m),              # z position in frame
        (1.0,0.0,0.0,0.0),       # orientation quaternion
        int(2),                  # type of landing target: 2 = Fiducial marker
        int(0),                  # position_valid boolean
    )
    # print(msg)
    vehicle.send_mavlink(msg)


# Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
# http://mavlink.org/messages/common#DISTANCE_SENSOR
def send_distance_message( dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)     

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav = x_cam
    y_uav = y_cam
    return(x_uav, y_uav)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle with retry loop
print('Connecting...')
while True:
    try:
        vehicle = connect(args.connect)
        break
    except Exception as e:
        print(f"Connection failed: {e}. Retrying in 2s...")
        time.sleep(2)
print(vehicle, "connected!!!")

#--------------------------------------------------
#-------------- LED CONTROLLER & FAILSAFE LISTENER
#--------------------------------------------------
led_controller = DroneLEDController()
led_controller.start()

battery_failsafe_active = False
other_failsafe_active = False

@vehicle.on_message('STATUSTEXT')
def listener(self, name, message):
    global battery_failsafe_active, other_failsafe_active
    text = message.text.upper()
    if "BATTERY" in text and "FAILSAFE" in text:
        battery_failsafe_active = True
        print("[LED] Battery Failsafe Detected!")
    elif "FAILSAFE" in text:
        other_failsafe_active = True
        print(f"[LED] Failsafe Detected: {text}")

# Reset flags on arming?
@vehicle.on_attribute('armed')
def armed_listener(self, attr_name, value):
    global battery_failsafe_active, other_failsafe_active
    if value: # particle armed
        battery_failsafe_active = False
        other_failsafe_active = False
        print("[LED] Armed - Resetting Failsafe Flags")


#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
vehicle.parameters['PLND_ENABLED']       = 1
vehicle.parameters['PLND_TYPE']          = 1 # Mavlink landing backend

# vehicle.parameters['LAND_REPOSITION']   = 0 # !!!!!! ONLY FOR SITL IF NO RC IS CONNECTED


# vehicle.parameters['RNGFND_TYPE']       = 10
# vehicle.parameters['RNGFND_MIN_CM']     = 1
# vehicle.parameters['RNGFND_MAX_CM']     = 10000
# vehicle.parameters['RNGFND_GNDCLEAR']   = 5     

# --- Start Firebase Listener ---
init_firebase_listener()

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 72
marker_size     = 29.7 #- [cm]
freq_send       = 10 #- Hz


#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
# Raspberry Pi Module 3 NoIR Wide camera resolution
# Native: 2304x1296 (16:9, 120° FOV)
# Recommended for processing: 640x360 (preserves 16:9 aspect ratio, faster computation)
camera_resolution   = [640, 360]  # 16:9 aspect ratio for wide FOV
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True, axis_scale=0.01,
                camera_matrix=camera_matrix, camera_distortion=camera_distortion, camera_size=camera_resolution)
                
                
time_0 = time.time()

#--- Check mavlink standard
mavlink20 = 'MAVLINK20' in os.environ

#--------------------------------------------------
#-------------- ROLLING STABILITY BUFFER
#--------------------------------------------------
# Initialize detection stability buffer with maxlen=7
# This prevents jerky movements when marker detection drops temporarily
detection_buffer = deque(maxlen=7)
last_known_position = None  # Store last known valid position (x_cm, y_cm, z_cm)
confidence_threshold =20.0  # Minimum confidence percentage to send position data

print(f"Rolling Stability Buffer initialized (size: 7, threshold: {confidence_threshold}%)")

while True:                
    # Detect marker in current frame
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    
    # Update detection buffer
    if marker_found:
        # Marker detected - append 1 and update last known position
        detection_buffer.append(1)
        x_cm, y_cm = camera_to_uav(x_cm, y_cm)
        z_cm = max(1.0, float(z_cm))
        last_known_position = (x_cm, y_cm, z_cm)
    else:
        # Marker not detected - append 0, keep last known position
        detection_buffer.append(0)
    
    # Calculate confidence score (percentage of successful detections in buffer)
    if len(detection_buffer) > 0:
        confidence_score = (sum(detection_buffer) / len(detection_buffer)) * 100.0
    else:
        confidence_score = 0.0
    
    # Send position data only if confidence exceeds threshold and we have a valid position
    if confidence_score >= confidence_threshold and last_known_position is not None:
        x_cm, y_cm, z_cm = last_known_position
        angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)
        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            status = "DETECTED" if marker_found else "TRACKING"
            print(f"[{status}] Confidence: {confidence_score:.1f}% | x={x_cm:5.0f}cm y={y_cm:5.0f}cm z={z_cm:5.0f}cm | angles=({angle_x:.3f}, {angle_y:.3f})")
            # send_land_message(x_m=x_cm*0.01, y_m=y_cm*0.01, z_m=z_cm*0.01)
            send_land_message_v2(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
    else:
        # Low confidence or no position data - do not send
        # No print statements to avoid log spam
        pass
    
    #--------------------------------------------------
    #-------------- LED CONTROL UPDATE
    #--------------------------------------------------
    # Determine the target LED state based on priorities
    target_led_state = DroneLEDController.STATE_DISARMED # Default
    
    # Check for Failsafes first (Highest Priority)
    if battery_failsafe_active:
        target_led_state = DroneLEDController.STATE_BAT_FAILSAFE
    elif other_failsafe_active:
        target_led_state = DroneLEDController.STATE_FAILSAFE
    
    # Check Flight Modes
    elif vehicle.mode.name == 'LAND':
        target_led_state = DroneLEDController.STATE_LAND
    elif vehicle.mode.name == 'RTL':
        target_led_state = DroneLEDController.STATE_RTL
        
    # Check Armed Status
    elif not vehicle.armed:
        target_led_state = DroneLEDController.STATE_DISARMED
    else:
        # Armed
        if vehicle.location.global_relative_frame.alt < 0.3: # Below 30cm -> Considered on Ground? Or verify logic
            # "ARMED and on the ground"
            # Detecting "on ground" reliably without a sensor can be tricky. 
            # Using altitude < 0.5m as a proxy or if motors are spinning but not taking off.
            # Ideally use vehicle.system_status.state or similar if available, 
            # but altitude is a common simple check.
            target_led_state = DroneLEDController.STATE_ARMED_GROUND
        else:
            target_led_state = DroneLEDController.STATE_FLYING
            
    # Update LED Controller
    led_controller.set_state(target_led_state)
      