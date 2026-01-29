"""

NOTE: Call it with Mavlink 2.0 as follows:
MAVLINK20=1 python test_.....py

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

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
def execute_mission_logic(mission_items):
    print("Uploading mission via DroneKit...")
    try:
        cmds = vehicle.commands
        cmds.clear()
        
        for item in mission_items:
            # Create DroneKit Command
            # Note: DroneKit uses floats for lat/lon, so we use item.x/y directly (assuming valid floats)
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
        
        print("Setting Mode to AUTO...")
        vehicle.mode = VehicleMode("AUTO")
        
        print("Arming...")
        vehicle.armed = True
        
        # We don't block heavily here waiting for arming in the thread, 
        # or we could wait a bit. The main loop monitors state too.
        # But for valid dispatch flow, we should ensure it proceeds.
        # Wait up to 5s for arming
        for _ in range(5):
            if vehicle.armed:
                break
            time.sleep(1)
            
        print(f"ARMED Status: {vehicle.armed}")
        return True
    except Exception as e:
        print(f"Mission Execution Error: {e}")
        return False

def run_mission_thread(command):
    cmd_id = command.get('id')
    print(f"[{threading.current_thread().name}] Processing Mission {cmd_id}")
    payload = command.get('payload', {})
    
    target_lat = payload.get('target_lat')
    target_lng = payload.get('target_lng')
    
    if target_lat is None or target_lng is None:
        print("Invalid Target Location")
        return

    # Generate Mission
    print("Generating Mission...")
    template = DeliveryTemplate.get_default_template()
    # TODO: Get actual home location
    home_loc = LatLng(0.0, 0.0) 
    target_loc = LatLng(target_lat, target_lng)
    
    mission_items = template.generate_mission(
        home_location=home_loc,
        delivery_location=target_loc,
        override_values=payload
    )
    
    success = execute_mission_logic(mission_items)

    ref = db.reference(f'missions/{DRONE_ID}/active_command')
    if success:
        ref.update({'status': 'IN_PROGRESS', 'started_at': int(time.time() * 1000)})
    else:
        ref.update({'status': 'FAILED'})

def check_mission(command):
    if not command or not isinstance(command, dict): return
    if command.get('status') == 'PENDING':
        sender = command.get('sender_email', 'Unknown')
        print(f"Received PENDING Mission from {sender}")
        t = threading.Thread(target=run_mission_thread, args=(command,), name="MissionThread")
        t.start()

def init_firebase_listener():
    try:
        cred = credentials.Certificate(CREDENTIALS_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        print("Firebase Initialized")
        
        ref = db.reference(f'missions/{DRONE_ID}/active_command')
        # Initial check
        check_mission(ref.get())
        # Listen
        ref.listen(lambda event: check_mission(event.data))
        print("Firebase Listener Started")
    except Exception as e:
        print(f"Firebase Init Error (Check credentials path): {e}")



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
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            if last_known_position is None:
                print(f"[NO DATA] Confidence: {confidence_score:.1f}% | Waiting for initial marker detection...")
            else:
                print(f"[LOW CONFIDENCE] Confidence: {confidence_score:.1f}% | Not sending (threshold: {confidence_threshold}%)")
    
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
      