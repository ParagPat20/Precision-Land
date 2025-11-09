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
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

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
    print(msg)
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

def set_velocity_body(vehicle, vx, vy, vz):
    """
    Send velocity command in body frame.
    Remember: vz is positive downward!!!
    vx: forward/backward (positive = forward)
    vy: left/right (positive = right)
    vz: up/down (positive = down)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def stop_movement(vehicle):
    """Stop all movement by sending zero velocity."""
    set_velocity_body(vehicle, 0, 0, 0)

def star_pattern_search(vehicle, aruco_tracker, search_speed=1.0, search_distance=2.0, search_duration=3.0):
    """
    Execute a star pattern search (*) in 6 directions to find the ArUco marker.
    
    Pattern:
        Forward
        Backward
        Left
        Right
        Forward-Left (diagonal)
        Forward-Right (diagonal)
    
    Returns: True if marker found, False otherwise
    """
    print("Starting star pattern search for ArUco marker...")
    
    # Define 6 directions for star pattern (*)
    # Each direction: (vx, vy) in body frame
    # vx: forward/backward (positive = forward)
    # vy: left/right (positive = right)
    directions = [
        (search_speed, 0),      # Forward (North)
        (-search_speed, 0),     # Backward (South)
        (0, -search_speed),     # Left (West)
        (0, search_speed),      # Right (East)
        (search_speed * 0.707, -search_speed * 0.707),  # Forward-Left (North-West, 45 deg)
        (search_speed * 0.707, search_speed * 0.707),  # Forward-Right (North-East, 45 deg)
    ]
    
    direction_names = [
        "Forward",
        "Backward", 
        "Left",
        "Right",
        "Forward-Left",
        "Forward-Right"
    ]
    
    # Store starting position
    start_location = vehicle.location.global_relative_frame
    
    # Try each direction
    for i, (vx, vy) in enumerate(directions):
        print(f"Searching direction {i+1}/6: {direction_names[i]} (vx={vx:.2f}, vy={vy:.2f})")
        
        # Move in this direction
        set_velocity_body(vehicle, vx, vy, 0)
        
        # Search while moving
        search_start_time = time.time()
        check_interval = 0.1  # Check for marker every 100ms
        
        while time.time() - search_start_time < search_duration:
            # Check for marker
            marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
            
            if marker_found:
                print(f"Marker FOUND while searching {direction_names[i]}!")
                stop_movement(vehicle)
                return True
            
            time.sleep(check_interval)
        
        # Stop movement before changing direction
        stop_movement(vehicle)
        time.sleep(0.5)  # Brief pause between directions
    
    # Return to starting position (optional - can be commented out if not needed)
    print("Star pattern search completed - marker not found. Returning to start position...")
    stop_movement(vehicle)
    
    return False
        
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
#-------------- PARAMETERS  
#-------------------------------------------------- 
vehicle.parameters['PLND_ENABLED']       = 1
vehicle.parameters['PLND_TYPE']          = 1 # Mavlink landing backend

# vehicle.parameters['LAND_REPOSITION']   = 0 # !!!!!! ONLY FOR SITL IF NO RC IS CONNECTED


# vehicle.parameters['RNGFND_TYPE']       = 10
# vehicle.parameters['RNGFND_MIN_CM']     = 1
# vehicle.parameters['RNGFND_MAX_CM']     = 10000
# vehicle.parameters['RNGFND_GNDCLEAR']   = 5     

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 72
marker_size     = 10 #- [cm]
freq_send       = 10 #- Hz

# Star pattern search parameters
SEARCH_SPEED = 1.0  # m/s - speed for search movements
SEARCH_DURATION = 3.0  # seconds - how long to search in each direction
MARKER_NOT_FOUND_TIMEOUT = 5.0  # seconds - how long marker must be missing before triggering search
SEARCH_COOLDOWN = 30.0  # seconds - wait time before allowing another search

#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True, axis_scale=0.01,
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
                
time_0 = time.time()
time_log_0 = time.time()
last_marker_found_time = time.time()
last_search_time = 0.0
search_in_progress = False

#--- Check mavlink standard
mavlink20 = 'MAVLINK20' in os.environ

print("Starting main loop...")

while True:
    # Get current vehicle state
    current_mode = vehicle.mode.name
    current_alt = vehicle.location.global_relative_frame.alt
    
    # Check for marker
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    
    if marker_found:
        # Marker found - update timestamp and process
        last_marker_found_time = time.time()
        search_in_progress = False  # Reset search flag
        
        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        # ensure positive distance in cm
        z_cm                = max(1.0, float(z_cm))
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            print("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x, angle_y))
            # send_land_message(x_m=x_cm*0.01, y_m=y_cm*0.01, z_m=z_cm*0.01)
            send_land_message_v2(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
    else:
        # Marker not found
        time_since_last_marker = time.time() - last_marker_found_time
        time_since_last_search = time.time() - last_search_time
        
        # Check if we should trigger star pattern search
        # Conditions:
        # 1. In RTL mode
        # 2. Marker not found for longer than timeout
        # 3. Not currently searching
        # 4. Enough time has passed since last search (cooldown)
        if (current_mode == 'RTL' and 
            not search_in_progress and
            time_since_last_marker > MARKER_NOT_FOUND_TIMEOUT and
            time_since_last_search > SEARCH_COOLDOWN):
            
            print(f"Marker not found for {time_since_last_marker:.1f}s in RTL mode. Starting star pattern search...")
            print(f"Current altitude: {current_alt:.1f}m")
            
            # Ensure we're in GUIDED mode for velocity control
            if vehicle.mode.name != 'GUIDED':
                print("Switching to GUIDED mode for search...")
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)  # Wait for mode change
            
            search_in_progress = True
            last_search_time = time.time()
            
            # Execute star pattern search
            marker_found_during_search = star_pattern_search(
                vehicle, 
                aruco_tracker, 
                search_speed=SEARCH_SPEED,
                search_duration=SEARCH_DURATION
            )
            
            if marker_found_during_search:
                print("Marker found during search! Resuming normal operation.")
                last_marker_found_time = time.time()
            else:
                print("Marker not found after star pattern search. Resuming RTL...")
                # Switch back to RTL mode
                vehicle.mode = VehicleMode("RTL")
                time.sleep(1)
            
            search_in_progress = False
        else:
            # Log status periodically when marker not found
            if time.time() >= time_log_0 + 2.0:  # Log every 2 seconds
                time_log_0 = time.time()
                if current_mode == 'RTL':
                    print(f"Marker not found - Mode: {current_mode}, Alt: {current_alt:.1f}m, "
                          f"Time since last marker: {time_since_last_marker:.1f}s, "
                          f"Search in progress: {search_in_progress}")
    
    time.sleep(0.1)  # Small delay to prevent excessive CPU usage
      