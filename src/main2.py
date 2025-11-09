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
import cv2

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

def get_distance_metres(location1, location2):
    """Returns the ground distance in metres between two LocationGlobal objects."""
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def detect_white_ground(frame, white_ground_size_cm=75.0):
    """
    Detect white paper ground in the frame using color detection.
    Returns: (found, center_x, center_y, area_cm2)
    - found: boolean if white ground detected
    - center_x, center_y: center of white region in pixels (relative to frame center)
    - area_cm2: estimated area in cm^2
    """
    if frame is None:
        return False, 0, 0, 0
    
    # Convert BGR to HSV for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define white color range in HSV
    # White has low saturation and high value
    lower_white = np.array([0, 0, 200])  # Lower bound for white
    upper_white = np.array([180, 30, 255])  # Upper bound for white
    
    # Create mask for white regions
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return False, 0, 0, 0
    
    # Find the largest contour (assuming it's the white ground)
    largest_contour = max(contours, key=cv2.contourArea)
    area_pixels = cv2.contourArea(largest_contour)
    
    # Filter out very small detections
    min_area_pixels = 1000  # Minimum area threshold
    if area_pixels < min_area_pixels:
        return False, 0, 0, 0
    
    # Calculate center of the contour
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return False, 0, 0, 0
    
    center_x_pixel = int(M["m10"] / M["m00"])
    center_y_pixel = int(M["m01"] / M["m00"])
    
    # Get frame center
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2
    
    # Calculate offset from frame center
    offset_x = center_x_pixel - frame_center_x
    offset_y = center_y_pixel - frame_center_y
    
    # Estimate area in cm^2 (rough approximation)
    # This is a simplified calculation - actual area depends on altitude and camera FOV
    # For a 750mm x 750mm ground (75cm x 75cm = 5625 cm^2)
    area_cm2 = area_pixels * (white_ground_size_cm * white_ground_size_cm) / (frame.shape[0] * frame.shape[1]) * 10  # Rough scaling
    
    return True, offset_x, offset_y, area_cm2

# Global camera capture for white detection (fallback if tracker's camera is not accessible)
_white_detection_cap = None

def get_camera_frame(aruco_tracker):
    """
    Get the current camera frame from the aruco tracker.
    Returns the frame or None if unavailable.
    """
    global _white_detection_cap
    
    # Try to get frame from aruco tracker's camera
    try:
        # For Picamera2
        if hasattr(aruco_tracker, '_use_picamera') and aruco_tracker._use_picamera:
            if hasattr(aruco_tracker, '_picam2') and aruco_tracker._picam2 is not None:
                rgb = aruco_tracker._picam2.capture_array()
                frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                return frame
        # For OpenCV VideoCapture - try to read from the same capture
        elif hasattr(aruco_tracker, '_cap') and aruco_tracker._cap is not None:
            # Read a fresh frame from the tracker's capture
            # Note: This reads a new frame, which is what we want for white detection
            ret, frame = aruco_tracker._cap.read()
            if ret:
                return frame
    except Exception as e:
        # If accessing tracker's camera fails, use fallback
        pass
    
    # Fallback: create a separate camera capture for white detection
    # This is used if we can't access the tracker's camera directly
    try:
        if _white_detection_cap is None:
            # Try to use the same camera index (0)
            _white_detection_cap = cv2.VideoCapture(0)
            if _white_detection_cap.isOpened():
                # Set same resolution as aruco tracker (typically 640x480)
                _white_detection_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                _white_detection_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if _white_detection_cap is not None and _white_detection_cap.isOpened():
            ret, frame = _white_detection_cap.read()
            if ret:
                return frame
    except Exception as e:
        print(f"Error getting camera frame (fallback): {e}")
    
    return None
        
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
white_ground_size_cm = 75.0  # 750mm = 75cm

# White ground detection parameters
ALT_MIN_WHITE_DETECTION = 4.0  # meters - minimum altitude for white detection
ALT_MAX_WHITE_DETECTION = 14.0  # meters - maximum altitude for white detection
HOME_PROXIMITY_THRESHOLD = 20.0  # meters - distance to home to start white detection

#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True, axis_scale=0.01,
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
                
time_0 = time.time()
time_white_0 = time.time()

#--- Check mavlink standard
mavlink20 = 'MAVLINK20' in os.environ

# Wait for home location to be set
print("Waiting for home location...")
while vehicle.home_location is None or vehicle.home_location.lat == 0:
    time.sleep(0.5)
print(f"Home location set: {vehicle.home_location}")

print("Starting main loop...")

while True:
    # Get current vehicle state
    current_mode = vehicle.mode.name
    current_alt = vehicle.location.global_relative_frame.alt
    current_location = vehicle.location.global_relative_frame
    
    # Check if home location is available
    home_location = vehicle.home_location
    if home_location is None:
        home_location = current_location
    
    # Calculate distance to home
    distance_to_home = get_distance_metres(current_location, home_location)
    
    # Determine if we should use white ground detection
    use_white_detection = False
    if (current_mode == 'RTL' and 
        ALT_MIN_WHITE_DETECTION < current_alt < ALT_MAX_WHITE_DETECTION and
        distance_to_home < HOME_PROXIMITY_THRESHOLD):
        use_white_detection = True
    
    if use_white_detection:
        # Use white ground detection when in RTL mode, between 4-14m altitude, and near home
        frame = get_camera_frame(aruco_tracker)
        white_found, offset_x, offset_y, area_cm2 = detect_white_ground(frame, white_ground_size_cm)
        
        if white_found:
            # Convert pixel offset to angular offset (rough approximation)
            # Assuming typical camera FOV of ~60 degrees
            fov_deg = 60.0
            fov_rad = math.radians(fov_deg)
            frame_width = frame.shape[1] if frame is not None else 640
            
            # Calculate angular offsets
            angle_x_white = (offset_x / frame_width) * fov_rad
            angle_y_white = (offset_y / frame_width) * fov_rad  # Assuming square pixels
            
            # Estimate distance based on area (rough approximation)
            # Larger area = closer to ground
            # For 750mm x 750mm ground, at different altitudes
            # This is a simplified model
            if area_cm2 > 0:
                # Rough distance estimation based on area
                # At 4m: area should be larger, at 14m: area should be smaller
                estimated_dist_m = max(4.0, min(14.0, 10.0 / (area_cm2 / 1000.0)))  # Rough scaling
            else:
                estimated_dist_m = current_alt  # Use current altitude as fallback
            
            if time.time() >= time_white_0 + 1.0/freq_send:
                time_white_0 = time.time()
                print(f"White ground found: offset_x={offset_x:.1f}px, offset_y={offset_y:.1f}px, "
                      f"area={area_cm2:.1f}cm², alt={current_alt:.1f}m, dist_home={distance_to_home:.1f}m")
                print(f"  -> angle_x={math.degrees(angle_x_white):.3f}°, angle_y={math.degrees(angle_y_white):.3f}°")
                
                # Send landing target message to keep white ground centered
                send_land_message_v2(x_rad=angle_x_white, y_rad=angle_y_white, 
                                    dist_m=estimated_dist_m, time_usec=time.time()*1e6)
        else:
            if time.time() >= time_white_0 + 1.0/freq_send:
                time_white_0 = time.time()
                print(f"White ground not found - searching... (alt={current_alt:.1f}m, dist_home={distance_to_home:.1f}m)")
    else:
        # Use ArUco marker detection (normal operation or when below 4m)
        marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
        if marker_found:
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
            # Log when in RTL but conditions not met for white detection
            if current_mode == 'RTL' and time.time() >= time_0 + 2.0:  # Log every 2 seconds
                time_0 = time.time()
                print(f"RTL mode: alt={current_alt:.1f}m, dist_home={distance_to_home:.1f}m, "
                      f"white_detection={'ON' if use_white_detection else 'OFF'}")
      