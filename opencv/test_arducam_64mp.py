"""
Test script for Arducam 64MP OV64A40 camera with ArUco marker tracking

This script demonstrates how to use the updated lib_aruco_pose.py with the Arducam 64MP camera.

Prerequisites:
1. Raspberry Pi OS Bookworm or Trixie
2. Camera configured in /boot/firmware/config.txt:
   - Set camera_auto_detect=0
   - Add: dtoverlay=ov64a40,link-frequency=360000000
   - Reboot after changes

3. Install required packages:
   sudo apt install -y python3-picamera2 python3-opencv python3-numpy

Features enabled:
- 1920×1080 resolution (lowest available, 45.65 fps)
- Continuous autofocus mode
- ArUco marker detection and tracking
- Image downscaling optimization for faster detection (2-4x speedup)

Performance Optimization:
- Camera captures at 1920×1080 (full resolution)
- ArUco detection runs on 960×540 (50% scale, default)
- Corners scaled back to full resolution for accurate pose estimation
- This provides 2-4x faster detection with minimal accuracy loss

Available camera modes (from rpicam-still --list-cameras):
- 1920×1080 @ 45.65 fps (lowest resolution - used here)
- 2312×1736 @ 26.75 fps
- 3840×2160 @ 14.89 fps
- 4624×3472 @ 7.66 fps
- 8000×6000 @ 2.58 fps
- 9248×6944 @ 2.02 fps (maximum resolution)
"""

import numpy as np
from lib_aruco_pose import ArucoSingleTracker

if __name__ == "__main__":
    
    # ArUco marker configuration
    id_to_find = 72      # ArUco marker ID to track
    marker_size = 10     # Physical size of the marker in cm
    
    # Camera calibration matrices
    # NOTE: Update these paths to match your camera calibration files
    # You should calibrate your camera using the 1280×720 resolution
    calib_path = ""
    
    try:
        camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
        camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')
    except FileNotFoundError:
        print("WARNING: Camera calibration files not found!")
        print("Using default calibration values - results may be inaccurate")
        print("Please calibrate your Arducam 64MP camera at 1920×1080 resolution")
        
        # Default calibration for Arducam 64MP at 1920×1080 (approximate)
        # These values should be replaced with actual calibration data
        camera_matrix = np.array([
            [1400.0, 0.0, 960.0],    # fx, 0, cx (center x = 1920/2)
            [0.0, 1400.0, 540.0],    # 0, fy, cy (center y = 1080/2)
            [0.0, 0.0, 1.0]
        ])
        camera_distortion = np.zeros((1, 5))
    
    # Initialize ArUco tracker with Arducam 64MP settings
    print("Initializing Arducam 64MP camera...")
    print("Resolution: 1920×1080 (lowest available)")
    print("Frame Rate: 45.65 fps")
    print("Autofocus: Continuous mode enabled")
    print("")
    print("Performance Optimization:")
    print("- Detection scale: 0.5 (960×540 for detection)")
    print("- Expected speedup: 2-4x faster")
    print("- Pose estimation: Full resolution (1920×1080)")
    print("")
    
    # Detection scale options:
    # 1.0 = Full resolution detection (slowest, most accurate)
    # 0.5 = Half resolution (960×540) - RECOMMENDED (2-4x faster)
    # 0.33 = Third resolution (640×360) - fastest, may miss small markers
    # 0.25 = Quarter resolution (480×270) - very fast, only large markers
    
    aruco_tracker = ArucoSingleTracker(
        id_to_find=id_to_find,
        marker_size=marker_size,
        camera_matrix=camera_matrix,
        camera_distortion=camera_distortion,
        camera_size=[1920, 1080],  # Arducam 64MP lowest resolution
        show_video=True,           # Display video window
        use_picamera=None,         # Auto-detect Picamera2
        detection_scale=0.5        # 50% scale for detection (2-4x speedup)
    )
    
    print("Starting marker tracking...")
    print("Press 'q' to quit")
    print("")
    
    # Start tracking with verbose output
    try:
        aruco_tracker.track(verbose=True, show_video=True)
    except KeyboardInterrupt:
        print("\nTracking stopped by user")
    finally:
        aruco_tracker.stop()
        print("Camera stopped successfully")
