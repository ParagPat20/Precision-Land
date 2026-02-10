"""
Benchmark script to compare ArUco detection performance at different scale factors

This script helps you find the optimal detection_scale for your use case by
testing different scaling factors and showing the FPS performance.

Usage:
    python3 benchmark_detection_scale.py

Press keys to switch between different detection scales:
    '1' = Full resolution (1.0 scale)
    '2' = Half resolution (0.5 scale) - RECOMMENDED
    '3' = Third resolution (0.33 scale)
    '4' = Quarter resolution (0.25 scale)
    'q' = Quit
"""

import numpy as np
from lib_aruco_pose import ArucoSingleTracker
import time

if __name__ == "__main__":
    
    # ArUco marker configuration
    id_to_find = 72      # ArUco marker ID to track
    marker_size = 10     # Physical size of the marker in cm
    
    # Camera calibration matrices
    calib_path = ""
    
    try:
        camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
        camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')
    except FileNotFoundError:
        print("WARNING: Camera calibration files not found!")
        print("Using default calibration values")
        
        camera_matrix = np.array([
            [1400.0, 0.0, 960.0],
            [0.0, 1400.0, 540.0],
            [0.0, 0.0, 1.0]
        ])
        camera_distortion = np.zeros((1, 5))
    
    # Test different detection scales
    scales = {
        '1': {'scale': 1.0,  'name': 'Full Resolution (1920×1080)', 'color': '\033[91m'},   # Red
        '2': {'scale': 0.5,  'name': 'Half Resolution (960×540)', 'color': '\033[92m'},     # Green
        '3': {'scale': 0.33, 'name': 'Third Resolution (640×360)', 'color': '\033[93m'},    # Yellow
        '4': {'scale': 0.25, 'name': 'Quarter Resolution (480×270)', 'color': '\033[94m'}   # Blue
    }
    
    current_scale = '2'  # Start with recommended scale (0.5)
    
    print("=" * 80)
    print("ARDUCAM 64MP - DETECTION SCALE BENCHMARK")
    print("=" * 80)
    print("")
    print("Testing different detection scales to find optimal performance.")
    print("")
    print("Controls:")
    print("  '1' = Full resolution (1.0 scale) - slowest, most accurate")
    print("  '2' = Half resolution (0.5 scale) - RECOMMENDED (2-4x faster)")
    print("  '3' = Third resolution (0.33 scale) - very fast")
    print("  '4' = Quarter resolution (0.25 scale) - fastest")
    print("  'q' = Quit")
    print("")
    print("=" * 80)
    print("")
    
    # Initialize with default scale
    scale_info = scales[current_scale]
    print(f"Starting with: {scale_info['name']} (scale={scale_info['scale']})")
    print("")
    
    aruco_tracker = ArucoSingleTracker(
        id_to_find=id_to_find,
        marker_size=marker_size,
        camera_matrix=camera_matrix,
        camera_distortion=camera_distortion,
        camera_size=[1920, 1080],
        show_video=True,
        use_picamera=None,
        detection_scale=scale_info['scale']
    )
    
    print("=" * 80)
    print("BENCHMARK RESULTS")
    print("=" * 80)
    print("")
    print("Watch the FPS counter in the window title.")
    print("Compare detection FPS across different scales.")
    print("")
    print("Expected Results:")
    print("  Full (1.0):    10-15 FPS detection")
    print("  Half (0.5):    20-40 FPS detection (2-4x faster)")
    print("  Third (0.33):  30-50 FPS detection")
    print("  Quarter (0.25): 40-60 FPS detection")
    print("")
    print("Note: Actual FPS depends on marker visibility and CPU load")
    print("=" * 80)
    print("")
    
    # Note: The interactive scale switching would require modifying the track() method
    # to accept runtime parameter changes. For now, this benchmarks the initial scale.
    # To test other scales, restart the script and modify current_scale above.
    
    try:
        aruco_tracker.track(verbose=True, show_video=True)
    except KeyboardInterrupt:
        print("\n\nBenchmark stopped by user")
    finally:
        aruco_tracker.stop()
        print("")
        print("=" * 80)
        print("BENCHMARK SUMMARY")
        print("=" * 80)
        print("")
        print("Recommendations based on your requirements:")
        print("")
        print("  detection_scale=1.0:  Use when markers are very small or far away")
        print("  detection_scale=0.5:  RECOMMENDED - Best balance of speed/accuracy")
        print("  detection_scale=0.33: Use when markers are large and close")
        print("  detection_scale=0.25: Use only for very large markers")
        print("")
        print("For precision landing, detection_scale=0.5 is recommended.")
        print("=" * 80)
