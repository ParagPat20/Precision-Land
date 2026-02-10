"""
Visual comparison of detection at different scales

This script shows side-by-side comparison of:
- Original captured image (1920×1080)
- Downscaled image used for detection (e.g., 960×540)
- ArUco detection results on both

Helps visualize the downscaling optimization and verify detection quality.
"""

import numpy as np
import cv2
from picamera2 import Picamera2
import time

def show_scale_comparison(detection_scale=0.5):
    """
    Show visual comparison of original vs downscaled detection
    
    Args:
        detection_scale: Scale factor for detection (0.1 to 1.0)
    """
    
    print("=" * 80)
    print("DOWNSCALING VISUALIZATION")
    print("=" * 80)
    print("")
    print(f"Camera resolution: 1920×1080")
    print(f"Detection scale: {detection_scale}")
    print(f"Detection resolution: {int(1920*detection_scale)}×{int(1080*detection_scale)}")
    print("")
    print("This window shows:")
    print("  LEFT:  Original captured image (1920×1080)")
    print("  RIGHT: Downscaled image for detection")
    print("")
    print("Press 'q' to quit")
    print("=" * 80)
    print("")
    
    # Initialize camera
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(main={"size": (1920, 1080)})
    picam2.configure(cfg)
    picam2.start()
    
    # Enable autofocus
    try:
        picam2.set_controls({"AfMode": 2})
        print("Autofocus enabled")
    except:
        pass
    
    time.sleep(0.5)
    
    # ArUco detector setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    print("Capturing frames... (Press 'q' to quit)")
    
    try:
        while True:
            # Capture frame
            rgb = picam2.capture_array()
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            
            # Original grayscale
            gray_original = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Downscaled grayscale
            detect_width = int(1920 * detection_scale)
            detect_height = int(1080 * detection_scale)
            gray_downscaled = cv2.resize(gray_original, (detect_width, detect_height), 
                                        interpolation=cv2.INTER_AREA)
            
            # Detect on original
            corners_orig, ids_orig, _ = detector.detectMarkers(gray_original)
            
            # Detect on downscaled
            corners_down, ids_down, _ = detector.detectMarkers(gray_downscaled)
            
            # Draw results on original
            frame_orig_display = frame.copy()
            if ids_orig is not None:
                cv2.aruco.drawDetectedMarkers(frame_orig_display, corners_orig, ids_orig)
            
            # Scale up downscaled image for display
            frame_down_display = cv2.resize(gray_downscaled, (1920, 1080), 
                                           interpolation=cv2.INTER_NEAREST)
            frame_down_display = cv2.cvtColor(frame_down_display, cv2.COLOR_GRAY2BGR)
            
            # Scale corners back and draw
            if ids_down is not None and len(corners_down) > 0:
                corners_scaled = [corner * (1.0/detection_scale) for corner in corners_down]
                cv2.aruco.drawDetectedMarkers(frame_down_display, corners_scaled, ids_down)
            
            # Add text labels
            cv2.putText(frame_orig_display, f"ORIGINAL (1920x1080)", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_orig_display, f"Markers: {len(ids_orig) if ids_orig is not None else 0}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.putText(frame_down_display, f"DOWNSCALED ({detect_width}x{detect_height})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame_down_display, f"Markers: {len(ids_down) if ids_down is not None else 0}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame_down_display, f"Scale: {detection_scale}", 
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Resize for display (to fit on screen)
            display_width = 1600
            display_height = 450
            frame_orig_small = cv2.resize(frame_orig_display, (800, 450))
            frame_down_small = cv2.resize(frame_down_display, (800, 450))
            
            # Concatenate side by side
            combined = np.hstack([frame_orig_small, frame_down_small])
            
            # Show
            cv2.imshow('Downscaling Comparison (Press q to quit)', combined)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print("Done!")


if __name__ == "__main__":
    
    print("Testing different detection scales...")
    print("")
    print("Choose a detection scale to visualize:")
    print("  1. Full resolution (1.0)")
    print("  2. Half resolution (0.5) - RECOMMENDED")
    print("  3. Third resolution (0.33)")
    print("  4. Quarter resolution (0.25)")
    print("")
    
    choice = input("Enter choice (1-4) [default: 2]: ").strip()
    
    scales = {
        '1': 1.0,
        '2': 0.5,
        '3': 0.33,
        '4': 0.25
    }
    
    scale = scales.get(choice, 0.5)  # Default to 0.5
    
    print("")
    show_scale_comparison(detection_scale=scale)
