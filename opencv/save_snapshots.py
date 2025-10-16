"""
Saves a series of snapshots with the current camera as snapshot_<width>_<height>_<nnn>.jpg
Uses Raspberry Pi camera (picamera) if available, falls back to OpenCV for regular cameras.

Arguments:
    --f <output folder>     default: current folder
    --n <file name>         default: snapshot
    --w <width px>          default: none
    --h <height px>         default: none

Buttons:
    q           - quit
    s           - save the snapshot
    
  
"""

import cv2
import time
import sys
import argparse
import os

# Try to import picamera for Raspberry Pi camera support
try:
    from picamera import PiCamera
    from picamera.array import PiRGBArray
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

__author__ = "Parag"
__date__ = "16/10/2025"


def save_snaps_picamera(width=0, height=0, name="snapshot", folder="."):
    """Save snapshots using Raspberry Pi camera (picamera library)"""
    try:
        # Create folder if it doesn't exist
        if not os.path.exists(folder):
            os.makedirs(folder)
        
        # Initialize PiCamera
        camera = PiCamera()
        
        # Set resolution if specified
        if width > 0 and height > 0:
            camera.resolution = (width, height)
            print(f"Setting camera resolution to {width}x{height}")
        else:
            # Use default resolution
            width, height = camera.resolution
        
        # Start preview
        camera.start_preview()
        time.sleep(2)  # Allow camera to adjust to lighting
        
        nSnap = 0
        fileName = "%s/%s_%d_%d_" % (folder, name, width, height)
        
        print("Using Raspberry Pi Camera (picamera)")
        print("Press 'q' to quit, 's' to save snapshot")
        
        while True:
            # Capture image to memory
            raw_capture = PiRGBArray(camera)
            camera.capture(raw_capture, format="bgr")
            frame = raw_capture.array
            
            # Display the frame
            cv2.imshow('Raspberry Pi Camera', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                print(f"Saving image {nSnap}")
                cv2.imwrite("%s%d.jpg" % (fileName, nSnap), frame)
                nSnap += 1
        
        camera.stop_preview()
        camera.close()
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Error with Raspberry Pi camera: {e}")
        print("Falling back to regular camera mode...")
        save_snaps_opencv(width, height, name, folder)


def save_snaps_opencv(width=0, height=0, name="snapshot", folder=".", raspi=False):
    """Save snapshots using OpenCV (regular camera mode)"""
    if raspi:
        os.system('sudo modprobe bcm2835-v4l2')

    cap = cv2.VideoCapture(0)
    if width > 0 and height > 0:
        print("Setting the custom Width and Height")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    try:
        if not os.path.exists(folder):
            os.makedirs(folder)
    except:
        pass

    nSnap = 0
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    fileName = "%s/%s_%d_%d_" % (folder, name, w, h)
    
    print("Using OpenCV camera")
    print("Press 'q' to quit, 's' to save snapshot")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera")
            break

        cv2.imshow('camera', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print("Saving image ", nSnap)
            cv2.imwrite("%s%d.jpg" % (fileName, nSnap), frame)
            nSnap += 1

    cap.release()
    cv2.destroyAllWindows()


def save_snaps(width=0, height=0, name="snapshot", folder=".", raspi=False):
    """Main function that chooses between picamera and OpenCV"""
    if PICAMERA_AVAILABLE and raspi:
        print("Raspberry Pi camera mode detected, using picamera library...")
        save_snaps_picamera(width, height, name, folder)
    else:
        if not PICAMERA_AVAILABLE:
            print("picamera library not available, using OpenCV...")
        else:
            print("Using OpenCV camera mode...")
        save_snaps_opencv(width, height, name, folder, raspi)




def main():
    # ---- DEFAULT VALUES ---
    SAVE_FOLDER = "."
    FILE_NAME = "snapshot"
    FRAME_WIDTH = 0
    FRAME_HEIGHT = 0

    # ----------- PARSE THE INPUTS -----------------
    parser = argparse.ArgumentParser(
        description="Saves snapshot from the camera. \n q to quit \n s to save the snapshot")
    parser.add_argument("--folder", default=SAVE_FOLDER, help="Path to the save folder (default: current)")
    parser.add_argument("--name", default=FILE_NAME, help="Picture file name (default: snapshot)")
    parser.add_argument("--dwidth", default=FRAME_WIDTH, type=int, help="<width> px (default the camera output)")
    parser.add_argument("--dheight", default=FRAME_HEIGHT, type=int, help="<height> px (default the camera output)")
    parser.add_argument("--raspi", action="store_true", help="Force use of Raspberry Pi camera (picamera)")
    parser.add_argument("--no-raspi", action="store_true", help="Force use of OpenCV camera (disable Pi camera)")
    args = parser.parse_args()

    # Determine camera mode
    use_raspi = False
    if args.no_raspi:
        use_raspi = False
        print("Forcing OpenCV camera mode...")
    elif args.raspi:
        use_raspi = True
        print("Forcing Raspberry Pi camera mode...")
    elif PICAMERA_AVAILABLE:
        # Auto-detect Raspberry Pi camera
        try:
            test_camera = PiCamera()
            test_camera.close()
            use_raspi = True
            print("Raspberry Pi camera detected automatically.")
        except:
            use_raspi = False
            print("Raspberry Pi camera not available, using OpenCV...")
    else:
        use_raspi = False
        print("picamera library not available, using OpenCV...")

    save_snaps(width=args.dwidth, height=args.dheight, name=args.name, folder=args.folder, raspi=use_raspi)

    print("Files saved")

if __name__ == "__main__":
    main()



