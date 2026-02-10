"""
Saves a series of snapshots with the current camera as snapshot_<width>_<height>_<nnn>.jpg
Uses Raspberry Pi camera (Picamera2) if available, falls back to OpenCV for regular cameras.

Configured for Arducam 64MP OV64A40 Camera:
- Default resolution: 1920×1080 (lowest available mode, 45.65 fps)
- Autofocus: Continuous mode enabled automatically
- Use these snapshots for camera calibration at 1920×1080

Arguments:
    --folder <output folder>    default: current folder
    --name <file name>          default: snapshot
    --dwidth <width px>         default: 1920
    --dheight <height px>       default: 1080
    --raspi                     Force Raspberry Pi camera mode
    --no-raspi                  Force OpenCV camera mode

Buttons:
    q           - quit
    s           - save the snapshot
    
  
"""

import cv2
import time
import sys
import argparse
import os

# Try to import Picamera2 for Raspberry Pi camera support
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False

__author__ = "Parag"
__date__ = "16/10/2025"


def save_snaps_picamera2(width=0, height=0, name="snapshot", folder="."):
    """Save snapshots using Raspberry Pi camera (Picamera2 library)"""
    try:
        # Create folder if it doesn't exist
        if not os.path.exists(folder):
            os.makedirs(folder)

        cam = Picamera2()
        # Configure resolution
        if width > 0 and height > 0:
            config = cam.create_preview_configuration(main={"size": (int(width), int(height))})
            cam.configure(config)
            eff_w, eff_h = int(width), int(height)
            print(f"Setting camera resolution to {eff_w}x{eff_h}")
        else:
            config = cam.create_preview_configuration()
            cam.configure(config)
            eff_w, eff_h = config["main"]["size"]

        cam.start()
        
        # Enable continuous autofocus for Arducam 64MP
        try:
            cam.set_controls({"AfMode": 2})  # Continuous autofocus
            print("Arducam 64MP: Continuous autofocus enabled")
        except Exception as e:
            print(f"Note: Could not enable autofocus - {e}")
        
        time.sleep(0.5)  # brief warmup

        nSnap = 0
        fileName = "%s/%s_%d_%d_" % (folder, name, eff_w, eff_h)

        print("Using Raspberry Pi Camera (Picamera2)")
        print("Press 'q' to quit, 's' to save snapshot")

        while True:
            # capture_array returns RGB; convert to BGR for OpenCV
            rgb = cam.capture_array()
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            cv2.imshow('Raspberry Pi Camera', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                print(f"Saving image {nSnap}")
                cv2.imwrite("%s%d.jpg" % (fileName, nSnap), frame)
                nSnap += 1

        cam.stop()
        cam.close()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error with Raspberry Pi Picamera2: {e}")
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
    """Main function that chooses between Picamera2 and OpenCV"""
    if PICAMERA2_AVAILABLE and raspi:
        print("Raspberry Pi camera mode detected, using Picamera2 library...")
        save_snaps_picamera2(width, height, name, folder)
    else:
        if not PICAMERA2_AVAILABLE:
            print("Picamera2 library not available, using OpenCV...")
        else:
            print("Using OpenCV camera mode...")
        save_snaps_opencv(width, height, name, folder, raspi)


def main():
    # ---- DEFAULT VALUES ---
    SAVE_FOLDER = "."
    FILE_NAME = "snapshot"
    # Arducam 64MP OV64A40 Camera
    # Maximum resolution: 9248×6944 (64MP)
    # Lowest resolution: 1920×1080 @ 45.65 fps
    # Recommended for calibration: 1920×1080 (native mode, good balance)
    # Set to 0 to use camera default
    FRAME_WIDTH = 1920   # Arducam 64MP lowest resolution
    FRAME_HEIGHT = 1080  # 16:9 aspect ratio

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
    elif PICAMERA2_AVAILABLE:
        # Auto-detect Raspberry Pi camera via Picamera2 init
        try:
            test_cam = Picamera2()
            test_cam.close()
            use_raspi = True
            print("Raspberry Pi camera (Picamera2) detected automatically.")
        except:
            use_raspi = False
            print("Raspberry Pi camera not available, using OpenCV...")
    else:
        use_raspi = False
        print("Picamera2 library not available, using OpenCV...")

    save_snaps(width=args.dwidth, height=args.dheight, name=args.name, folder=args.folder, raspi=use_raspi)

    print("Files saved")

if __name__ == "__main__":
    main()



