"""
Saves a series of snapshots with the current camera as snapshot_<width>_<height>_<nnn>.jpg
Supports USB cameras (with DirectShow on Windows for zero-lag/anti-wavy streaming),
and Raspberry Pi camera (Picamera2 / libcamera) on Linux.

Features:
- Windows DirectShow backend to prevent MSMF video lag, stutter, and rolling shutter wave distortion.
- Selectable camera device index (--device / -d).
- Live Checkerboard corner detection preview toggle ('c').
- Real-time Undistortion preview toggle ('u') if calibration files are found.
- Live OSD HUD overlay with status, resolution, and snapshot counter.

Arguments:
    --folder <output folder>    default: current folder
    --name <file name>          default: snapshot
    --dwidth <width px>         default: 1280 (or 0 for camera native)
    --dheight <height px>       default: 720 (or 0 for camera native)
    --device <index>            default: 0
    --fps <fps>                 default: 30
    --rows <nRows>              default: 9 (inner corners for checkerboard)
    --cols <nCols>              default: 6 (inner corners for checkerboard)
    --raspi                     Force Raspberry Pi camera mode
    --no-raspi                  Force OpenCV camera mode

Keyboard Controls:
    's' - Save snapshot
    'c' - Toggle live Checkerboard / Chessboard corner detection preview
    'u' - Toggle Undistortion view (requires cameraMatrix and cameraDistortion files)
    'q' / ESC - Quit
"""

import os
if "QT_QPA_PLATFORM" not in os.environ and os.name != "nt":
    os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
import time
import sys
import argparse
import glob

# Try to import Picamera2 for Raspberry Pi camera support
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False

__author__ = "Parag"
__date__ = "16/10/2025 (Updated 28/10/2026)"


def try_load_calibration(folder="."):
    """Attempts to find and load camera matrix and distortion coefficients."""
    search_dirs = [folder, ".", os.path.dirname(__file__)]
    matrix = None
    distortion = None

    for d in search_dirs:
        m_files = glob.glob(os.path.join(d, "*cameraMatrix*.txt")) + glob.glob(os.path.join(d, "*matrix*.txt"))
        d_files = glob.glob(os.path.join(d, "*cameraDistortion*.txt")) + glob.glob(os.path.join(d, "*distortion*.txt"))
        if m_files and d_files:
            try:
                matrix = np.loadtxt(m_files[0])
                distortion = np.loadtxt(d_files[0])
                print(f"[CALIB] Loaded calibration files from {d}:")
                print(f"        Matrix: {os.path.basename(m_files[0])}")
                print(f"        Distortion: {os.path.basename(d_files[0])}")
                break
            except Exception as e:
                print(f"[CALIB] Warning: Failed to parse calibration files in {d}: {e}")

    return matrix, distortion


def save_snaps_picamera2(width=0, height=0, name="snapshot", folder=".", board_shape=(9, 6)):
    """Save snapshots using Raspberry Pi camera (Picamera2 library)"""
    try:
        if not os.path.exists(folder):
            os.makedirs(folder)

        cam = Picamera2()
        if width > 0 and height > 0:
            config = cam.create_preview_configuration(main={"size": (int(width), int(height))})
            cam.configure(config)
            eff_w, eff_h = int(width), int(height)
            print(f"[CAMERA] Setting Picamera2 resolution to {eff_w}x{eff_h}")
        else:
            config = cam.create_preview_configuration()
            cam.configure(config)
            eff_w, eff_h = config["main"]["size"]

        cam.start()

        # Try continuous autofocus if supported (e.g. 64MP AF); fixed-focus IMX500 ignores safely
        try:
            cam.set_controls({"AfMode": 2})
            print("[CAMERA] Autofocus enabled (Continuous mode)")
        except Exception:
            print("[CAMERA] Fixed-focus / manual optics detected; skipping AfMode control.")

        time.sleep(0.5)

        nSnap = 0
        show_corners = False
        show_undistort = False
        matrix, distortion = try_load_calibration(folder)

        fileName = "%s/%s_%d_%d_" % (folder, name, eff_w, eff_h)
        print("Using Raspberry Pi Camera (Picamera2)")
        print("Controls: 's' to save, 'c' toggle checkerboard preview, 'u' toggle undistort, 'q' to quit")

        while True:
            rgb = cam.capture_array()
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            display_frame = frame.copy()

            # Live checkerboard detection preview
            if show_corners:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, board_shape, None)
                if ret:
                    cv2.drawChessboardCorners(display_frame, board_shape, corners, ret)
                    cv2.putText(display_frame, f"BOARD DETECTED ({board_shape[0]}x{board_shape[1]})", (20, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, f"SEARCHING BOARD ({board_shape[0]}x{board_shape[1]})...", (20, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Live undistortion preview
            if show_undistort and matrix is not None and distortion is not None:
                display_frame = cv2.undistort(display_frame, matrix, distortion)
                cv2.putText(display_frame, "UNDISTORTED VIEW", (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

            # HUD OSD
            hud_text = f"Snaps: {nSnap} | {eff_w}x{eff_h} | [S] Save | [C] Board: {'ON' if show_corners else 'OFF'} | [U] Undistort: {'ON' if show_undistort else 'OFF'} | [Q] Quit"
            cv2.rectangle(display_frame, (0, 0), (display_frame.shape[1], 35), (0, 0, 0), -1)
            cv2.putText(display_frame, hud_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow('Raspberry Pi Camera (Picamera2)', display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                save_path = "%s%d.jpg" % (fileName, nSnap)
                cv2.imwrite(save_path, frame)
                print(f"[SAVED] {save_path}")
                nSnap += 1
            elif key == ord('c'):
                show_corners = not show_corners
                print(f"[HUD] Checkerboard corner preview: {'ENABLED' if show_corners else 'DISABLED'}")
            elif key == ord('u'):
                if matrix is None:
                    print("[HUD] No calibration files found to preview undistortion.")
                else:
                    show_undistort = not show_undistort
                    print(f"[HUD] Undistortion view: {'ENABLED' if show_undistort else 'DISABLED'}")

        cam.stop()
        cam.close()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error with Raspberry Pi Picamera2: {e}")
        print("Falling back to OpenCV camera mode...")
        save_snaps_opencv(width=width, height=height, name=name, folder=folder, board_shape=board_shape)


def save_snaps_opencv(device=0, width=0, height=0, fps=30, name="snapshot", folder=".", raspi=False, board_shape=(9, 6)):
    """Save snapshots using OpenCV with anti-lag DirectShow on Windows."""
    if raspi:
        os.system('sudo modprobe bcm2835-v4l2')

    # Parse device argument
    try:
        dev_idx = int(device)
    except (ValueError, TypeError):
        dev_idx = device

    # On Windows, prefer cv2.CAP_DSHOW to eliminate MSMF lag, stutter, and rolling shutter wave distortion
    print(f"[CAMERA] Opening camera device {dev_idx}...")
    if os.name == 'nt' and isinstance(dev_idx, int):
        cap = cv2.VideoCapture(dev_idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("[CAMERA] DirectShow open failed, trying default backend...")
            cap = cv2.VideoCapture(dev_idx)
    else:
        cap = cv2.VideoCapture(dev_idx)

    if not cap.isOpened():
        print(f"[CAMERA ERROR] Could not open camera {dev_idx}. Please verify connection and device index.")
        return

    # Request high-speed MJPG and resolution
    req_w = width if width > 0 else 1280
    req_h = height if height > 0 else 720
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    except Exception:
        pass

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, req_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, req_h)

    if fps > 0:
        try:
            cap.set(cv2.CAP_PROP_FPS, fps)
        except Exception:
            pass

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    eff_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    eff_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    eff_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[CAMERA] Active Stream: {eff_w}x{eff_h} @ {eff_fps:.1f} FPS (Requested: {req_w}x{req_h} @ {fps} FPS)")

    if not os.path.exists(folder):
        try:
            os.makedirs(folder)
        except Exception:
            pass

    nSnap = 0
    show_corners = False
    show_undistort = False
    matrix, distortion = try_load_calibration(folder)

    fileName = "%s/%s_%d_%d_" % (folder, name, eff_w, eff_h)
    print("Controls: 's' to save, 'c' toggle checkerboard preview, 'u' toggle undistort, 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[CAMERA] Failed to read frame from camera.")
            time.sleep(0.1)
            continue

        display_frame = frame.copy()

        # Live checkerboard detection preview
        if show_corners:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_b, corners = cv2.findChessboardCorners(gray, board_shape, None)
            if ret_b:
                cv2.drawChessboardCorners(display_frame, board_shape, corners, ret_b)
                cv2.putText(display_frame, f"BOARD DETECTED ({board_shape[0]}x{board_shape[1]})", (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, f"SEARCHING BOARD ({board_shape[0]}x{board_shape[1]})...", (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Live undistortion preview
        if show_undistort and matrix is not None and distortion is not None:
            display_frame = cv2.undistort(display_frame, matrix, distortion)
            cv2.putText(display_frame, "UNDISTORTED VIEW", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

        # HUD OSD
        hud_text = f"Snaps: {nSnap} | {eff_w}x{eff_h} | [S] Save | [C] Board: {'ON' if show_corners else 'OFF'} | [U] Undistort: {'ON' if show_undistort else 'OFF'} | [Q] Quit"
        cv2.rectangle(display_frame, (0, 0), (display_frame.shape[1], 35), (0, 0, 0), -1)
        cv2.putText(display_frame, hud_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Camera Snapshot Capture', display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('s'):
            save_path = "%s%d.jpg" % (fileName, nSnap)
            cv2.imwrite(save_path, frame)
            print(f"[SAVED] Image #{nSnap} saved to {save_path}")
            nSnap += 1
        elif key == ord('c'):
            show_corners = not show_corners
            print(f"[HUD] Checkerboard corner preview: {'ENABLED' if show_corners else 'DISABLED'}")
        elif key == ord('u'):
            if matrix is None:
                print("[HUD] No calibration files found to preview undistortion.")
            else:
                show_undistort = not show_undistort
                print(f"[HUD] Undistortion view: {'ENABLED' if show_undistort else 'DISABLED'}")

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Saves snapshots from camera with anti-distortion DirectShow, live checkerboard HUD, and undistort preview.")
    parser.add_argument("--folder", default=".", help="Path to save folder (default: current)")
    parser.add_argument("--name", default="snapshot", help="Picture file prefix (default: snapshot)")
    parser.add_argument("--device", "-d", default=0, help="Camera device index (default: 0)")
    parser.add_argument("--dwidth", default=1280, type=int, help="Requested width px (default: 1280, 0 for native)")
    parser.add_argument("--dheight", default=720, type=int, help="Requested height px (default: 720, 0 for native)")
    parser.add_argument("--fps", default=30, type=int, help="Target framerate (default: 30)")
    parser.add_argument("--rows", default=9, type=int, help="Checkerboard inner rows (default: 9)")
    parser.add_argument("--cols", default=6, type=int, help="Checkerboard inner cols (default: 6)")
    parser.add_argument("--raspi", action="store_true", help="Force use of Raspberry Pi camera (Picamera2)")
    parser.add_argument("--no-raspi", action="store_true", help="Force use of OpenCV camera")
    args = parser.parse_args()

    board_shape = (args.rows, args.cols)

    use_raspi = False
    if args.no_raspi:
        use_raspi = False
    elif args.raspi:
        use_raspi = True
    elif PICAMERA2_AVAILABLE and os.name != 'nt':
        try:
            test_cam = Picamera2()
            test_cam.close()
            use_raspi = True
            print("Raspberry Pi camera (Picamera2) detected automatically.")
        except Exception:
            use_raspi = False
            print("Raspberry Pi camera not available, using OpenCV...")
    else:
        use_raspi = False

    if use_raspi:
        save_snaps_picamera2(width=args.dwidth, height=args.dheight, name=args.name, folder=args.folder, board_shape=board_shape)
    else:
        save_snaps_opencv(device=args.device, width=args.dwidth, height=args.dheight, fps=args.fps,
                          name=args.name, folder=args.folder, raspi=args.raspi, board_shape=board_shape)

    print("Snapshot capture session ended.")


if __name__ == "__main__":
    main()
