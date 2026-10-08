"""
This demo calculates multiple things for different scenarios.

IF RUNNING ON A PI, BE SURE TO sudo modprobe bcm2835-v4l2

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import sys
import shutil
import glob

# Suppress Qt font directory warnings on Linux by dynamically creating the expected font folder if missing
try:
    import cv2
    cv2_dir = os.path.dirname(cv2.__file__)
    qt_fonts_dir = os.path.join(cv2_dir, "qt", "fonts")
    if not os.path.exists(qt_fonts_dir):
        os.makedirs(qt_fonts_dir, exist_ok=True)
        # Copy a standard system TrueType font to the directory
        system_font_paths = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        ]
        font_copied = False
        for font_path in system_font_paths:
            if os.path.exists(font_path):
                shutil.copy(font_path, os.path.join(qt_fonts_dir, os.path.basename(font_path)))
                font_copied = True
                break
        if not font_copied:
            found_fonts = glob.glob("/usr/share/fonts/**/*.ttf", recursive=True)
            if found_fonts:
                shutil.copy(found_fonts[0], os.path.join(qt_fonts_dir, os.path.basename(found_fonts[0])))
except Exception:
    pass

import numpy as np
import cv2.aruco as aruco
import time, math
import threading

# Optional Raspberry Pi camera support via Picamera2, with graceful fallback
try:
    from picamera2 import Picamera2
    _PICAMERA2_AVAILABLE = True
except Exception:
    _PICAMERA2_AVAILABLE = False

class ArucoSingleTracker():
    def __init__(self,
                id_to_find,
                marker_size,
                camera_matrix,
                camera_distortion,
                camera_size=[1280,720],  # Default: 1280x720 for OV9281 full resolution (120° FOV mono), or 640x360 for 64MP
                show_video=False,
                axis_scale=0.03,
                use_picamera=None,
                calib_size=[640, 480],   # Calibration resolution (matches cameraMatrix_webcam.txt)
                target_fps=60,
                focus_mode=None,         # "auto", "continuous", "manual", "infinity"
                lens_position=None,      # Dioptres for manual focus (0.0=infinity, 0.5=2m, 1.0=1m)
                vflip=None,              # Vertical flip for rotated camera mounting (default: True)
                hflip=None               # Horizontal flip (default: False, or True for 180° rotation)
                ):
        
        
        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._show_video    = show_video
        self._axis_scale    = axis_scale
        self.target_fps     = target_fps

        # Vertical and Horizontal flip for rotated / inverted camera mounting
        if vflip is not None:
            self._vflip = bool(vflip)
        else:
            self._vflip = os.environ.get("JECH_VFLIP", "1").lower() in ("1", "true", "yes")

        if hflip is not None:
            self._hflip = bool(hflip)
        else:
            self._hflip = os.environ.get("JECH_HFLIP", "0").lower() in ("1", "true", "yes")

        # Focus configuration for 64MP Arducam (OV64A40)
        self.focus_mode = str(focus_mode or os.environ.get("JECH_FOCUS_MODE", "auto")).lower()
        try:
            self.lens_position = float(lens_position if lens_position is not None else os.environ.get("JECH_LENS_POSITION", "0.5"))
        except (ValueError, TypeError):
            self.lens_position = 0.5
        
        # Scale camera matrix if requested camera size differs from calibration resolution
        self._camera_matrix = np.array(camera_matrix, dtype=np.float32)
        self._camera_distortion = np.array(camera_distortion, dtype=np.float32)
        
        calib_w, calib_h = calib_size[0], calib_size[1]
        w_cur, h_cur = camera_size[0], camera_size[1]
        sx = float(w_cur) / float(calib_w)
        sy = float(h_cur) / float(calib_h)
        if abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3:
            self._camera_matrix[0,0] *= sx  # fx
            self._camera_matrix[1,1] *= sy  # fy
            self._camera_matrix[0,2] *= sx  # cx
            self._camera_matrix[1,2] *= sy  # cy
            print(f"[TRACKER] Scaled camera matrix from calibration size {calib_w}x{calib_h} to active camera size {w_cur}x{h_cur} (sx={sx:.3f}, sy={sy:.3f})")
        
        self.is_detected    = False
        self._kill          = False

        # Latest captured frame (BGR) for external consumers (e.g., video recording).
        # This avoids re-opening the camera from another thread/process.
        self._frame_lock = threading.Lock()
        self.last_frame = None
        self.last_frame_ts = 0.0
        
        #--- 180 deg rotation matrix around the x axis
        self._R_flip      = np.zeros((3,3), dtype=np.float32)
        self._R_flip[0,0] = 1.0
        self._R_flip[1,1] =-1.0
        self._R_flip[2,2] =-1.0

        #--- Define the aruco dictionary
        self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        # Create detector parameters compatible with both OpenCV APIs
        self._use_new_api = False
        try:
            # OpenCV >= 4.7
            self._parameters = aruco.DetectorParameters()
            try:
                self._parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            except Exception:
                pass
            self._detector = aruco.ArucoDetector(self._aruco_dict, self._parameters)
            self._use_new_api = True
        except Exception:
            # Legacy API (OpenCV 3.x/4.6 and below)
            self._parameters  = aruco.DetectorParameters_create()
            try:
                self._parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            except Exception:
                pass
            self._detector = None

        # Decide capture backend (Picamera2 vs OpenCV USB Camera) with graceful fallback
        # use_picamera: True forces Picamera2 (64MP), False forces OpenCV (OV9281 USB), None auto-detect
        self._use_picamera = False
        if (use_picamera is True or use_picamera is None) and _PICAMERA2_AVAILABLE:
            # Enable Picamera2 if requested or available (64MP Camera with AF)
            try:
                self._picam2 = Picamera2()
                # Configure camera with specified resolution in native BGR format for OpenCV
                cfg = self._picam2.create_preview_configuration(main={"size": (int(camera_size[0]), int(camera_size[1])), "format": "BGR888"})
                self._picam2.configure(cfg)
                self._picam2.start()
                
                # Configure Focus for 64MP Camera (OV64A40)
                # 64MP has a Voice Coil Motor (VCM) for autofocus.
                try:
                    if self.focus_mode in ("manual", "fixed"):
                        # Manual Focus: dioptres = 1 / distance_in_meters
                        # e.g., 0.5 dioptres = 2.0 meters, giving wide depth of field from 0.8m to infinity
                        self._picam2.set_controls({"AfMode": 0, "LensPosition": float(self.lens_position)})
                        dist_m = 1.0 / max(0.01, self.lens_position) if self.lens_position > 0.01 else float('inf')
                        print(f"[CAMERA] 64MP Manual Focus locked at LensPosition={self.lens_position} dioptres (~{dist_m:.1f}m)")
                    elif self.focus_mode == "infinity":
                        self._picam2.set_controls({"AfMode": 0, "LensPosition": 0.0})
                        print("[CAMERA] 64MP Manual Focus locked at Infinity (LensPosition=0.0)")
                    else:
                        # Continuous / Auto Focus:
                        # AfMode: 2 = Continuous
                        # AfRange: 0 = Normal range (prevents getting trapped in extreme macro < 15cm)
                        # AfSpeed: 1 = Fast slew rate
                        af_controls = {
                            "AfMode": 2,
                            "AfRange": 0,
                            "AfSpeed": 1,
                        }
                        self._picam2.set_controls(af_controls)
                        # Trigger an immediate autofocus sweep so lens does not sit idle at macro
                        try:
                            self._picam2.set_controls({"AfTrigger": 0})
                        except Exception:
                            pass
                        print("[CAMERA] 64MP Continuous Autofocus active (Normal Range, Fast Slew, Sweep Triggered)")
                except Exception as af_error:
                    print(f"[CAMERA] Warning: Could not configure 64MP autofocus: {af_error}")
                
                # Hardware VCM focus lock for Arducam 64MP on Raspberry Pi (focus_absolute=160 gives sharp focus from 0.8m to infinity)
                for subdev in ("/dev/v4l-subdev3", "/dev/v4l-subdev1", "/dev/v4l-subdev2"):
                    if os.path.exists(subdev):
                        try:
                            import subprocess
                            subprocess.run(["v4l2-ctl", "-d", subdev, "--set-ctrl=focus_absolute=160"],
                                           check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            print(f"[CAMERA] 64MP VCM hardware focus locked at focus_absolute=160 on {subdev}")
                            break
                        except Exception:
                            pass

                time.sleep(0.5)  # warmup for camera and autofocus to stabilize
                self._use_picamera = True
                print(f"[CAMERA] Picamera2 (64MP AF) initialized successfully at {camera_size[0]}x{camera_size[1]}")
            except Exception as e:
                if use_picamera is True:
                    print(f"[CAMERA] Warning: Forced Picamera2 initialization failed: {e}")
                self._use_picamera = False

        if self._use_picamera:
            self._cap = None
        else:
            #--- USB Video Capture (Fallback / Testing)
            if os.name == 'nt':
                self._cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            else:
                self._cap = cv2.VideoCapture(0)
                try:
                    self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                except Exception:
                    pass
                try:
                    self._cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                except Exception:
                    pass

            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

            try:
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS))
            print(f"[CAMERA] Camera initialized: {actual_w}x{actual_h} @ {actual_fps:.0f} FPS")

        #-- Font for the text in the image
        self.font = cv2.FONT_HERSHEY_PLAIN

        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0    

    def _rotationMatrixToEulerAngles(self,R):
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def _update_fps_read(self):
        t           = time.time()
        self.fps_read    = 1.0/(t - self._t_read)
        self._t_read      = t
        
    def _update_fps_detect(self):
        t           = time.time()
        self.fps_detect  = 1.0/(t - self._t_detect)
        self._t_detect      = t    

    def stop(self):
        self._kill = True
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        try:
            if self._use_picamera and hasattr(self, '_picam2') and self._picam2 is not None:
                self._picam2.stop()
                self._picam2.close()
        except Exception:
            pass

    def trigger_autofocus(self):
        """Triggers an active autofocus search cycle on 64MP camera."""
        if self._use_picamera and hasattr(self, '_picam2') and self._picam2 is not None:
            try:
                self._picam2.set_controls({"AfMode": 2, "AfRange": 0, "AfSpeed": 1, "AfTrigger": 0})
                print("[CAMERA] 64MP Autofocus sweep triggered.")
            except Exception as e:
                print(f"[CAMERA] Autofocus trigger error: {e}")

    def set_lens_position(self, dioptres: float):
        """Sets manual lens position in dioptres (0.0=infinity, 0.5=2m, 1.0=1m)."""
        if self._use_picamera and hasattr(self, '_picam2') and self._picam2 is not None:
            try:
                self._picam2.set_controls({"AfMode": 0, "LensPosition": float(dioptres)})
                dist_m = 1.0 / max(0.01, float(dioptres)) if float(dioptres) > 0.01 else float('inf')
                print(f"[CAMERA] 64MP Lens position set to {dioptres} dioptres (~{dist_m:.1f}m).")
            except Exception as e:
                print(f"[CAMERA] Set lens position error: {e}")

    def set_hardware_focus(self, val=160):
        """Set VCM hardware focus on Linux subdev (0-1023). 160 is optimal for Arducam 64MP."""
        for subdev in ("/dev/v4l-subdev3", "/dev/v4l-subdev1", "/dev/v4l-subdev2"):
            if os.path.exists(subdev):
                try:
                    import subprocess
                    subprocess.run(["v4l2-ctl", "-d", subdev, f"--set-ctrl=focus_absolute={int(val)}"],
                                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f"[CAMERA] Set VCM focus_absolute={val} on {subdev}")
                    return True
                except Exception:
                    pass
        return False

    def track(self, loop=True, verbose=False, show_video=None):
        
        self._kill = False
        if show_video is None: show_video = self._show_video
        
        marker_found = False
        x = y = z = 0
        
        while not self._kill:
            
            #-- Read the camera frame (Picamera2 or OpenCV)
            if self._use_picamera:
                try:
                    raw_frame = self._picam2.capture_array()
                    frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
                    ret = True
                except Exception as e:
                    ret = False
                    frame = None
                    print(f"[CAMERA ERROR] Picamera2 capture failed: {e}")
            else:
                ret, frame = self._cap.read()

            if not ret or frame is None:
                if not loop:
                    return (False, 0.0, 0.0, 0.0)
                time.sleep(0.01)  # Avoid high-CPU busy loop on capture error
                continue

            # Flip image if camera is physically inverted / rotated
            if self._vflip and self._hflip:
                frame = cv2.flip(frame, -1)  # 180-degree rotation (both axes)
            elif self._vflip:
                frame = cv2.flip(frame, 0)   # Vertical flip
            elif self._hflip:
                frame = cv2.flip(frame, 1)   # Horizontal flip

            # Expose the most recent frame to callers (copy to avoid accidental mutation).
            # Convert single-channel mono images to BGR for display/recording compatibility.
            if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 1):
                gray = frame if len(frame.shape) == 2 else frame[:, :, 0]
                frame_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame_bgr = frame

            try:
                with self._frame_lock:
                    self.last_frame = frame_bgr.copy()
                    self.last_frame_ts = time.time()
            except Exception:
                pass

            self._update_fps_read()

            #-- Find all the aruco markers in the image
            if self._use_new_api:
                corners, ids, rejected = self._detector.detectMarkers(gray)
            else:
                corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
                                parameters=self._parameters,
                                cameraMatrix=self._camera_matrix, 
                                distCoeff=self._camera_distortion)
                            
            if ids is not None and self.id_to_find in (np.array(ids).flatten().tolist() if hasattr(ids, 'flatten') else ids[0]):
                marker_found = True
                self._update_fps_detect()
                # select correct marker index
                ids_flat = np.array(ids).flatten()
                idx = int(np.where(ids_flat == self.id_to_find)[0][0]) if hasattr(ids_flat, 'shape') else 0
                #-- Estimate pose with compatible path
                rvec = None
                tvec = None
                try:
                    ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)
                    rvec, tvec = ret[0][idx,0,:], ret[1][idx,0,:]
                except Exception:
                    # Fallback using solvePnP
                    img_pts = corners[idx][0].astype(np.float32)
                    half = (self.marker_size * 0.5)
                    obj_pts = np.array([
                        [-half,  half, 0.0],
                        [ half,  half, 0.0],
                        [ half, -half, 0.0],
                        [-half, -half, 0.0]
                    ], dtype=np.float32)
                    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self._camera_matrix, self._camera_distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok:
                        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self._camera_matrix, self._camera_distortion)
                    rvec = rvec.reshape(-1)
                    tvec = tvec.reshape(-1)
                
                tvec = np.asarray(tvec).flatten()
                rvec = np.asarray(rvec).flatten()
                x = float(tvec[0])
                y = float(tvec[1])
                z = float(tvec[2])

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                try:
                    # Draw custom warning-free axes (avoiding console warnings from solvepnp/drawFrameAxes)
                    axis_len = max(5.0, self.marker_size * 0.5)
                    axes_pts = np.float32([[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]])
                    img_pts, _ = cv2.projectPoints(axes_pts, rvec, tvec, self._camera_matrix, self._camera_distortion)
                    img_pts = img_pts.reshape(-1, 2).astype(int)
                    cv2.line(frame, tuple(img_pts[0]), tuple(img_pts[1]), (0, 0, 255), 2, cv2.LINE_AA) # X-axis Red
                    cv2.line(frame, tuple(img_pts[0]), tuple(img_pts[2]), (0, 255, 0), 2, cv2.LINE_AA) # Y-axis Green
                    cv2.line(frame, tuple(img_pts[0]), tuple(img_pts[3]), (255, 0, 0), 2, cv2.LINE_AA) # Z-axis Blue
                except Exception:
                    pass

                #-- Obtain the rotation matrix tag->camera
                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T

                #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)

                #-- Now get Position and attitude of the camera respect to the marker (flattened for safe scalar formatting)
                pos_camera = np.asarray(-R_tc * np.matrix(tvec).reshape(3, 1)).flatten()
                
                if verbose:
                    print("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f" % (x, y, z, self.fps_detect))

                if show_video:

                    #-- Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (x, y, z)
                    cv2.putText(frame, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)        
                    
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)
                    )
                    cv2.putText(frame, str_attitude, (0, 150), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                        float(pos_camera[0]), float(pos_camera[1]), float(pos_camera[2])
                    )
                    cv2.putText(frame, str_position, (0, 200), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)
                    )
                    cv2.putText(frame, str_attitude, (0, 250), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)


            else:
                marker_found = False
                if verbose:
                    print("Nothing detected - fps = %.0f" % self.fps_read)
            

            if show_video:
                #--- Display the frame
                try:
                    cv2.imshow('frame', frame)
                except Exception:
                    pass

                #--- use 'q' to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    try:
                        if self._cap is not None:
                            self._cap.release()
                    except Exception:
                        pass
                    try:
                        if self._use_picamera and hasattr(self, '_picam2') and self._picam2 is not None:
                            self._picam2.stop()
                            self._picam2.close()
                    except Exception:
                        pass
                    cv2.destroyAllWindows()
                    return (False, 0.0, 0.0, 0.0)
            
            with self._frame_lock:
                self.last_frame = frame.copy()
                self.last_frame_ts = time.time()

            if not loop:
                return (marker_found, float(x), float(y), float(z))

        return (marker_found, float(x), float(y), float(z))
            

if __name__ == "__main__":

    #--- Define Tag
    id_to_find  = 72
    marker_size  = 17.8 #- [cm]

    #--- Get the camera calibration path
    calib_path  = ""
    try:
        camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
        camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')
    except Exception:
        camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
        camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')
    aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
    
    aruco_tracker.track(verbose=True)

