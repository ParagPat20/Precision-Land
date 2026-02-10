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

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

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
                camera_size=[640,360],  # Default: 640x360 for 16:9 wide FOV (120°), native: 2304x1296
                show_video=False,
                axis_scale=0.03,
                use_picamera=None
                ):
        
        
        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._show_video    = show_video
        self._axis_scale    = axis_scale
        
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        
        self.is_detected    = False
        self._kill          = False
        
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

        # Decide capture backend (Picamera2 vs OpenCV) with graceful fallback
        # use_picamera: True forces Picamera2, False forces OpenCV, None auto-detect
        self._use_picamera = False
        if use_picamera is True and _PICAMERA2_AVAILABLE:
            self._use_picamera = True
        elif use_picamera is None and _PICAMERA2_AVAILABLE:
            # Auto-enable if library is available
            try:
                self._picam2 = Picamera2()
                # Configure camera with specified resolution
                cfg = self._picam2.create_preview_configuration(main={"size": (int(camera_size[0]), int(camera_size[1]))})
                self._picam2.configure(cfg)
                self._picam2.start()
                
                # Enable Continuous Autofocus for Arducam 64MP OV64A40
                # Continuous autofocus keeps adjusting focus as the drone moves
                try:
                    self._picam2.set_controls({"AfMode": 2})  # AfMode: 2 = Continuous Autofocus
                    print("[CAMERA] Autofocus enabled (Continuous mode)")
                except Exception as af_error:
                    print(f"[CAMERA] Warning: Could not enable autofocus: {af_error}")
                
                time.sleep(0.5)  # warmup for camera and autofocus to stabilize
                self._use_picamera = True
                print(f"[CAMERA] Picamera2 initialized successfully at {camera_size[0]}x{camera_size[1]}")
            except Exception:
                self._use_picamera = False

        if self._use_picamera:
            self._cap = None
        else:
            #--- Capture the videocamera (this may also be a video or a picture)
            self._cap = cv2.VideoCapture(0)
            #-- Set the camera size as the one it was calibrated with
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

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

    def track(self, loop=True, verbose=False, show_video=None):
        
        self._kill = False
        if show_video is None: show_video = self._show_video
        
        marker_found = False
        x = y = z = 0
        
        # Debug: Print tracking configuration on first call only
        if not hasattr(self, '_track_started'):
            self._track_started = True
            print(f"[DEBUG] Starting ArUco tracking - loop={loop}, show_video={show_video}, using_picamera={self._use_picamera}")
        
        while not self._kill:
            
            #-- Read the camera frame (Picamera2 or OpenCV)
            if self._use_picamera:
                try:
                    rgb = self._picam2.capture_array()
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    ret = True
                except Exception as e:
                    ret = False
                    frame = None
                    print(f"[DEBUG] Picamera2 capture failed: {e}")
            else:
                ret, frame = self._cap.read()
                if not ret:
                    print("[DEBUG] OpenCV capture failed")

            if not ret or frame is None:
                print("[DEBUG] Failed to capture frame, skipping iteration")
                continue

            self._update_fps_read()
            
            # Debug: Print frame properties (only once per 30 frames to avoid spam)
            if hasattr(self, '_frame_count'):
                self._frame_count += 1
            else:
                self._frame_count = 0
                print(f"[DEBUG] Frame captured: shape={frame.shape}, dtype={frame.dtype}, fps_read={self.fps_read:.1f}")
            
            #-- Convert in gray scale
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

            #-- Find all the aruco markers in the image
            if self._use_new_api:
                corners, ids, rejected = self._detector.detectMarkers(gray)
            else:
                corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
                                parameters=self._parameters,
                                cameraMatrix=self._camera_matrix, 
                                distCoeff=self._camera_distortion)
            
            # Debug: Print detection results (only when markers are found or occasionally)
            if ids is not None:
                detected_ids = np.array(ids).flatten().tolist() if hasattr(ids, 'flatten') else ids[0]
                print(f"[DEBUG] ArUco markers detected: {detected_ids}, looking for ID {self.id_to_find}")
            elif self._frame_count % 60 == 0:  # Print every 60 frames when no detection
                print(f"[DEBUG] No ArUco markers detected (looking for ID {self.id_to_find})")
                            
            if ids is not None and self.id_to_find in (np.array(ids).flatten().tolist() if hasattr(ids, 'flatten') else ids[0]):
                marker_found = True
                self._update_fps_detect()
                print(f"[DEBUG] ✓ Target marker ID {self.id_to_find} FOUND!")
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
                
                x = tvec[0]
                y = tvec[1]
                z = tvec[2]

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                # Choose a short axis length relative to image size
                axis_len = max(3, int(min(frame.shape[0], frame.shape[1]) * float(self._axis_scale)))
                # Project a few axis endpoints to check if they are in frame
                try:
                    axes = np.float32([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]])
                    img_pts, _ = cv2.projectPoints(axes, rvec, tvec, self._camera_matrix, self._camera_distortion)
                    img_pts = img_pts.reshape(-1,2)
                    in_bounds = np.all((img_pts[:,0] >= 0) & (img_pts[:,0] < frame.shape[1]) &
                                       (img_pts[:,1] >= 0) & (img_pts[:,1] < frame.shape[0]))
                except Exception:
                    in_bounds = True
                try:
                    if in_bounds:
                        aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, axis_len)
                except Exception:
                    try:
                        if in_bounds:
                            cv2.drawFrameAxes(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, axis_len)
                    except Exception:
                        pass

                #-- Obtain the rotation matrix tag->camera
                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T

                #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)

                #-- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc*np.matrix(tvec).T
                
                # print "Camera X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(pos_camera[0], pos_camera[1], pos_camera[2],fps_detect)
                if verbose:
                    print("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f" % (tvec[0], tvec[1], tvec[2], self.fps_detect))

                if show_video:

                    #-- Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                    cv2.putText(frame, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)        
                    
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv2.putText(frame, str_attitude, (0, 150), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv2.putText(frame, str_position, (0, 200), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv2.putText(frame, str_attitude, (0, 250), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)


            else:
                marker_found = False
                if verbose:
                    print("Nothing detected - fps = %.0f" % self.fps_read)
            

            if show_video:
                # Debug: First time video window is created
                if not hasattr(self, '_video_window_created'):
                    self._video_window_created = True
                    print(f"[DEBUG] Creating video window 'frame' with size {frame.shape[1]}x{frame.shape[0]}")
                    print("[DEBUG] NOTE: If running headless (no display), video window will not show. Set show_video=False to disable.")
                
                #--- Display the frame
                try:
                    cv2.imshow('frame', frame)
                except Exception as e:
                    if not hasattr(self, '_video_display_error_shown'):
                        self._video_display_error_shown = True
                        print(f"[DEBUG] Video display error (headless?): {e}")
                        print("[DEBUG] Continuing without video display...")

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
                    break
            
            if not loop:
                if self._frame_count % 30 == 0:  # Debug every 30 frames
                    print(f"[DEBUG] Returning from track(): marker_found={marker_found}, x={x:.1f}, y={y:.1f}, z={z:.1f}")
                return(marker_found, x, y, z)
            

if __name__ == "__main__":

    #--- Define Tag
    id_to_find  = 72
    marker_size  = 4 #- [cm]

    #--- Get the camera calibration path
    calib_path  = ""
    camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')                                      
    aruco_tracker = ArucoSingleTracker(id_to_find=72, marker_size=10, show_video=False, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
    
    aruco_tracker.track(verbose=True)

