"""
This demo calculates multiple things for different scenarios.

ARDUCAM 64MP OV64A40 CAMERA SETUP:
For the Arducam 64MP camera, ensure the following configuration in /boot/firmware/config.txt:
    1. Set camera_auto_detect=0
    2. Add under [all] section: dtoverlay=ov64a40,link-frequency=360000000
    3. Reboot the system

This configuration enables Low-Speed Mode for optimal stability.
Default resolution: 1920×1080 @ 45.65 fps (lowest available mode)

Available camera modes:
- 1920×1080 @ 45.65 fps (default - lowest resolution, highest fps)
- 2312×1736 @ 26.75 fps
- 3840×2160 @ 14.89 fps
- 4624×3472 @ 7.66 fps
- 8000×6000 @ 2.58 fps
- 9248×6944 @ 2.02 fps (maximum resolution)

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
                camera_size=[1920,1080],  # Arducam 64MP: 1920×1080 lowest resolution at 45.65 fps
                show_video=False,
                axis_scale=0.03,
                use_picamera=None,
                detection_scale=0.5  # Scale factor for ArUco detection (0.5 = half size for 2-4x speedup)
                ):
        
        
        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._show_video    = show_video
        self._axis_scale    = axis_scale
        
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        
        # Detection optimization: scale down image for faster ArUco detection
        # 0.5 = half size (960×540) for 2-4x speedup, 1.0 = no scaling
        self._detection_scale = max(0.1, min(1.0, detection_scale))  # Clamp between 0.1 and 1.0
        
        # Scaled camera matrix for detection (if scale != 1.0)
        if self._detection_scale != 1.0:
            self._camera_matrix_scaled = camera_matrix.copy()
            self._camera_matrix_scaled[0, 0] *= self._detection_scale  # fx
            self._camera_matrix_scaled[1, 1] *= self._detection_scale  # fy
            self._camera_matrix_scaled[0, 2] *= self._detection_scale  # cx
            self._camera_matrix_scaled[1, 2] *= self._detection_scale  # cy
        else:
            self._camera_matrix_scaled = camera_matrix
        
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
                # Initialize Picamera2 for Arducam 64MP with autofocus
                self._picam2 = Picamera2()
                
                # Create configuration with specified resolution
                # For Arducam 64MP OV64A40, use native resolution from available modes
                # Lowest: 1920×1080 @ 45.65 fps
                cfg = self._picam2.create_preview_configuration(main={"size": (int(camera_size[0]), int(camera_size[1]))})
                self._picam2.configure(cfg)
                
                # Start the camera
                self._picam2.start()
                
                # Enable continuous autofocus for Arducam 64MP OV64A40
                # AfMode: 2 = Continuous autofocus (always active)
                try:
                    self._picam2.set_controls({"AfMode": 2})  # Continuous autofocus
                    print("Arducam 64MP: Continuous autofocus enabled")
                except Exception as e:
                    print(f"Note: Could not set autofocus mode - {e}")
                
                time.sleep(0.5)  # warmup for camera and autofocus initialization
                self._use_picamera = True
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

    def trigger_autofocus(self):
        """
        Manually trigger autofocus for Arducam 64MP camera.
        Useful for refocusing when the target distance changes significantly.
        """
        if self._use_picamera and hasattr(self, '_picam2') and self._picam2 is not None:
            try:
                # Trigger autofocus cycle
                self._picam2.set_controls({"AfTrigger": 0})  # Trigger autofocus
                print("Arducam 64MP: Autofocus triggered")
            except Exception as e:
                print(f"Note: Could not trigger autofocus - {e}")
    
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
        
        while not self._kill:
            
            #-- Read the camera frame (Picamera2 or OpenCV)
            if self._use_picamera:
                try:
                    rgb = self._picam2.capture_array()
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    ret = True
                except Exception as e:
                    print(f"Picamera2 capture error: {e}")
                    ret = False
                    frame = None
            else:
                ret, frame = self._cap.read()

            # Check if frame is valid before processing
            if not ret or frame is None:
                if verbose:
                    print("Failed to capture frame")
                if not loop:
                    return (False, 0, 0, 0)
                continue

            self._update_fps_read()
            
            #-- Convert in gray scale
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

            # Downscale image for faster ArUco detection (performance optimization)
            if self._detection_scale != 1.0:
                # Calculate new dimensions
                detect_width = int(gray.shape[1] * self._detection_scale)
                detect_height = int(gray.shape[0] * self._detection_scale)
                gray_detect = cv2.resize(gray, (detect_width, detect_height), interpolation=cv2.INTER_AREA)
            else:
                gray_detect = gray

            #-- Find all the aruco markers in the downscaled image
            if self._use_new_api:
                corners, ids, rejected = self._detector.detectMarkers(gray_detect)
            else:
                corners, ids, rejected = aruco.detectMarkers(image=gray_detect, dictionary=self._aruco_dict, 
                                parameters=self._parameters,
                                cameraMatrix=self._camera_matrix_scaled, 
                                distCoeff=self._camera_distortion)
            
            # Scale corners back to original resolution for accurate pose estimation
            if self._detection_scale != 1.0 and corners is not None and len(corners) > 0:
                corners = [corner / self._detection_scale for corner in corners]
                            
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
                if verbose:
                    print("Nothing detected - fps = %.0f" % self.fps_read)
            

            if show_video:
                #--- Display the frame
                cv2.imshow('frame', frame)

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
            
            if not loop: return(marker_found, x, y, z)
            

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

