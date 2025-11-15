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
                camera_size=[4608,2592],  # Default resolution for imx708_wide_noir camera at 30fps
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
                # Configure for imx708_wide_noir camera at 4608x2592 resolution, 30fps
                # Use RGB888 format - Picamera2 returns RGB, we'll convert to BGR for OpenCV
                cfg = self._picam2.create_preview_configuration(
                    main={"size": (int(camera_size[0]), int(camera_size[1]))}
                )
                self._picam2.configure(cfg)
                # Set frame rate to 30 fps
                self._picam2.set_controls({"FrameRate": 30.0})
                self._picam2.start()
                time.sleep(0.5)  # warmup
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
        # Calculate font scale and thickness based on camera resolution for readability
        # Base scale for 640x480 resolution, scale up for larger resolutions
        base_resolution = 640
        resolution_factor = max(camera_size[0], camera_size[1]) / base_resolution
        self.font_scale = max(2.0, min(6.0, resolution_factor * 0.8))  # Scale between 2-6
        self.font_thickness = max(2, int(resolution_factor * 0.6))  # Thickness scales with resolution
        self.line_spacing = int(40 * resolution_factor * 0.8)  # Spacing between text lines

        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0
        
        #--- OpenCV tracker for hybrid ArUco + visual tracking
        # Tracker state variables
        self._tracker = None  # OpenCV tracker object (CSRT or KCF)
        self._tracker_active = False  # Whether tracker is currently active
        self._last_aruco_detection_time = 0  # Timestamp of last ArUco detection
        self._last_aruco_update_time = 0  # Timestamp of last ArUco update to tracker
        self._tracker_expiry_time = 4.0  # Tracker expires after 10 seconds without ArUco detection (increased from 3s)
        self._aruco_update_interval = 1.5  # Update tracker with ArUco every 0.5 seconds (reduced from 1.5s for more frequent updates)
        self._tracked_bbox = None  # Current tracked bounding box (x, y, w, h)
        self._last_known_tvec = None  # Last known position from ArUco detection (for interpolation)
        self._last_known_rvec = None  # Last known rotation from ArUco detection
        
        # Drone communication tracking variables
        self._last_drone_send_time = 0  # Timestamp of last successful send to drone
        self._drone_send_count = 0  # Total count of messages sent to drone
        self._last_drone_send_data = None  # Last data sent: (x_cm, y_cm, z_cm, angle_x, angle_y, dist_m)
        self._drone_send_enabled = True  # Whether drone sending is enabled    

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
    
    def update_drone_send_status(self, x_cm, y_cm, z_cm, angle_x, angle_y, dist_m):
        """Update drone send status when data is sent to drone
        Called from main.py after successfully sending MAVLink message
        """
        self._last_drone_send_time = time.time()
        self._drone_send_count += 1
        self._last_drone_send_data = (x_cm, y_cm, z_cm, angle_x, angle_y, dist_m)
    
    def _initialize_tracker(self, frame, bbox):
        """Initialize OpenCV tracker with bounding box from ArUco detection"""
        try:
            # Try different tracker APIs based on OpenCV version
            # OpenCV 4.5.1+ moved trackers to cv2.legacy or use new object-oriented API
            tracker_created = False
            
            # Try new object-oriented API (OpenCV 4.5.1+)
            if not tracker_created:
                try:
                    self._tracker = cv2.TrackerCSRT.create()
                    tracker_created = True
                except (AttributeError, cv2.error):
                    pass
            
            # Try legacy API (OpenCV 4.5.1+)
            if not tracker_created:
                try:
                    self._tracker = cv2.legacy.TrackerCSRT_create()
                    tracker_created = True
                except (AttributeError, cv2.error):
                    pass
            
            # Try old API (OpenCV 3.x and early 4.x) - for backward compatibility
            if not tracker_created:
                try:
                    self._tracker = cv2.TrackerCSRT_create()
                    tracker_created = True
                except (AttributeError, cv2.error):
                    pass
            
            # Try KCF with new object-oriented API as fallback (faster, less accurate)
            if not tracker_created:
                try:
                    self._tracker = cv2.TrackerKCF.create()
                    tracker_created = True
                except (AttributeError, cv2.error):
                    pass
            
            # Try KCF with legacy API
            if not tracker_created:
                try:
                    self._tracker = cv2.legacy.TrackerKCF_create()
                    tracker_created = True
                except (AttributeError, cv2.error):
                    pass
            
            # Try old KCF API for backward compatibility
            if not tracker_created:
                try:
                    self._tracker = cv2.TrackerKCF_create()
                    tracker_created = True
                except (AttributeError, cv2.error):
                    pass
            
            if not tracker_created:
                print("No compatible tracker found in OpenCV")
                self._tracker = None
                self._tracker_active = False
                return False
            
            # Initialize the tracker with the frame and bounding box
            self._tracker.init(frame, bbox)
            self._tracker_active = True
            self._tracked_bbox = bbox
            return True
            
        except Exception as e:
            print(f"Tracker initialization failed: {e}")
            self._tracker = None
            self._tracker_active = False
            return False
    
    def _update_tracker(self, frame):
        """Update OpenCV tracker with current frame, returns True if tracking successful"""
        if self._tracker is None or not self._tracker_active:
            return False
        
        try:
            success, bbox = self._tracker.update(frame)
            if success:
                self._tracked_bbox = bbox
                return True
            else:
                # Tracker lost the target
                self._tracker_active = False
                return False
        except Exception:
            self._tracker_active = False
            return False
    
    def _estimate_pose_from_bbox(self, bbox, frame_shape):
        """Estimate approximate pose from tracked bounding box center
        Uses last known z distance and camera intrinsics to estimate x, y
        Returns (x, y, z) in cm, or None if estimation fails
        """
        if bbox is None or self._last_known_tvec is None:
            return None
        
        # Extract bounding box coordinates
        x_bbox, y_bbox, w_bbox, h_bbox = bbox
        
        # Calculate center of bounding box
        center_x = x_bbox + w_bbox / 2.0
        center_y = y_bbox + h_bbox / 2.0
        
        # Use last known z distance (in cm)
        z_cm = self._last_known_tvec[2]
        
        # Estimate x, y from bbox center using camera intrinsics
        # Convert pixel coordinates to normalized camera coordinates
        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        cx = self._camera_matrix[0, 2]
        cy = self._camera_matrix[1, 2]
        
        # Normalized coordinates
        x_norm = (center_x - cx) / fx
        y_norm = (center_y - cy) / fy
        
        # Estimate x, y in cm (approximate, using last known z)
        x_cm = x_norm * z_cm
        y_cm = y_norm * z_cm
        
        return (x_cm, y_cm, z_cm)
    
    def _get_bbox_from_corners(self, corners, frame_shape):
        """Extract bounding box from ArUco marker corners"""
        # corners shape: (1, 4, 2) or (4, 2)
        if len(corners.shape) == 3:
            corners_2d = corners[0]
        else:
            corners_2d = corners
        
        # Get bounding box
        x_coords = corners_2d[:, 0]
        y_coords = corners_2d[:, 1]
        
        x_min = int(np.min(x_coords))
        y_min = int(np.min(y_coords))
        x_max = int(np.max(x_coords))
        y_max = int(np.max(y_coords))
        
        # Add large padding to track the entire page/object, not just the marker
        # Use 150% padding (1.5x marker size on each side) to capture the whole page
        marker_width = x_max - x_min
        marker_height = y_max - y_min
        padding_x = int(marker_width * 1.5)  # 150% horizontal padding
        padding_y = int(marker_height * 1.5)  # 150% vertical padding
        
        x_min = max(0, x_min - padding_x)
        y_min = max(0, y_min - padding_y)
        x_max = min(frame_shape[1], x_max + padding_x)
        y_max = min(frame_shape[0], y_max + padding_y)
        
        width = x_max - x_min
        height = y_max - y_min
        
        return (x_min, y_min, width, height)    

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
        tracking_confidence = 0.0  # 1.0 = ArUco detection, 0.7 = tracker, 0.0 = no tracking
        
        while not self._kill:
            current_time = time.time()
            
            #-- Read the camera frame (Picamera2 or OpenCV)
            if self._use_picamera:
                try:
                    # Get frame from Picamera2
                    cam_array = self._picam2.capture_array()
                    # Picamera2 returns RGB888, convert to BGR for OpenCV compatibility
                    # OpenCV functions expect BGR format (cvtColor, imshow, etc.)
                    frame = cv2.cvtColor(cam_array, cv2.COLOR_RGB2BGR)
                    ret = True
                except Exception as e:
                    ret = False
                    frame = None
                    if verbose:
                        print(f"Frame capture error: {e}")
            else:
                # OpenCV VideoCapture already returns BGR format
                ret, frame = self._cap.read()
            
            if not ret or frame is None:
                continue

            self._update_fps_read()
            frame_shape = frame.shape
            
            #-- Convert in gray scale for ArUco detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #-- Check if tracker has expired (10 seconds without ArUco detection)
            if self._tracker_active and (current_time - self._last_aruco_detection_time) > self._tracker_expiry_time:
                self._tracker_active = False
                self._tracker = None
                if verbose:
                    print(f"Tracker expired after {self._tracker_expiry_time:.0f} seconds without ArUco detection")

            #-- Find all the aruco markers in the image
            # Only detect ArUco if: not tracking, or it's time for update (every 0.5s), or tracker expired
            should_detect_aruco = (not self._tracker_active or 
                                 (current_time - self._last_aruco_update_time) >= self._aruco_update_interval)
            
            aruco_detected = False
            corners = None
            ids = None
            
            if should_detect_aruco:
                if self._use_new_api:
                    corners, ids, rejected = self._detector.detectMarkers(gray)
                else:
                    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
                                    parameters=self._parameters,
                                    cameraMatrix=self._camera_matrix, 
                                    distCoeff=self._camera_distortion)
                            
            if ids is not None and self.id_to_find in (np.array(ids).flatten().tolist() if hasattr(ids, 'flatten') else ids[0]):
                aruco_detected = True
                marker_found = True
                tracking_confidence = 1.0  # Full confidence from ArUco detection
                self._update_fps_detect()
                self._last_aruco_detection_time = current_time
                
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
                
                # Store last known position for tracker interpolation
                self._last_known_tvec = tvec.copy()
                self._last_known_rvec = rvec.copy()
                
                # Initialize or update tracker with ArUco bounding box
                bbox = self._get_bbox_from_corners(corners[idx], frame_shape)
                if not self._tracker_active:
                    # Initialize tracker on first detection
                    if self._initialize_tracker(frame, bbox):
                        if verbose:
                            print("Tracker initialized with ArUco detection")
                        self._last_aruco_update_time = current_time
                elif (current_time - self._last_aruco_update_time) >= self._aruco_update_interval:
                    # Update tracker with new ArUco detection (every 0.5s for more frequent updates)
                    try:
                        # Reinitialize tracker with new bbox to keep it accurate
                        self._tracker = None
                        if self._initialize_tracker(frame, bbox):
                            if verbose:
                                print("Tracker updated with ArUco detection")
                            self._last_aruco_update_time = current_time
                    except Exception as e:
                        if verbose:
                            print(f"Tracker update failed: {e}")
                else:
                    # Tracker is active but not time for ArUco update yet - still update tracker every frame
                    # This keeps tracking smooth between ArUco updates
                    self._update_tracker(frame)

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
                    # Calculate y position for text lines with proper spacing
                    y_pos = self.line_spacing
                    
                    #-- Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                    cv2.putText(frame, str_position, (0, y_pos), self.font, self.font_scale, (0, 255, 0), self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing
                    
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv2.putText(frame, str_attitude, (0, y_pos), self.font, self.font_scale, (0, 255, 0), self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv2.putText(frame, str_position, (0, y_pos), self.font, self.font_scale, (0, 255, 0), self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv2.putText(frame, str_attitude, (0, y_pos), self.font, self.font_scale, (0, 255, 0), self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing
                    
                    #-- Display tracking status with confidence level
                    if tracking_confidence >= 1.0:
                        status_text = "TRACKING STATUS: ArUco Detection (100%% confidence)"
                        status_color = (0, 255, 0)  # Green
                    elif tracking_confidence >= 0.7:
                        status_text = "TRACKING STATUS: Visual Tracker (70%% confidence)"
                        status_color = (255, 165, 0)  # Orange
                    else:
                        status_text = "TRACKING STATUS: No Tracking (0%% confidence)"
                        status_color = (0, 0, 255)  # Red
                    cv2.putText(frame, status_text, (0, y_pos), self.font, self.font_scale, status_color, self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing
                    
                    #-- Display tracker expiration time status
                    if self._tracker_active:
                        time_since_aruco = current_time - self._last_aruco_detection_time
                        time_remaining = self._tracker_expiry_time - time_since_aruco
                        if time_remaining > 0:
                            expiry_text = "TRACKER EXPIRY: %.1f seconds remaining" % time_remaining
                            # Color changes from green to red as time runs out (now 10s total)
                            if time_remaining > 5.0:
                                expiry_color = (0, 255, 0)  # Green
                            elif time_remaining > 2.0:
                                expiry_color = (0, 255, 255)  # Yellow
                            else:
                                expiry_color = (0, 0, 255)  # Red
                        else:
                            expiry_text = "TRACKER EXPIRY: Expired"
                            expiry_color = (0, 0, 255)  # Red
                        cv2.putText(frame, expiry_text, (0, y_pos), self.font, self.font_scale, expiry_color, self.font_thickness, cv2.LINE_AA)
                        y_pos += self.line_spacing
                    else:
                        expiry_text = "TRACKER EXPIRY: Not active"
                        cv2.putText(frame, expiry_text, (0, y_pos), self.font, self.font_scale, (128, 128, 128), self.font_thickness, cv2.LINE_AA)
                        y_pos += self.line_spacing
                    
                    #-- Display current time status
                    time_text = "TIME: %.3f seconds" % current_time
                    cv2.putText(frame, time_text, (0, y_pos), self.font, self.font_scale, (255, 255, 255), self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing
                    
                    #-- Display drone communication status
                    if self._last_drone_send_time > 0:
                        time_since_send = current_time - self._last_drone_send_time
                        if time_since_send < 0.5:  # Recently sent (within 0.5 seconds)
                            drone_status = "DRONE COMM: Sending data (Count: %d)" % self._drone_send_count
                            drone_color = (0, 255, 0)  # Green
                        elif time_since_send < 2.0:  # Sent recently (within 2 seconds)
                            drone_status = "DRONE COMM: Last send %.1fs ago (Count: %d)" % (time_since_send, self._drone_send_count)
                            drone_color = (0, 255, 255)  # Yellow
                        else:  # Not sending
                            drone_status = "DRONE COMM: Not sending (%.1fs ago, Count: %d)" % (time_since_send, self._drone_send_count)
                            drone_color = (128, 128, 128)  # Gray
                    else:
                        drone_status = "DRONE COMM: No data sent yet"
                        drone_color = (128, 128, 128)  # Gray
                    cv2.putText(frame, drone_status, (0, y_pos), self.font, self.font_scale, drone_color, self.font_thickness, cv2.LINE_AA)
                    y_pos += self.line_spacing
                    
                    #-- Display last sent data to drone
                    if self._last_drone_send_data is not None:
                        x_sent, y_sent, z_sent, angle_x_sent, angle_y_sent, dist_sent = self._last_drone_send_data
                        sent_data_text = "LAST SENT: x=%.0fcm y=%.0fcm z=%.0fcm | angles(%.3f,%.3f) | dist=%.2fm" % (
                            x_sent, y_sent, z_sent, angle_x_sent, angle_y_sent, dist_sent)
                        cv2.putText(frame, sent_data_text, (0, y_pos), self.font, self.font_scale * 0.8, (200, 200, 200), self.font_thickness, cv2.LINE_AA)

            else:
                # ArUco not detected - try using tracker if active
                if self._tracker_active and (current_time - self._last_aruco_detection_time) <= self._tracker_expiry_time:
                    # Update tracker with current frame
                    if self._update_tracker(frame):
                        # Estimate pose from tracked bounding box (70% confidence)
                        tracked_pose = self._estimate_pose_from_bbox(self._tracked_bbox, frame_shape)
                        if tracked_pose is not None:
                            marker_found = True
                            tracking_confidence = 0.7  # 70% confidence from tracker
                            x, y, z = tracked_pose
                            
                            if verbose:
                                print("Tracking (no ArUco): x=%.1f y=%.1f z=%.1f" % (x, y, z))
                            
                            if show_video:
                                # Draw tracked bounding box with thicker lines for high resolution
                                x_bbox, y_bbox, w_bbox, h_bbox = self._tracked_bbox
                                box_thickness = max(2, int(self.font_thickness * 0.5))
                                cv2.rectangle(frame, (int(x_bbox), int(y_bbox)), 
                                            (int(x_bbox + w_bbox), int(y_bbox + h_bbox)), (255, 165, 0), box_thickness)
                                
                                # Calculate y position for text lines with proper spacing
                                y_pos = self.line_spacing
                                
                                # Display tracking status
                                status_text = "TRACKING STATUS: Visual Tracker (70%% confidence)"
                                cv2.putText(frame, status_text, (0, y_pos), self.font, self.font_scale, (255, 165, 0), self.font_thickness, cv2.LINE_AA)
                                y_pos += self.line_spacing
                                
                                # Display tracked position
                                pos_text = "TRACKED Position x=%4.0f  y=%4.0f  z=%4.0f" % (x, y, z)
                                cv2.putText(frame, pos_text, (0, y_pos), self.font, self.font_scale, (255, 165, 0), self.font_thickness, cv2.LINE_AA)
                                y_pos += self.line_spacing
                                
                                # Display tracker expiration time status
                                time_since_aruco = current_time - self._last_aruco_detection_time
                                time_remaining = self._tracker_expiry_time - time_since_aruco
                                if time_remaining > 0:
                                    expiry_text = "TRACKER EXPIRY: %.1f seconds remaining" % time_remaining
                                    # Color changes from green to red as time runs out (now 10s total)
                                    if time_remaining > 5.0:
                                        expiry_color = (0, 255, 0)  # Green
                                    elif time_remaining > 2.0:
                                        expiry_color = (0, 255, 255)  # Yellow
                                    else:
                                        expiry_color = (0, 0, 255)  # Red
                                else:
                                    expiry_text = "TRACKER EXPIRY: Expired"
                                    expiry_color = (0, 0, 255)  # Red
                                cv2.putText(frame, expiry_text, (0, y_pos), self.font, self.font_scale, expiry_color, self.font_thickness, cv2.LINE_AA)
                                y_pos += self.line_spacing
                                
                                # Display current time status
                                time_text = "TIME: %.3f seconds" % current_time
                                cv2.putText(frame, time_text, (0, y_pos), self.font, self.font_scale, (255, 255, 255), self.font_thickness, cv2.LINE_AA)
                                y_pos += self.line_spacing
                                
                                #-- Display drone communication status
                                if self._last_drone_send_time > 0:
                                    time_since_send = current_time - self._last_drone_send_time
                                    if time_since_send < 0.5:  # Recently sent (within 0.5 seconds)
                                        drone_status = "DRONE COMM: Sending data (Count: %d)" % self._drone_send_count
                                        drone_color = (0, 255, 0)  # Green
                                    elif time_since_send < 2.0:  # Sent recently (within 2 seconds)
                                        drone_status = "DRONE COMM: Last send %.1fs ago (Count: %d)" % (time_since_send, self._drone_send_count)
                                        drone_color = (0, 255, 255)  # Yellow
                                    else:  # Not sending
                                        drone_status = "DRONE COMM: Not sending (%.1fs ago, Count: %d)" % (time_since_send, self._drone_send_count)
                                        drone_color = (128, 128, 128)  # Gray
                                else:
                                    drone_status = "DRONE COMM: No data sent yet"
                                    drone_color = (128, 128, 128)  # Gray
                                cv2.putText(frame, drone_status, (0, y_pos), self.font, self.font_scale, drone_color, self.font_thickness, cv2.LINE_AA)
                                y_pos += self.line_spacing
                                
                                #-- Display last sent data to drone
                                if self._last_drone_send_data is not None:
                                    x_sent, y_sent, z_sent, angle_x_sent, angle_y_sent, dist_sent = self._last_drone_send_data
                                    sent_data_text = "LAST SENT: x=%.0fcm y=%.0fcm z=%.0fcm | angles(%.3f,%.3f) | dist=%.2fm" % (
                                        x_sent, y_sent, z_sent, angle_x_sent, angle_y_sent, dist_sent)
                                    cv2.putText(frame, sent_data_text, (0, y_pos), self.font, self.font_scale * 0.8, (200, 200, 200), self.font_thickness, cv2.LINE_AA)
                    else:
                        # Tracker lost the target
                        self._tracker_active = False
                        marker_found = False
                        tracking_confidence = 0.0
                        if verbose:
                            print("Tracker lost target")
                else:
                    # No ArUco and no active tracker
                    marker_found = False
                    tracking_confidence = 0.0
                    if verbose:
                        print("Nothing detected - fps = %.0f" % self.fps_read)
                    
                    # Display status when nothing is detected
                    if show_video:
                        y_pos = self.line_spacing
                        status_text = "TRACKING STATUS: No Detection (0%% confidence)"
                        cv2.putText(frame, status_text, (0, y_pos), self.font, self.font_scale, (0, 0, 255), self.font_thickness, cv2.LINE_AA)
                        y_pos += self.line_spacing
                        expiry_text = "TRACKER EXPIRY: Not active"
                        cv2.putText(frame, expiry_text, (0, y_pos), self.font, self.font_scale, (128, 128, 128), self.font_thickness, cv2.LINE_AA)
                        y_pos += self.line_spacing
                        time_text = "TIME: %.3f seconds" % current_time
                        cv2.putText(frame, time_text, (0, y_pos), self.font, self.font_scale, (255, 255, 255), self.font_thickness, cv2.LINE_AA)
                        y_pos += self.line_spacing
                        
                        #-- Display drone communication status
                        if self._last_drone_send_time > 0:
                            time_since_send = current_time - self._last_drone_send_time
                            if time_since_send < 0.5:  # Recently sent (within 0.5 seconds)
                                drone_status = "DRONE COMM: Sending data (Count: %d)" % self._drone_send_count
                                drone_color = (0, 255, 0)  # Green
                            elif time_since_send < 2.0:  # Sent recently (within 2 seconds)
                                drone_status = "DRONE COMM: Last send %.1fs ago (Count: %d)" % (time_since_send, self._drone_send_count)
                                drone_color = (0, 255, 255)  # Yellow
                            else:  # Not sending
                                drone_status = "DRONE COMM: Not sending (%.1fs ago, Count: %d)" % (time_since_send, self._drone_send_count)
                                drone_color = (128, 128, 128)  # Gray
                        else:
                            drone_status = "DRONE COMM: No data sent yet"
                            drone_color = (128, 128, 128)  # Gray
                        cv2.putText(frame, drone_status, (0, y_pos), self.font, self.font_scale, drone_color, self.font_thickness, cv2.LINE_AA)
                        y_pos += self.line_spacing
                        
                        #-- Display last sent data to drone
                        if self._last_drone_send_data is not None:
                            x_sent, y_sent, z_sent, angle_x_sent, angle_y_sent, dist_sent = self._last_drone_send_data
                            sent_data_text = "LAST SENT: x=%.0fcm y=%.0fcm z=%.0fcm | angles(%.3f,%.3f) | dist=%.2fm" % (
                                x_sent, y_sent, z_sent, angle_x_sent, angle_y_sent, dist_sent)
                            cv2.putText(frame, sent_data_text, (0, y_pos), self.font, self.font_scale * 0.8, (200, 200, 200), self.font_thickness, cv2.LINE_AA)
            

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
            
            if not loop: 
                # Return marker_found, x, y, z, and tracking_confidence
                # tracking_confidence: 1.0 = ArUco, 0.7 = tracker, 0.0 = none
                return (marker_found, x, y, z, tracking_confidence)
            

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

























