"""
This demo calculates multiple things for different scenarios.

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
import os

# Optional Raspberry Pi camera support via picamera, with graceful fallback
try:
    from picamera import PiCamera
    from picamera.array import PiRGBArray
    _PICAMERA_AVAILABLE = True
except Exception:
    _PICAMERA_AVAILABLE = False

#--- Define Tag
id_to_find  = 72
marker_size  = 10 #- [cm]


#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
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




#--- Get the camera calibration path (relative to this file)
calib_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.loadtxt(os.path.join(calib_dir, 'cameraMatrix_webcam.txt'), delimiter=',')
camera_distortion = np.loadtxt(os.path.join(calib_dir, 'cameraDistortion_webcam.txt'), delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

#--- Create detector parameters compatible with both OpenCV 3/4 APIs
use_new_aruco_api = False
try:
    # OpenCV >= 4.7
    parameters = aruco.DetectorParameters()
    # Improve corner accuracy
    try:
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    except Exception:
        pass
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    use_new_aruco_api = True
except Exception:
    # Legacy API (OpenCV 3.x/4.6 and below)
    parameters  = aruco.DetectorParameters_create()
    try:
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    except Exception:
        pass
    detector = None


#--- Capture the videocamera (this may also be a video or a picture)
#-- Calibration was done at 640x480.
CALIB_W, CALIB_H = 640, 480
REQ_W, REQ_H = 640, 480

use_picamera = False
cap = None
_picam = None
if _PICAMERA_AVAILABLE:
    try:
        _picam = PiCamera()
        _picam.resolution = (REQ_W, REQ_H)
        _picam.framerate = 30
        time.sleep(2.0)  # warmup
        use_picamera = True
    except Exception:
        use_picamera = False

if not use_picamera:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQ_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_H)

#-- Read back actual frame size and scale intrinsics if different from calibration
if use_picamera:
    w_cur, h_cur = REQ_W, REQ_H
else:
    _ret, _tmp = cap.read()
    if _ret:
        h_cur, w_cur = _tmp.shape[:2]
    else:
        h_cur, w_cur = CALIB_H, CALIB_W

sx = float(w_cur) / float(CALIB_W)
sy = float(h_cur) / float(CALIB_H)
if abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3:
    camera_matrix[0,0] *= sx  # fx
    camera_matrix[1,1] *= sy  # fy
    camera_matrix[0,2] *= sx  # cx
    camera_matrix[1,2] *= sy  # cy

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

while True:

    #-- Read the camera frame
    if use_picamera:
        try:
            raw = PiRGBArray(_picam)
            _picam.capture(raw, format="bgr")
            frame = raw.array
            ret = True
        except Exception:
            ret = False
            frame = None
    else:
        ret, frame = cap.read()

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image (handle both APIs)
    if use_new_aruco_api:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    
    if ids is not None and id_to_find in np.array(ids).flatten().tolist():
        # select the corner set for the requested id
        ids_flat = np.array(ids).flatten()
        idx = int(np.where(ids_flat == id_to_find)[0][0])

        #-- Estimate pose: use aruco.estimatePoseSingleMarkers if available, else fall back to solvePnP
        rvec = None
        tvec = None
        try:
            # Legacy API available
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
        except Exception:
            # Fallback using solvePnP
            # use corners of the selected marker; shape (1, 4, 2)
            img_pts = corners[idx][0].astype(np.float32)
            half = (marker_size * 0.5)  # marker_size is in cm; units are arbitrary as long as consistent
            obj_pts = np.array([
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0]
            ], dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, camera_distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, camera_distortion)
            rvec = rvec.reshape(-1)
            tvec = tvec.reshape(-1)

        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)
        try:
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, max(5, marker_size*0.5))
        except Exception:
            try:
                cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, max(5, marker_size*0.5))
            except Exception:
                pass

        #-- Print the tag position in camera frame
        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(float(tvec[0]), float(tvec[1]), float(tvec[2]))
        cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera frame
        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                            math.degrees(yaw_marker))
        cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        #-- Now get Position and attitude f the camera respect to the marker
        pos_camera = -R_tc*np.matrix(tvec).T
        pc = np.asarray(pos_camera).ravel()

        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(float(pc[0]), float(pc[1]), float(pc[2]))
        cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
        cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


    #--- Display the frame
    cv2.imshow('frame', frame)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        try:
            if cap is not None:
                cap.release()
        except Exception:
            pass
        try:
            if use_picamera and _picam is not None:
                _picam.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        break
