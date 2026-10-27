"""
Centralized Configuration & Parameter Manager for Precision-Land Codebase.
Manages parameters across Precision Landing, Vision, RTSP Streaming, Servos, MAVLink, and System Services.
Provides persistence to codebase_params.json and live integration with os.environ.
"""

import os
import json
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

# Root directory of the repository
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CONFIG_DIR = REPO_ROOT / "config"
CONFIG_FILE = CONFIG_DIR / "codebase_params.json"

# Master Parameter Definitions (Metadata, Types, Units, Descriptions, Defaults)
PARAM_DEFINITIONS: List[Dict[str, Any]] = [
    # -------------------------------------------------------------------------
    # 1. PRECISION LANDING & ARUCO TARGETING
    # -------------------------------------------------------------------------
    {
        "name": "PLND_TARGET_ID",
        "category": "Precision Landing",
        "default": 72,
        "type": "int",
        "unit": "ID",
        "env_var": "JECH_PLND_TARGET_ID",
        "description": "Primary ArUco marker ID to track for precision landing (standard: 72 or 0)."
    },
    {
        "name": "PLND_MARKER_SIZE",
        "category": "Precision Landing",
        "default": 10.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_PLND_MARKER_SIZE",
        "description": "Physical side length of the primary ArUco marker in centimeters (e.g., 10.0 cm or 35.0 cm)."
    },
    {
        "name": "PLND_DUAL_STAGE",
        "category": "Precision Landing",
        "default": False,
        "type": "bool",
        "env_var": "JECH_PLND_DUAL_STAGE",
        "description": "Enable dual-stage landing handover (tracks Big 350mm marker from 10m to 3m, then switches to Small 100mm marker below 3m)."
    },
    {
        "name": "PLND_BIG_MARKER_ID",
        "category": "Precision Landing",
        "default": 0,
        "type": "int",
        "unit": "ID",
        "env_var": "JECH_PLND_BIG_MARKER_ID",
        "description": "ArUco ID for the large 350mm approach marker located in the middle of the dock."
    },
    {
        "name": "PLND_BIG_MARKER_SIZE",
        "category": "Precision Landing",
        "default": 35.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_PLND_BIG_MARKER_SIZE",
        "description": "Physical size of the large approach marker in centimeters."
    },
    {
        "name": "PLND_SMALL_MARKER_ID",
        "category": "Precision Landing",
        "default": 1,
        "type": "int",
        "unit": "ID",
        "env_var": "JECH_PLND_SMALL_MARKER_ID",
        "description": "ArUco ID for the small 100mm terminal touchdown marker."
    },
    {
        "name": "PLND_SMALL_MARKER_SIZE",
        "category": "Precision Landing",
        "default": 10.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_PLND_SMALL_MARKER_SIZE",
        "description": "Physical size of the small touchdown marker in centimeters."
    },
    {
        "name": "PLND_SMALL_FWD_OFFSET",
        "category": "Precision Landing",
        "default": 25.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_PLND_SMALL_FWD_OFFSET",
        "description": "Physical forward offset of small marker ahead of the dock center to align with the nose camera."
    },
    {
        "name": "PLND_HANDOVER_ALT",
        "category": "Precision Landing",
        "default": 3.0,
        "type": "float",
        "unit": "m",
        "env_var": "JECH_PLND_HANDOVER_ALT",
        "description": "Altitude (meters) where tracking transitions from the big approach marker to the small terminal marker."
    },
    {
        "name": "PLND_SEND_FREQ",
        "category": "Precision Landing",
        "default": 10.0,
        "type": "float",
        "unit": "Hz",
        "env_var": "JECH_PLND_SEND_FREQ",
        "description": "Dispatch rate of MAVLink LANDING_TARGET messages sent to ArduPilot (recommended: 10 to 20 Hz)."
    },
    {
        "name": "PLND_CONF_THRESH",
        "category": "Precision Landing",
        "default": 20.0,
        "type": "float",
        "unit": "%",
        "env_var": "JECH_PLND_CONF_THRESH",
        "description": "Minimum rolling buffer detection confidence score required to send landing target vectors."
    },
    {
        "name": "PLND_BUFFER_SIZE",
        "category": "Precision Landing",
        "default": 7,
        "type": "int",
        "unit": "frames",
        "env_var": "JECH_PLND_BUFFER_SIZE",
        "description": "Number of recent frames in the rolling stability buffer to smooth momentary detection loss."
    },
    {
        "name": "PLND_LAND_ALT_CM",
        "category": "Precision Landing",
        "default": 50.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_PLND_LAND_ALT_CM",
        "description": "Visual altitude threshold below which the vehicle commands final touchdown and motor disarm."
    },
    {
        "name": "PLND_LAND_SPEED_CMS",
        "category": "Precision Landing",
        "default": 30.0,
        "type": "float",
        "unit": "cm/s",
        "env_var": "JECH_PLND_LAND_SPEED_CMS",
        "description": "Controlled descent velocity in cm/s during visual precision approach."
    },

    # -------------------------------------------------------------------------
    # 2. CAMERA GEOMETRY & VISION PIPELINE
    # -------------------------------------------------------------------------
    {
        "name": "CAM_TILT_PITCH_DEG",
        "category": "Camera & Vision",
        "default": 20.0,
        "type": "float",
        "unit": "deg",
        "env_var": "JECH_CAM_TILT_PITCH_DEG",
        "description": "Downward forward pitch tilt angle of the landing camera relative to nadir straight down (degrees)."
    },
    {
        "name": "CAM_OFFSET_FWD_CM",
        "category": "Camera & Vision",
        "default": 0.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_CAM_OFFSET_FORWARD_CM",
        "description": "Forward lever-arm distance from vehicle Center of Gravity (CG) to camera optical center (keep 0.0 if handled via PLND_CAM_POS_X in ArduPilot)."
    },
    {
        "name": "CAM_OFFSET_RIGHT_CM",
        "category": "Camera & Vision",
        "default": 0.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_CAM_OFFSET_RIGHT_CM",
        "description": "Lateral starboard distance from vehicle CG to camera optical center (cm)."
    },
    {
        "name": "CAM_OFFSET_DOWN_CM",
        "category": "Camera & Vision",
        "default": 0.0,
        "type": "float",
        "unit": "cm",
        "env_var": "JECH_CAM_OFFSET_DOWN_CM",
        "description": "Downward distance from vehicle CG to camera optical center (cm)."
    },
    {
        "name": "CAM_RESOLUTION",
        "category": "Camera & Vision",
        "default": "1280x720",
        "type": "choice",
        "options": ["1920x1080", "1456x1088", "1280x720", "640x480", "4608x2592"],
        "unit": "WxH",
        "env_var": "JECH_CAM_RESOLUTION",
        "description": "Camera video capture resolution Width x Height (1280x720 recommended for high FPS)."
    },
    {
        "name": "CAM_FPS",
        "category": "Camera & Vision",
        "default": 60,
        "type": "int",
        "unit": "FPS",
        "env_var": "JECH_CAM_FPS",
        "description": "Sensor capture target framerate (30, 60, or 120 FPS depending on camera)."
    },
    {
        "name": "CAM_VFLIP",
        "category": "Camera & Vision",
        "default": True,
        "type": "bool",
        "env_var": "JECH_VFLIP",
        "description": "Vertically flip camera image in software (required when camera is mounted upside down/rotated)."
    },
    {
        "name": "CAM_HFLIP",
        "category": "Camera & Vision",
        "default": False,
        "type": "bool",
        "env_var": "JECH_HFLIP",
        "description": "Horizontally flip camera image in software."
    },
    {
        "name": "CAM_FLIP_X",
        "category": "Camera & Vision",
        "default": False,
        "type": "bool",
        "env_var": "JECH_FLIP_X",
        "description": "Invert X coordinate axis in camera-to-UAV body transformation."
    },
    {
        "name": "CAM_FLIP_Y",
        "category": "Camera & Vision",
        "default": False,
        "type": "bool",
        "env_var": "JECH_FLIP_Y",
        "description": "Invert Y coordinate axis in camera-to-UAV body transformation."
    },
    {
        "name": "CAM_FOCUS_MODE",
        "category": "Camera & Vision",
        "default": "fixed",
        "type": "choice",
        "options": ["fixed", "manual", "auto", "continuous", "infinity"],
        "env_var": "JECH_FOCUS_MODE",
        "description": "Focus mode for motorized autofocus cameras ('fixed'/'manual' locks hyperfocal to eliminate vibration hunting)."
    },
    {
        "name": "CAM_LENS_POSITION",
        "category": "Camera & Vision",
        "default": 0.5,
        "type": "float",
        "unit": "dioptres",
        "env_var": "JECH_LENS_POSITION",
        "description": "Manual lens focus position in dioptres (0.0=infinity, 0.5=2m hyperfocal, 1.0=1m)."
    },
    {
        "name": "CAM_VCM_FOCUS_ABS",
        "category": "Camera & Vision",
        "default": 160,
        "type": "int",
        "unit": "val",
        "env_var": "JECH_VCM_FOCUS_ABS",
        "description": "Hardware VCM focus_absolute register on /dev/v4l-subdev (160=optimal hyperfocal for 64MP OwlSight)."
    },

    # -------------------------------------------------------------------------
    # 3. VIDEO STREAMING & RECORDING
    # -------------------------------------------------------------------------
    {
        "name": "RTSP_STREAM_ENABLED",
        "category": "Streaming & Recording",
        "default": True,
        "type": "bool",
        "env_var": "JECH_RTSP_ENABLED",
        "description": "Enable low-latency RTSP live streaming for GCS/pilot HUD via MediaMTX."
    },
    {
        "name": "RTSP_URL",
        "category": "Streaming & Recording",
        "default": "rtsp://127.0.0.1:8554/cam",
        "type": "string",
        "env_var": "JECH_RTSP_URL",
        "description": "RTSP publish URL for MediaMTX server."
    },
    {
        "name": "RTSP_FPS",
        "category": "Streaming & Recording",
        "default": 30,
        "type": "int",
        "unit": "FPS",
        "env_var": "JECH_RTSP_FPS",
        "description": "Framerate for live RTSP HUD telemetry stream."
    },
    {
        "name": "RTSP_BITRATE",
        "category": "Streaming & Recording",
        "default": "2000k",
        "type": "string",
        "unit": "bps",
        "env_var": "JECH_RTSP_BITRATE",
        "description": "Target video encoding bitrate for RTSP stream (e.g., 2000k for 4G/5G cellular link)."
    },
    {
        "name": "RECORD_ON_ARM_ENABLED",
        "category": "Streaming & Recording",
        "default": True,
        "type": "bool",
        "env_var": "JECH_RECORD_ENABLED",
        "description": "Automatically record onboard high-definition video during armed flight."
    },
    {
        "name": "RECORD_FPS",
        "category": "Streaming & Recording",
        "default": 30,
        "type": "int",
        "unit": "FPS",
        "env_var": "PL_RECORD_FPS",
        "description": "Framerate for on-disk MP4 flight video recordings."
    },
    {
        "name": "RECORDINGS_DIR",
        "category": "Streaming & Recording",
        "default": "recordings",
        "type": "string",
        "unit": "path",
        "env_var": "JECH_RECORDINGS_DIR",
        "description": "Directory where date-stamped MP4 flight logs are stored."
    },

    # -------------------------------------------------------------------------
    # 4. SERVO CONTROLS (ST3215 & SC09 BUS)
    # -------------------------------------------------------------------------
    {
        "name": "SERVO_CONTROLLER_ENABLED",
        "category": "Servo Controls",
        "default": True,
        "type": "bool",
        "env_var": "JECH_SERVO_ENABLED",
        "description": "Enable serial bus servo controller monitoring and positioning."
    },
    {
        "name": "SERVO_SERIAL_PORT",
        "category": "Servo Controls",
        "default": "auto",
        "type": "string",
        "unit": "port",
        "env_var": "JECH_SERVO_PORT",
        "description": "Serial port device path for ST3215/SC09 servo driver board (or 'auto' to probe common USB ports)."
    },
    {
        "name": "SERVO_BAUDRATE",
        "category": "Servo Controls",
        "default": 1000000,
        "type": "int",
        "unit": "baud",
        "env_var": "JECH_SERVO_BAUD",
        "description": "Communication baud rate for serial bus servos (standard: 1000000)."
    },
    {
        "name": "SERVO_LOCK_PWM",
        "category": "Servo Controls",
        "default": 1000,
        "type": "int",
        "unit": "us",
        "env_var": "JECH_SERVO_LOCK_PWM",
        "description": "Initial startup PWM pulse width for MAVLink Servo Channel 16 (1000 = locked/low)."
    },
    {
        "name": "SERVO_UNLOCK_PWM",
        "category": "Servo Controls",
        "default": 2000,
        "type": "int",
        "unit": "us",
        "env_var": "JECH_SERVO_UNLOCK_PWM",
        "description": "PWM pulse width commanded to unlock/release payload (2000 = unlocked/high)."
    },
    {
        "name": "SERVO_1_LOCK_POS",
        "category": "Servo Controls",
        "default": 1450,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_1_LOCK_POS",
        "description": "Servo ID 1 (ST3215 Lid 1) target position for LOCKED state (0-4095)."
    },
    {
        "name": "SERVO_1_UNLOCK_POS",
        "category": "Servo Controls",
        "default": 4000,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_1_UNLOCK_POS",
        "description": "Servo ID 1 (ST3215 Lid 1) target position for UNLOCKED state (0-4095)."
    },
    {
        "name": "SERVO_2_LOCK_POS",
        "category": "Servo Controls",
        "default": 3450,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_2_LOCK_POS",
        "description": "Servo ID 2 (ST3215 Lid 2) target position for LOCKED state (0-4095)."
    },
    {
        "name": "SERVO_2_UNLOCK_POS",
        "category": "Servo Controls",
        "default": 500,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_2_UNLOCK_POS",
        "description": "Servo ID 2 (ST3215 Lid 2) target position for UNLOCKED state (0-4095)."
    },
    {
        "name": "SERVO_3_LOCK_POS",
        "category": "Servo Controls",
        "default": 520,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_3_LOCK_POS",
        "description": "Servo ID 3 (SC09 Latch 3 / Kadi) target position for LOCKED state (0-1023)."
    },
    {
        "name": "SERVO_3_UNLOCK_POS",
        "category": "Servo Controls",
        "default": 675,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_3_UNLOCK_POS",
        "description": "Servo ID 3 (SC09 Latch 3 / Kadi) target position for UNLOCKED state (0-1023)."
    },
    {
        "name": "SERVO_4_LOCK_POS",
        "category": "Servo Controls",
        "default": 700,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_4_LOCK_POS",
        "description": "Servo ID 4 (SC09 Latch 4 / Kadi) target position for LOCKED state (0-1023)."
    },
    {
        "name": "SERVO_4_UNLOCK_POS",
        "category": "Servo Controls",
        "default": 550,
        "type": "int",
        "unit": "ticks",
        "env_var": "JECH_SERVO_4_UNLOCK_POS",
        "description": "Servo ID 4 (SC09 Latch 4 / Kadi) target position for UNLOCKED state (0-1023)."
    },
    {
        "name": "SERVO_ST_SPEED",
        "category": "Servo Controls",
        "default": 2400,
        "type": "int",
        "unit": "steps/s",
        "env_var": "JECH_SERVO_ST_SPEED",
        "description": "Movement speed for ST3215 Dual Lid Servos (ID 1 & 2)."
    },
    {
        "name": "SERVO_SC_SPEED",
        "category": "Servo Controls",
        "default": 1500,
        "type": "int",
        "unit": "steps/s",
        "env_var": "JECH_SERVO_SC_SPEED",
        "description": "Movement speed for SC09 Latch Servos (ID 3 & 4)."
    },

    # -------------------------------------------------------------------------
    # 5. MAVLINK TELEMETRY & FLIGHT CONTROLLER
    # -------------------------------------------------------------------------
    {
        "name": "MAVLINK_CONNECT_URI",
        "category": "MAVLink & Telemetry",
        "default": "udpin:127.0.0.1:14555",
        "type": "string",
        "env_var": "JECH_MAVLINK_CONNECT",
        "description": "Connection string for ArduPilot flight controller (e.g., udpin:127.0.0.1:14555 or /dev/ttyUSB0)."
    },
    {
        "name": "MAVLINK_SERIAL_BAUD",
        "category": "MAVLink & Telemetry",
        "default": 921600,
        "type": "int",
        "unit": "baud",
        "env_var": "JECH_MAVLINK_BAUD",
        "description": "Baud rate when connecting to flight controller via serial UART (default: 921600)."
    },
    {
        "name": "MAVLINK_FORCE_V2",
        "category": "MAVLink & Telemetry",
        "default": True,
        "type": "bool",
        "env_var": "MAVLINK20",
        "description": "Force MAVLink 2.0 dialect protocol (strictly required for 14-field 3D LANDING_TARGET messages)."
    },
    {
        "name": "DRONE_ID",
        "category": "MAVLink & Telemetry",
        "default": "Victoris",
        "type": "string",
        "env_var": "JECH_DRONE_ID",
        "description": "Unique drone callsign/identifier registered in Firebase RTDB and telemetry dashboards."
    },
    {
        "name": "TELEMETRY_INTERVAL_SEC",
        "category": "MAVLink & Telemetry",
        "default": 5.0,
        "type": "float",
        "unit": "s",
        "env_var": "JECH_TELEMETRY_INTERVAL",
        "description": "Interval (seconds) between periodic status updates dispatched to Firebase Cloud Database."
    },
    {
        "name": "FIREBASE_DB_URL",
        "category": "MAVLink & Telemetry",
        "default": "https://jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app",
        "type": "string",
        "env_var": "JECH_FIREBASE_URL",
        "description": "Firebase Realtime Database endpoint for remote mission dispatch and telemetry sync."
    },
    {
        "name": "DRONE_DRY_WEIGHT_KG",
        "category": "MAVLink & Telemetry",
        "default": 5.5,
        "type": "float",
        "unit": "kg",
        "env_var": "JECH_DRONE_DRY_WEIGHT",
        "description": "Empty dry weight of the hexacopter in kilograms used for battery endurance and payload calculations."
    },
    {
        "name": "USE_DOCK_INTERLOCK",
        "category": "MAVLink & Telemetry",
        "default": False,
        "type": "bool",
        "env_var": "JECH_USE_DOCK",
        "description": "Require physical docking station latch interlock before granting mission clearance or disarming."
    },

    # -------------------------------------------------------------------------
    # 6. SYSTEM SERVICES & LOGGING
    # -------------------------------------------------------------------------
    {
        "name": "FC_LOG_PORT",
        "category": "System Services",
        "default": 8765,
        "type": "int",
        "unit": "port",
        "env_var": "JECH_FC_LOG_PORT",
        "description": "HTTP port hosting WebTools, DataFlash Log Finder, and the Codebase Parameter Manager."
    },
    {
        "name": "FC_LOG_CACHE_DIR",
        "category": "System Services",
        "default": "downloaded_logs",
        "type": "string",
        "unit": "path",
        "env_var": "JECH_FC_LOG_CACHE_DIR",
        "description": "Local directory where binary .bin DataFlash logs from the flight controller are saved."
    },

    # -------------------------------------------------------------------------
    # 7. LOG TRANSFER & DOWNLOADING
    # -------------------------------------------------------------------------
    {
        "name": "FC_LOG_DEFAULT_METHOD",
        "category": "Log Transfer Options",
        "default": "qgc",
        "type": "string",
        "options": ["qgc", "mavftp"],
        "env_var": "JECH_FC_LOG_DEFAULT_METHOD",
        "description": "Default download protocol for DataFlash logs: 'qgc' (QGC Fast streaming, 400-600 KB/s) or 'mavftp' (MAVLink FTP, ~100 KB/s)."
    },
    {
        "name": "FC_LOG_PAUSE_TELEMETRY",
        "category": "Log Transfer Options",
        "default": True,
        "type": "bool",
        "env_var": "JECH_FC_LOG_PAUSE_TELEMETRY",
        "description": "Pause vehicle MAVLink telemetry stream requests during download to give maximum link bandwidth to log packets."
    },
    {
        "name": "FC_LOG_DOWNLOAD_TIMEOUT",
        "category": "Log Transfer Options",
        "default": 7200,
        "type": "int",
        "unit": "s",
        "env_var": "JECH_FC_LOG_DOWNLOAD_TIMEOUT_SEC",
        "description": "Maximum safety wall-clock seconds allowed for downloading a single .BIN flight log (default: 7200s / 2 hours)."
    },
    {
        "name": "FC_LOG_QGC_CHUNK_BINS",
        "category": "Log Transfer Options",
        "default": 512,
        "type": "int",
        "unit": "bins",
        "options": [128, 256, 512, 1024, 2048, 4096],
        "env_var": "JECH_FC_LOG_QGC_CHUNK_BINS",
        "description": "Block size for QGC fast streaming in 90-byte LOG_DATA bins (512 bins = 46 KB standard QGC block)."
    },
    {
        "name": "FC_LOG_QGC_RETRY_SEC",
        "category": "Log Transfer Options",
        "default": 0.2,
        "type": "float",
        "unit": "s",
        "env_var": "JECH_FC_LOG_QGC_RETRY_SEC",
        "description": "Silence timeout before QGC fast download sends a retry request for missing chunks."
    },
    {
        "name": "FC_LOG_FTP_MAX_BACKLOG",
        "category": "Log Transfer Options",
        "default": 160,
        "type": "int",
        "unit": "packets",
        "env_var": "JECH_FC_LOG_FTP_MAX_BACKLOG",
        "description": "MAVFTP maximum in-flight unacknowledged packets (backlog window). Default 160 allows high throughput on low-latency links."
    },
    {
        "name": "FC_LOG_FTP_BURST_SIZE",
        "category": "Log Transfer Options",
        "default": 239,
        "type": "int",
        "unit": "bytes",
        "env_var": "JECH_FC_LOG_FTP_BURST_SIZE",
        "description": "MAVFTP payload burst size in bytes per packet (max standard MAVLink FTP payload is 239 bytes)."
    },
    {
        "name": "FC_LOG_FTP_IDLE_DETECTION",
        "category": "Log Transfer Options",
        "default": 3.5,
        "type": "float",
        "unit": "s",
        "env_var": "JECH_FC_LOG_FTP_IDLE_DETECTION",
        "description": "MAVFTP inactivity timeout in seconds. If no packet is received or sent for this time, timeout is triggered."
    },
    {
        "name": "FC_LOG_FTP_READ_RETRY",
        "category": "Log Transfer Options",
        "default": 1.0,
        "type": "float",
        "unit": "s",
        "env_var": "JECH_FC_LOG_FTP_READ_RETRY",
        "description": "MAVFTP burst read timeout before re-issuing a missing burst read request (seconds)."
    },
    {
        "name": "FC_LOG_FTP_RETRY",
        "category": "Log Transfer Options",
        "default": 0.5,
        "type": "float",
        "unit": "s",
        "env_var": "JECH_FC_LOG_FTP_RETRY",
        "description": "MAVFTP command response retry interval in seconds (open, terminate, reset session, etc.)."
    }
]


class ConfigManager:
    """
    Thread-safe configuration manager that persists parameters to codebase_params.json,
    synchronizes with os.environ, and provides structured schema metadata for WebTools.
    """
    def __init__(self, config_file: Optional[Path] = None):
        self.config_file = config_file or CONFIG_FILE
        self._lock = threading.Lock()
        self._definitions: Dict[str, Dict[str, Any]] = {p["name"]: dict(p) for p in PARAM_DEFINITIONS}
        self._values: Dict[str, Any] = {}
        self._listeners: List[Any] = []
        self.load()

    def subscribe(self, callback: Any) -> None:
        """Register a callback function that is invoked when parameters change.
        Callback signature: callback(changed_params_dict)"""
        with self._lock:
            if callback not in self._listeners:
                self._listeners.append(callback)

    def unsubscribe(self, callback: Any) -> None:
        """Unregister a listener callback."""
        with self._lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    def _notify_listeners_locked(self, changes: Dict[str, Any]) -> None:
        """Invoke all registered callbacks with modified parameters."""
        for cb in list(self._listeners):
            try:
                cb(changes)
            except Exception as e:
                print(f"[CONFIG] Listener callback error: {e}")

    def load(self) -> Dict[str, Any]:
        """Loads configuration from disk, merges with defaults and environment variables."""
        with self._lock:
            # 1. Start with defaults
            for name, meta in self._definitions.items():
                val = meta["default"]
                # Check environment variable if defined
                env_k = meta.get("env_var")
                if env_k and env_k in os.environ:
                    env_val = os.environ[env_k]
                    val = self._cast_value(env_val, meta["type"])
                self._values[name] = val

            # 2. If config file exists on disk, read and merge saved values
            if self.config_file.exists():
                try:
                    with self.config_file.open("r", encoding="utf-8") as f:
                        data = json.load(f)
                    if isinstance(data, dict):
                        saved_params = data.get("parameters", data)
                        for k, v in saved_params.items():
                            if k in self._definitions:
                                self._values[k] = self._cast_value(v, self._definitions[k]["type"])
                except Exception as e:
                    print(f"[CONFIG] Warning: Could not read {self.config_file}: {e}")
            else:
                # Save initial default file
                self._save_locked()

            # 3. Synchronize environment variables for all components
            self._sync_env_locked()
            return dict(self._values)

    def _cast_value(self, raw: Any, val_type: str) -> Any:
        """Type cast helper with safe fallbacks."""
        if raw is None:
            return None
        try:
            if val_type == "int":
                return int(float(raw))
            elif val_type == "float":
                return float(raw)
            elif val_type == "bool":
                if isinstance(raw, bool):
                    return raw
                return str(raw).strip().lower() in ("1", "true", "yes", "on", "t")
            elif val_type in ("string", "choice"):
                return str(raw)
            return raw
        except Exception:
            return raw

    def _save_locked(self) -> None:
        """Saves current parameters with full metadata to disk (caller must hold _lock)."""
        try:
            self.config_file.parent.mkdir(parents=True, exist_ok=True)
            export_data = {
                "_version": "1.0",
                "_description": "Full Parameter Settings for Precision-Land Codebase",
                "parameters": dict(self._values)
            }
            with self.config_file.open("w", encoding="utf-8") as f:
                json.dump(export_data, f, indent=2)
        except Exception as e:
            print(f"[CONFIG] Error saving {self.config_file}: {e}")

    def _sync_env_locked(self) -> None:
        """Syncs all parameters into os.environ for instant backward compatibility."""
        for name, meta in self._definitions.items():
            env_k = meta.get("env_var")
            if env_k and name in self._values:
                val = self._values[name]
                if meta["type"] == "bool":
                    os.environ[env_k] = "1" if val else "0"
                else:
                    os.environ[env_k] = str(val)

    def get(self, name: str, fallback: Any = None) -> Any:
        """Get the current value of a parameter."""
        with self._lock:
            if name in self._values:
                return self._values[name]
            if name in self._definitions:
                return self._definitions[name]["default"]
            return fallback

    def set(self, name: str, value: Any) -> bool:
        """Set a single parameter and persist."""
        with self._lock:
            if name not in self._definitions:
                return False
            self._values[name] = self._cast_value(value, self._definitions[name]["type"])
            self._save_locked()
            self._sync_env_locked()
            self._notify_listeners_locked({name: self._values[name]})
            return True

    def update_bulk(self, changes: Dict[str, Any]) -> Dict[str, Any]:
        """Update multiple parameters simultaneously and save to disk."""
        updated = {}
        with self._lock:
            for k, raw_v in changes.items():
                if k in self._definitions:
                    casted = self._cast_value(raw_v, self._definitions[k]["type"])
                    self._values[k] = casted
                    updated[k] = casted
            self._save_locked()
            self._sync_env_locked()
            if updated:
                self._notify_listeners_locked(updated)
        print(f"[CONFIG] Updated {len(updated)} parameters to {self.config_file}")
        return updated

    def set_many(self, changes: Dict[str, Any]) -> Dict[str, Any]:
        """Alias for update_bulk."""
        return self.update_bulk(changes)

    def reset_defaults(self, param_names: Optional[List[str]] = None) -> Dict[str, Any]:
        """Reset all or specified parameters to default values and persist."""
        reset_changes = {}
        with self._lock:
            if param_names:
                for name in param_names:
                    if name in self._definitions:
                        self._values[name] = self._definitions[name]["default"]
                        reset_changes[name] = self._values[name]
            else:
                for name, meta in self._definitions.items():
                    self._values[name] = meta["default"]
                    reset_changes[name] = self._values[name]
            self._save_locked()
            self._sync_env_locked()
            if reset_changes:
                self._notify_listeners_locked(reset_changes)
        print(f"[CONFIG] Reset parameters to defaults.")
        return dict(self._values)

    def reset_to_defaults(self, param_names: Optional[List[str]] = None) -> Dict[str, Any]:
        """Alias for reset_defaults."""
        return self.reset_defaults(param_names)

    def get_full_catalog(self) -> List[Dict[str, Any]]:
        """Returns the full parameter list including current values, defaults, types, units, and descriptions."""
        with self._lock:
            catalog = []
            for name, meta in self._definitions.items():
                curr_val = self._values.get(name, meta["default"])
                item = dict(meta)
                item["value"] = curr_val
                item["modified"] = (curr_val != meta["default"])
                catalog.append(item)
            return catalog

    def get_categories(self) -> List[str]:
        """Returns unique categories in presentation order."""
        seen = []
        for p in PARAM_DEFINITIONS:
            cat = p["category"]
            if cat not in seen:
                seen.append(cat)
        return seen


# Global singleton instance
config_manager = ConfigManager()
