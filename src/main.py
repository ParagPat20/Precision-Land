"""
Precision Landing System with Firebase Remote Dispatch Integration

NOTE: Call it with Mavlink 2.0 as follows:
MAVLINK20=1 python test_.....py

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

FIREBASE INTEGRATION:
-------------------
This system integrates with Firebase Realtime Database for remote mission dispatch and control.

Mission Flow:
1. Flutter app sends mission command with status 'PENDING' to Firebase
2. This script listens for PENDING commands and starts mission execution
3. Status updates: PENDING -> IN_PROGRESS -> COMPLETED/FAILED/ABORTED

Abort Handling:
- Flutter app can send ABORT_REQUESTED status at any time
- Firebase listener detects ABORT_REQUESTED in real-time
- Abort works even before mission starts or during execution
- On abort: switches to RTL mode and updates status to ABORTED

Status Management:
- PENDING: New mission received, ready to process
- IN_PROGRESS: Mission is executing
- COMPLETED: Mission finished successfully
- FAILED: Mission execution failed
- ABORTED: Mission was aborted (either by user or system)
- ABORT_REQUESTED: User requested abort (transient state)
- EXPIRED_STALE: Mission command was too old (>45s)

"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse
import os
import traceback
import glob
from collections import deque  # For rolling stability buffer
from datetime import datetime

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import numpy as np
from opencv.lib_aruco_pose import *
from led_controller import DroneLEDController
from fc_log_service import start_log_services
import threading
import firebase_admin
from firebase_admin import credentials, db
from core.mission_generator import DeliveryTemplate, LatLng

# Helper function to format timestamp for debugging
def format_timestamp(timestamp_ms):
    """Convert milliseconds timestamp to readable format HH:MM:SS DD/MM/YY"""
    dt = datetime.fromtimestamp(timestamp_ms / 1000.0)
    return dt.strftime("%H:%M:%S %d/%m/%y")

# --- Configuration ---
DRONE_ID = "Victoris"
# Path to serviceAccountKey.json (Assuming it's in the same folder as this script or adjusted path)
# Since this script is in rpi_mission_manager/Precision-Land/src/, and key is in rpi_mission_manager/
# We need to point up two levels.
CREDENTIALS_PATH = os.path.join(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))), "serviceAccountKey.json")
DATABASE_URL = "https://jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app"
TELEMETRY_INTERVAL_SEC = 5

# --- Firebase & Mission Logic ---
firebase_initialized = False
abort_requested = False  # Global abort flag to stop mission execution
active_mission_ref = None  # Reference to current mission command in Firebase
mission_active = False  # Track if a mission is currently active (IN_PROGRESS)
latest_battery_voltage = 0.0
latest_battery_current = 0.0
latest_battery_remaining = -1

def build_telemetry_payload():
    """Build the latest telemetry snapshot for Firebase consumers."""
    loc = vehicle.location.global_relative_frame
    if loc is None or loc.lat is None or loc.lon is None:
        return None

    # Safe extraction of battery data
    batt_voltage = latest_battery_voltage
    batt_cur = latest_battery_current
    batt_remaining = latest_battery_remaining
    if vehicle.battery is not None:
        if vehicle.battery.voltage is not None:
            batt_voltage = float(vehicle.battery.voltage)
        if vehicle.battery.current is not None:
            batt_cur = float(vehicle.battery.current)
        if vehicle.battery.level is not None:
            batt_remaining = int(vehicle.battery.level)

    return {
        'lat': loc.lat,
        'lng': loc.lon,
        'alt': float(loc.alt) if loc.alt is not None else 0.0,
        'heading': float(vehicle.heading) if vehicle.heading is not None else 0.0,
        'mode': vehicle.mode.name if vehicle.mode is not None else 'UNKNOWN',
        'batteryVoltage': batt_voltage,
        'current': batt_cur,
        'updated_at': int(time.time() * 1000)
    }

def telemetry_loop(cmd_ref):
    """
    Sends telemetry updates to Firebase while mission is active.
    Also listens for ABORT signals.
    Only sends GPS/telemetry data when mission status is IN_PROGRESS.
    Runs regardless of arming status once mission is uploaded.
    """
    global abort_requested, mission_active
    print("[FIREBASE DEBUG] Starting Telemetry Loop...")
    mission_active = True  # Mark mission as active when telemetry loop starts
    
    # Continue sending telemetry while mission is active (not just when armed)
    while mission_active and not abort_requested:
        # 1. Check for Abort (check both global flag and Firebase status)
        try:
            current_status = cmd_ref.child('status').get()
            print(f"[FIREBASE DEBUG] Telemetry loop - current status: {current_status}")
            if current_status == 'ABORT_REQUESTED' or abort_requested:
                print("[ABORT] Received Abort Request! Stopping Mission...")
                abort_requested = True
                mission_active = False  # Stop telemetry immediately
                # Switch to RTL mode for safe return
                try:
                    vehicle.mode = VehicleMode("RTL")
                    print("[ABORT] Vehicle mode set to RTL")
                except Exception as e:
                    print(f"[ABORT] Error setting RTL mode: {e}")
                cmd_ref.update({
                    'status': 'ABORTED',
                    'aborted_at': int(time.time() * 1000)
                })
                print("[ABORT] Mission aborted, returning to launch...")
                return
            
            # Only send telemetry if mission is IN_PROGRESS
            if current_status != 'IN_PROGRESS':
                print(f"[FIREBASE DEBUG] Mission status is '{current_status}', not IN_PROGRESS. Stopping telemetry.")
                mission_active = False
                time.sleep(2)
                continue
                
        except Exception as e:
            print(f"[FIREBASE DEBUG] Abort Check Error: {e}")

        # 2. Send Telemetry (only when mission is active)
        if mission_active:
            try:
                loc = vehicle.location.global_relative_frame
                telemetry = build_telemetry_payload()
                if telemetry is not None:
                    cmd_ref.child('telemetry').update(telemetry)
                    print(
                        f"[FIREBASE DEBUG] [OK] Telemetry sent: "
                        f"lat={loc.lat:.6f}, lng={loc.lon:.6f}, alt={loc.alt:.1f}m, "
                        f"heading={vehicle.heading}, mode={vehicle.mode.name}, "
                        f"batt={telemetry['batteryVoltage']:.2f}V, current={telemetry['current']:.2f}A"
                    )
                else:
                    print("[FIREBASE DEBUG] Location data not available, skipping telemetry send")
            except Exception as e:
                print(f"[FIREBASE DEBUG] Telemetry Error: {e}")
                traceback.print_exc()
            
        time.sleep(TELEMETRY_INTERVAL_SEC)
    
    mission_active = False  # Mark mission as inactive when loop exits
    print("[FIREBASE DEBUG] Telemetry loop ended")

def execute_mission_logic(mission_items, cmd_ref):
    """
    Executes the mission logic: uploads mission, sets mode, arms, and starts telemetry.
    Checks for abort requests throughout the process.
    """
    global abort_requested
    print("[FIREBASE DEBUG] Uploading mission via DroneKit...")
    try:
        # Check for abort before starting
        if abort_requested:
            print("[FIREBASE DEBUG] [ABORT] Abort requested before mission upload. Cancelling...")
            return False
            
        cmds = vehicle.commands
        cmds.clear()
        
        for item in mission_items:
            if abort_requested:
                print("[ABORT] Abort requested during mission upload. Cancelling...")
                return False
            cmd = Command(
                0, 0, 0, 
                item.frame,
                item.command_id,
                item.current, item.autocontinue,
                item.param1, item.param2, item.param3, item.param4,
                item.x, item.y, item.z
            )
            cmds.add(cmd)
            
        cmds.upload()
        print(f"[FIREBASE DEBUG] Mission of {len(mission_items)} items Uploaded!")
        
        # Start telemetry loop immediately after mission upload (regardless of arming status)
        # This allows tracking mission progress even before arming
        print("[FIREBASE DEBUG] Starting telemetry transmission to Firebase...")
        telemetry_thread = threading.Thread(target=telemetry_loop, args=(cmd_ref,), name="TelemetryThread", daemon=True)
        telemetry_thread.start()
        
        # Check for abort before setting mode
        if abort_requested:
            print("[FIREBASE DEBUG] [ABORT] Abort requested after mission upload. Cancelling...")
            return False
        
        print("[FIREBASE DEBUG] Setting Mode to AUTO...")
        vehicle.mode = VehicleMode("AUTO")
        
        # Check for abort before arming
        if abort_requested:
            print("[FIREBASE DEBUG] [ABORT] Abort requested before arming. Cancelling...")
            return False
        
        print("[FIREBASE DEBUG] Arming...")
        vehicle.armed = True
        
        # Wait up to 5s for arming, checking for abort during wait
        for _ in range(5):
            if abort_requested:
                print("[FIREBASE DEBUG] [ABORT] Abort requested during arming wait. Switching to RTL...")
                try:
                    vehicle.mode = VehicleMode("RTL")
                    print("[ABORT] Vehicle mode set to RTL")
                except Exception as e:
                    print(f"[ABORT] Error setting RTL mode: {e}")
                return False
            if vehicle.armed:
                break
            time.sleep(1)
            
        print(f"[FIREBASE DEBUG] ARMED Status: {vehicle.armed}")
        
        # Final check for abort after arming wait
        if abort_requested:
            print("[FIREBASE DEBUG] [ABORT] Abort requested. Switching to RTL...")
            try:
                vehicle.mode = VehicleMode("RTL")
                print("[ABORT] Vehicle mode set to RTL")
            except Exception as e:
                print(f"[ABORT] Error setting RTL mode: {e}")
            return False
            
        return True
    except Exception as e:
        print(f"[FIREBASE DEBUG] Mission Execution Error: {e}")
        traceback.print_exc()
        return False

def run_mission_thread(command):
    """
    Runs a mission in a separate thread. Handles mission execution and status updates.
    """
    global abort_requested, active_mission_ref, mission_active
    cmd_id = command.get('id')
    timestamp = command.get('timestamp', 0)
    current_time = int(time.time() * 1000)
    
    print(f"[FIREBASE DEBUG] ========== NEW MISSION RECEIVED ==========")
    print(f"[FIREBASE DEBUG] Mission ID: {cmd_id}")
    print(f"[FIREBASE DEBUG] Timestamp: {timestamp} ({format_timestamp(timestamp)})")
    print(f"[FIREBASE DEBUG] Current Time: {current_time} ({format_timestamp(current_time)})")
    print(f"[FIREBASE DEBUG] Age: {(current_time - timestamp)/1000:.1f} seconds")
    
    ref = db.reference(f'missions/{DRONE_ID}/active_command')
    active_mission_ref = ref
    mission_active = False  # Reset mission active flag
    
    # 1. Stale Command Check (45 seconds = 45000 ms)
    if current_time - timestamp > 45000:
        print(f"[FIREBASE DEBUG] [REJECT] Mission {cmd_id} is STALE ({(current_time - timestamp)/1000:.1f}s old). Ignoring.")
        mission_active = False
        ref.update({'status': 'EXPIRED_STALE'})
        active_mission_ref = None
        return

    # 2. Check for abort before processing
    try:
        current_status = ref.child('status').get()
        print(f"[FIREBASE DEBUG] Current mission status in Firebase: {current_status}")
        if current_status == 'ABORT_REQUESTED':
            print(f"[FIREBASE DEBUG] [ABORT] Mission {cmd_id} was aborted before processing.")
            mission_active = False
            ref.update({'status': 'ABORTED', 'aborted_at': int(time.time() * 1000)})
            active_mission_ref = None
            return
    except Exception as e:
        print(f"[FIREBASE DEBUG] Status check error: {e}")

    print(f"[FIREBASE DEBUG] [{threading.current_thread().name}] Processing Mission {cmd_id}")
    payload = command.get('payload', {})
    print(f"[FIREBASE DEBUG] Payload: {payload}")
    
    target_lat = payload.get('target_lat')
    target_lng = payload.get('target_lng')
    
    if target_lat is None or target_lng is None:
        print("[FIREBASE DEBUG] [ERROR] Invalid Target Location - missing lat/lng")
        mission_active = False
        ref.update({'status': 'FAILED', 'error': 'Invalid target location'})
        active_mission_ref = None
        return

    print(f"[FIREBASE DEBUG] Target Location: ({target_lat}, {target_lng})")
    
    # Generate Mission
    print("[FIREBASE DEBUG] Generating Mission...")
    try:
        template = DeliveryTemplate.get_default_template()
        # Get actual home location from vehicle
        if vehicle.location.global_frame.lat is not None:
            home_loc = LatLng(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
            print(f"[FIREBASE DEBUG] Home Location: ({home_loc.latitude}, {home_loc.longitude})")
        else:
            home_loc = LatLng(0.0, 0.0)
            print("[FIREBASE DEBUG] [WARNING] Vehicle location not available, using (0,0)")
             
        target_loc = LatLng(target_lat, target_lng)
        
        mission_items = template.generate_mission(
            home_location=home_loc,
            delivery_location=target_loc,
            override_values=payload
        )
        print(f"[FIREBASE DEBUG] Mission generated with {len(mission_items)} waypoints")
    except Exception as e:
        print(f"[FIREBASE DEBUG] [ERROR] Mission generation error: {e}")
        import traceback
        traceback.print_exc()
        mission_active = False
        ref.update({'status': 'FAILED', 'error': str(e)})
        active_mission_ref = None
        return
    
    # Update status to IN_PROGRESS
    print("[FIREBASE DEBUG] Updating Firebase status to IN_PROGRESS...")
    ref.update({'status': 'IN_PROGRESS', 'started_at': int(time.time() * 1000)})
    mission_active = True  # Mark mission as active
    print("[FIREBASE DEBUG] Status updated to IN_PROGRESS")
    
    # Execute mission (this will check for abort internally)
    success = execute_mission_logic(mission_items, ref)

    # Check final status
    try:
        final_status = ref.child('status').get()
        print(f"[FIREBASE DEBUG] Final mission status: {final_status}")
        if final_status == 'ABORTED':
            print(f"[FIREBASE DEBUG] Mission {cmd_id} ended (ABORTED).")
            mission_active = False
            active_mission_ref = None
            return
        elif success:
            mission_active = False  # Stop telemetry when mission completes
            ref.update({
                'status': 'COMPLETED',
                'completed_at': int(time.time() * 1000)
            })
            print(f"[FIREBASE DEBUG] Mission {cmd_id} COMPLETED")
        else:
            mission_active = False  # Stop telemetry when mission fails
            ref.update({
                'status': 'FAILED',
                'failed_at': int(time.time() * 1000)
            })
            print(f"[FIREBASE DEBUG] Mission {cmd_id} FAILED")
    except Exception as e:
        print(f"[FIREBASE DEBUG] Status update error: {e}")
        traceback.print_exc()
    finally:
        mission_active = False
        active_mission_ref = None
        print("[FIREBASE DEBUG] ========== MISSION ENDED ==========")

def check_mission(command):
    """
    Checks incoming Firebase commands and handles them appropriately.
    Handles PENDING missions and ABORT_REQUESTED status changes.
    """
    global abort_requested, active_mission_ref, mission_active
    
    if not command or not isinstance(command, dict):
        print("[FIREBASE DEBUG] check_mission called with invalid command (None or not dict)")
        return
    
    status = command.get('status')
    print(f"[FIREBASE DEBUG] check_mission called - Status: {status}")
    
    # Handle abort requests (can come at any time)
    if status == 'ABORT_REQUESTED':
        print("[FIREBASE DEBUG] [ABORT] Abort request received from Firebase!")
        abort_requested = True
        mission_active = False  # Stop sending telemetry immediately
        
        # If there's an active mission, update its status
        if active_mission_ref:
            try:
                active_mission_ref.update({
                    'status': 'ABORTED',
                    'aborted_at': int(time.time() * 1000)
                })
                print("[FIREBASE DEBUG] Updated active mission status to ABORTED")
            except Exception as e:
                print(f"[FIREBASE DEBUG] Error updating abort status: {e}")
        
        # Always try to switch to RTL mode (regardless of armed status)
        try:
            print("[FIREBASE DEBUG] [ABORT] Switching vehicle to RTL mode...")
            vehicle.mode = VehicleMode("RTL")
            print("[ABORT] Vehicle mode set to RTL")
        except Exception as e:
            print(f"[FIREBASE DEBUG] Error switching to RTL: {e}")
        
        return
    
    # Handle new PENDING missions
    if status == 'PENDING':
        # Reset abort flag and mission active flag for new mission
        abort_requested = False
        mission_active = False
        sender = command.get('sender_email', 'Unknown')
        print(f"[FIREBASE DEBUG] ========================================")
        print(f"[FIREBASE DEBUG] Received PENDING Mission from {sender}")
        print(f"[FIREBASE DEBUG] Command data: {command}")
        print(f"[FIREBASE DEBUG] ========================================")
        t = threading.Thread(target=run_mission_thread, args=(command,), name="MissionThread")
        t.start()
        print(f"[FIREBASE DEBUG] Mission thread started: {t.name}")

def firebase_listener_thread():
    """
    Firebase listener thread that connects to Firebase and listens for mission commands.
    Handles both initial state and real-time updates.
    """
    global firebase_initialized
    print("[FIREBASE DEBUG] Attempting to connect to Firebase (Background Thread)...")
    start_time = time.time()
    
    # Pre-flight checks
    print(f"[FIREBASE DEBUG] [CHECK] Verifying service account key exists: {CREDENTIALS_PATH}")
    if not os.path.exists(CREDENTIALS_PATH):
        print(f"[FIREBASE DEBUG] [ERROR] Service account key NOT FOUND at: {CREDENTIALS_PATH}")
        print("[FIREBASE DEBUG] [ERROR] Cannot initialize Firebase without credentials!")
        return
    else:
        file_size = os.path.getsize(CREDENTIALS_PATH)
        print(f"[FIREBASE DEBUG] [CHECK] Service account key found ({file_size} bytes)")
    
    initial_command_fetch_started = False

    while True:
        try:
            if not firebase_admin._apps:
                print(f"[FIREBASE DEBUG] [TIMING] Starting Firebase initialization... (elapsed: {time.time() - start_time:.1f}s)")
                print(f"[FIREBASE DEBUG] [TIMING] Loading credentials from: {CREDENTIALS_PATH}")
                cred_start = time.time()
                cred = credentials.Certificate(CREDENTIALS_PATH)
                print(f"[FIREBASE DEBUG] [TIMING] Credentials loaded in {time.time() - cred_start:.2f}s")
                
                print(f"[FIREBASE DEBUG] [TIMING] Initializing Firebase app with database: {DATABASE_URL}")
                init_start = time.time()
                
                # Set timeout for Firebase operations to fail faster if network is bad
                # This prevents hanging for too long on bad connections
                options = {
                    'databaseURL': DATABASE_URL,
                    'httpTimeout': 30.0  # 30 second timeout for HTTP operations
                }
                firebase_admin.initialize_app(cred, options)
                print(f"[FIREBASE DEBUG] [TIMING] Firebase app initialized in {time.time() - init_start:.2f}s")
                print(f"[FIREBASE DEBUG] [OK] Firebase Connected! (Total time: {time.time() - start_time:.1f}s)")
                print(f"[FIREBASE DEBUG] Database URL: {DATABASE_URL}")
                print(f"[FIREBASE DEBUG] Drone ID: {DRONE_ID}")
                firebase_initialized = True
            
            print(f"[FIREBASE DEBUG] [TIMING] Getting database reference...")
            ref_start = time.time()
            ref = db.reference(f'missions/{DRONE_ID}/active_command')
            print(f"[FIREBASE DEBUG] [TIMING] Database reference obtained in {time.time() - ref_start:.2f}s")
            print(f"[FIREBASE DEBUG] Listening to path: missions/{DRONE_ID}/active_command")
            
            # Listen for real-time updates FIRST (don't wait for initial fetch)
            def on_firebase_event(event):
                """Handle Firebase real-time events"""
                try:
                    print(f"[FIREBASE DEBUG] ========== FIREBASE EVENT ==========")
                    print(f"[FIREBASE DEBUG] Event path: {event.path}")
                    print(f"[FIREBASE DEBUG] Event data: {event.data}")
                    print(f"[FIREBASE DEBUG] ====================================")
                    if event.data:
                        check_mission(event.data)
                    else:
                        # Command was deleted/cleared
                        print("[FIREBASE DEBUG] Active command cleared from Firebase.")
                except Exception as e:
                    print(f"[FIREBASE DEBUG] Firebase event handler error: {e}")
                    traceback.print_exc()
            
            print(f"[FIREBASE DEBUG] [TIMING] Setting up Firebase listener...")
            print("[FIREBASE DEBUG] This may take 3-5 minutes due to network latency to Singapore...")
            print("[FIREBASE DEBUG] System is fully operational during this time! ArUco tracking works!")

            # Fetch initial command in background (don't block the main listener)
            # This way the system is ready to receive new commands immediately
            def fetch_initial_command():
                try:
                    print(f"[FIREBASE DEBUG] [TIMING] Fetching initial command from Firebase (background)...")
                    fetch_start = time.time()
                    initial_command = ref.get()
                    print(f"[FIREBASE DEBUG] [TIMING] Initial command fetched in {time.time() - fetch_start:.2f}s")
                    
                    if initial_command:
                        initial_status = initial_command.get('status')
                        print("[FIREBASE DEBUG] Found existing command in Firebase:")
                        print(f"[FIREBASE DEBUG] Command ID: {initial_command.get('id')}")
                        print(f"[FIREBASE DEBUG] Status: {initial_status}")
                        print(f"[FIREBASE DEBUG] Timestamp: {initial_command.get('timestamp')}")
                        
                        # Only process PENDING commands on startup
                        if initial_status == 'PENDING':
                            print("[FIREBASE DEBUG] Processing PENDING command...")
                            check_mission(initial_command)
                        else:
                            print(f"[FIREBASE DEBUG] Skipping old command with status '{initial_status}' (not PENDING)")
                    else:
                        print("[FIREBASE DEBUG] No existing command found in Firebase")
                except Exception as e:
                    print(f"[FIREBASE DEBUG] Error fetching initial command: {e}")
                    traceback.print_exc()
            
            # Start initial fetch in background thread so it doesn't block
            if not initial_command_fetch_started:
                initial_fetch_thread = threading.Thread(target=fetch_initial_command, name="InitialFetchThread", daemon=True)
                initial_fetch_thread.start()
                initial_command_fetch_started = True
                print("[FIREBASE DEBUG] Initial command fetch started in background thread")

            listener_start = time.time()
            print("[FIREBASE DEBUG] [OK] Firebase Listener Active - Listening for commands and abort requests...")
            print("[FIREBASE DEBUG] READY! System can now receive missions from app!")
            print(f"[FIREBASE DEBUG] [TIMING] Total Firebase setup time: {time.time() - start_time:.1f}s")
            ref.listen(on_firebase_event)
            print(f"[FIREBASE DEBUG] Firebase listener ended after {time.time() - listener_start:.1f}s; reconnecting in 10s...")
            time.sleep(10)
            
        except Exception as e:
            print(f"[FIREBASE DEBUG] Firebase listener/connection failed: {e}. Retrying in 10s...")
            print(f"[FIREBASE DEBUG] [TIMING] Failed at {time.time() - start_time:.1f}s")
            if os.environ.get("JECH_FIREBASE_TRACEBACK", "0") == "1":
                traceback.print_exc()
            time.sleep(10)

def status_publisher_thread():
    """
    Publishes always-on drone state to Firebase every 5s, regardless of mission status.
    The dashboard uses this for remote map updates even when there is no active mission.
    """
    print("[FIREBASE DEBUG] Waiting for Firebase initialization...")
    while not firebase_initialized:
        time.sleep(2)
    status_ref = db.reference(f'missions/{DRONE_ID}/status')
    print(f"[FIREBASE DEBUG] [OK] Status publisher started - writing to missions/{DRONE_ID}/status")
    while True:
        try:
            current_time = int(time.time() * 1000)
            status_payload = {
                'last_seen': current_time,
            }
            telemetry = build_telemetry_payload()
            if telemetry is not None:
                status_payload['telemetry'] = telemetry

            status_ref.update(status_payload)
            readable_time = format_timestamp(current_time)
            if telemetry is not None:
                print(
                    f"[FIREBASE DEBUG] [OK] Status published - last_seen: {current_time} ({readable_time}), "
                    f"lat={telemetry['lat']:.6f}, lng={telemetry['lng']:.6f}, alt={telemetry['alt']:.1f}m, "
                    f"batt={telemetry['batteryVoltage']:.2f}V, current={telemetry['current']:.2f}A"
                )
            else:
                print(f"[FIREBASE DEBUG] [OK] Status published - last_seen: {current_time} ({readable_time}), telemetry unavailable")
        except Exception as e:
            print(f"[FIREBASE DEBUG] [STATUS] Write failed (no internet?): {e}")
        time.sleep(TELEMETRY_INTERVAL_SEC)


# --- Control Command Handling (Remote ARM / TAKEOFF / MODE_CHANGE) ---

ALLOWED_MODES = {'LAND', 'RTL', 'AUTO', 'GUIDED', 'BRAKE'}

def handle_control_command(command):
    """
    Executes a remote control command received from Firebase.
    Supported types: ARM, TAKEOFF, MODE_CHANGE
    """
    if not command or not isinstance(command, dict):
        return

    status = command.get('status')
    cmd_type = command.get('type')
    timestamp = command.get('timestamp', 0)
    current_time = int(time.time() * 1000)

    # Only process PENDING commands
    if status != 'PENDING':
        return

    # Stale command check (15 seconds)
    if current_time - timestamp > 15000:
        print(f"[CONTROL] Stale control command ({(current_time - timestamp)/1000:.1f}s old). Ignoring.")
        try:
            ctrl_ref = db.reference(f'missions/{DRONE_ID}/control_command')
            ctrl_ref.update({'status': 'FAILED', 'error': 'Command expired'})
        except Exception:
            pass
        return

    print(f"[CONTROL] ========== CONTROL COMMAND ==========")
    print(f"[CONTROL] Type: {cmd_type}")
    print(f"[CONTROL] Timestamp: {timestamp} ({format_timestamp(timestamp)})")

    ctrl_ref = db.reference(f'missions/{DRONE_ID}/control_command')

    try:
        if cmd_type == 'ARM':
            print("[CONTROL] Arming vehicle...")
            vehicle.armed = True
            # Wait up to 3s for arming
            for _ in range(3):
                if vehicle.armed:
                    break
                time.sleep(1)
            if vehicle.armed:
                print("[CONTROL] [OK] Vehicle ARMED")
                ctrl_ref.update({'status': 'EXECUTED'})
            else:
                print("[CONTROL] [FAIL] Arming failed")
                ctrl_ref.update({'status': 'FAILED', 'error': 'Arming timed out'})

        elif cmd_type == 'TAKEOFF':
            altitude = command.get('altitude', 5.0)
            print(f"[CONTROL] Takeoff to {altitude}m...")
            # Set GUIDED mode first
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.5)
            # Arm if not armed
            if not vehicle.armed:
                vehicle.armed = True
                for _ in range(3):
                    if vehicle.armed:
                        break
                    time.sleep(1)
            if not vehicle.armed:
                print("[CONTROL] [FAIL] Could not arm for takeoff")
                ctrl_ref.update({'status': 'FAILED', 'error': 'Could not arm'})
                return
            vehicle.simple_takeoff(altitude)
            print(f"[CONTROL] [OK] Takeoff command sent (alt={altitude}m)")
            ctrl_ref.update({'status': 'EXECUTED'})

        elif cmd_type == 'MODE_CHANGE':
            mode = command.get('mode', '')
            if mode not in ALLOWED_MODES:
                print(f"[CONTROL] [FAIL] Mode '{mode}' not allowed. Allowed: {ALLOWED_MODES}")
                ctrl_ref.update({'status': 'FAILED', 'error': f'Mode {mode} not allowed'})
                return
            print(f"[CONTROL] Changing mode to {mode}...")
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
            actual_mode = vehicle.mode.name
            print(f"[CONTROL] [OK] Mode is now: {actual_mode}")
            ctrl_ref.update({'status': 'EXECUTED'})

        else:
            print(f"[CONTROL] [FAIL] Unknown command type: {cmd_type}")
            ctrl_ref.update({'status': 'FAILED', 'error': f'Unknown type: {cmd_type}'})

    except Exception as e:
        print(f"[CONTROL] [ERROR] {e}")
        traceback.print_exc()
        try:
            ctrl_ref.update({'status': 'FAILED', 'error': str(e)})
        except Exception:
            pass

    print(f"[CONTROL] ========== COMMAND END ==========")


def control_command_listener_thread():
    """
    Listens for remote control commands (ARM, TAKEOFF, MODE_CHANGE) from Firebase.
    Runs after Firebase is initialized.
    """
    print("[CONTROL] Waiting for Firebase initialization...")
    while not firebase_initialized:
        time.sleep(2)

    print(f"[CONTROL] Setting up listener on missions/{DRONE_ID}/control_command")
    ctrl_ref = db.reference(f'missions/{DRONE_ID}/control_command')

    def on_control_event(event):
        try:
            print(f"[CONTROL] Firebase event - path: {event.path}, data: {event.data}")
            if event.data and isinstance(event.data, dict):
                handle_control_command(event.data)
        except Exception as e:
            print(f"[CONTROL] Event handler error: {e}")
            traceback.print_exc()

    ctrl_ref.listen(on_control_event)
    print("[CONTROL] [OK] Control command listener active")


def init_firebase_listener():
    # Start the connection logic in a separate thread so it NEVER blocks the main script
    # Main script (ArUco tracking, vehicle connection) starts immediately
    t = threading.Thread(target=firebase_listener_thread, name="FirebaseConnectionThread", daemon=True)
    t.start()
    print("[FIREBASE DEBUG] Firebase connection started in background (non-blocking)")
    print("[FIREBASE DEBUG] Main script will continue while Firebase connects...")
    # Start always-on status publishing so the app can keep the map updated remotely.
    h = threading.Thread(target=status_publisher_thread, name="DroneStatusPublisher", daemon=True)
    h.start()
    # Start control command listener for remote ARM/TAKEOFF/MODE_CHANGE
    c = threading.Thread(target=control_command_listener_thread, name="ControlCommandListener", daemon=True)
    c.start()




def resolve_vehicle_connection_path(manual_path=None):
    """Pick a stable /dev/serial/by-id path: Prolific USB–serial (telemetry) first, then Pixhawk."""
    if manual_path:
        return manual_path

    by_id_candidates = []
    # Drone telemetry on Prolific USB–Serial (e.g. usb-Prolific_Technology_Inc._USB-Serial_Controller_*-if00-port0)
    by_id_candidates.extend(sorted(glob.glob("/dev/serial/by-id/*Prolific*if00*")))
    by_id_candidates.extend(sorted(glob.glob("/dev/serial/by-id/*Prolific*")))
    by_id_candidates.extend(sorted(glob.glob("/dev/serial/by-id/*ArduPilot*if00")))
    by_id_candidates.extend(sorted(glob.glob("/dev/serial/by-id/*Pixhawk*if00")))
    by_id_candidates.extend(sorted(glob.glob("/dev/serial/by-id/*ArduPilot*")))
    by_id_candidates.extend(sorted(glob.glob("/dev/serial/by-id/*Pixhawk*")))

    seen = set()
    for candidate in by_id_candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        if os.path.exists(candidate):
            return candidate

    # Prolific adapters usually enumerate as ttyUSB*; ACM is native Pixhawk USB.
    fallback_candidates = ["/dev/ttyUSB0", "/dev/ttyACM0"]
    for candidate in fallback_candidates:
        if os.path.exists(candidate):
            return candidate

    return "/dev/ttyUSB0"


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = 'udpout:192.168.144.14:14551', help="Vehicle connection path. Defaults to Prolific *USB-Serial* under /dev/serial/by-id when present, else Pixhawk-style by-id, else ttyUSB0/ttyACM0. Overridden to UDP by default.")
parser.add_argument('--baud', type=int, default=int(os.environ.get("JECH_MAVLINK_BAUD", "921600")), help="Vehicle serial baud rate. Default: %(default)s")
args = parser.parse_args()
args.connect = resolve_vehicle_connection_path(args.connect)

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------     
# Define function to send landing_target mavlink message for mavlink based precision landing
# http://mavlink.org/messages/common#LANDING_TARGET
def send_land_message_v2(x_rad=0.0, y_rad=0.0, dist_m=0.0, x_m=0.0,y_m=0.0,z_m=0.0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        int(time_usec),          # time target data was processed, as close to sensor capture as possible
        int(target_num),         # target num, not used
        int(mavutil.mavlink.MAV_FRAME_BODY_FRD), # frame, not used
        float(x_rad),            # X-axis angular offset, in radians
        float(y_rad),            # Y-axis angular offset, in radians
        float(dist_m),       # distance, in meters
        0.0,                     # Target x-axis size, in radians
        0.0,                     # Target y-axis size, in radians
        float(x_m),              # x position in frame
        float(y_m),              # y position in frame
        float(z_m),              # z position in frame
        (1.0,0.0,0.0,0.0),       # orientation quaternion
        int(2),                  # type of landing target: 2 = Fiducial marker
        int(0),                  # position_valid boolean
    )
    # print(msg)
    vehicle.send_mavlink(msg)


# Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
# http://mavlink.org/messages/common#DISTANCE_SENSOR
def send_distance_message( dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)     

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav = x_cam
    y_uav = y_cam
    return(x_uav, y_uav)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle with retry loop
if args.connect.startswith('udp') or args.connect.startswith('tcp') or args.connect.startswith('192.168.'):
    print(f'Connecting using {args.connect} (Network IP connection)...')
else:
    print(f'Connecting using {args.connect} at {args.baud} baud...')

while True:
    try:
        if args.connect.startswith('udp') or args.connect.startswith('tcp') or args.connect.startswith('192.168.'):
            vehicle = connect(args.connect)
        else:
            vehicle = connect(args.connect, baud=args.baud)
        break
    except Exception as e:
        print(f"Connection failed: {e}. Retrying in 2s...")
        time.sleep(2)
print(vehicle, "connected!!!")

@vehicle.on_message('SYS_STATUS')
def sys_status_battery_listener(self, name, message):
    """Cache MAVLink SYS_STATUS battery data as a fallback for DroneKit vehicle.battery."""
    global latest_battery_voltage, latest_battery_current, latest_battery_remaining
    voltage_mv = int(getattr(message, "voltage_battery", -1))
    current_ca = int(getattr(message, "current_battery", -1))
    remaining = int(getattr(message, "battery_remaining", -1))
    if voltage_mv > 0:
        latest_battery_voltage = voltage_mv / 1000.0
    if current_ca >= 0:
        latest_battery_current = current_ca / 100.0
    if remaining >= 0:
        latest_battery_remaining = remaining

#--------------------------------------------------
#-------------- FLIGHT CONTROLLER LOG SERVICE
#--------------------------------------------------
# HTTP log browser + DataFlash download via QGC-style LOG_REQUEST_DATA chunks (fc_log_service.py).
# Optional env: JECH_FC_LOG_CACHE_DIR, JECH_FC_LOG_PORT, JECH_FC_LOG_DOWNLOAD_TIMEOUT_SEC, etc.
# No extra wiring here beyond start_log_services + disarm hook below.
# Single DroneKit Vehicle instance: pass-through only — fc_log_service does NOT call connect().
fc_log_service = start_log_services(vehicle)

#--------------------------------------------------
#-------------- LED CONTROLLER & FAILSAFE LISTENER
#--------------------------------------------------
led_controller = DroneLEDController()
led_controller.start()

battery_failsafe_active = False
other_failsafe_active = False

@vehicle.on_message('STATUSTEXT')
def listener(self, name, message):
    global battery_failsafe_active, other_failsafe_active
    text = message.text.upper()
    if "BATTERY" in text and "FAILSAFE" in text:
        battery_failsafe_active = True
        print("[LED] Battery Failsafe Detected!")
    elif "FAILSAFE" in text:
        other_failsafe_active = True
        print(f"[LED] Failsafe Detected: {text}")

@vehicle.on_attribute('armed')
def armed_listener(self, attr_name, value):
    global battery_failsafe_active, other_failsafe_active
    if value:
        battery_failsafe_active = False
        other_failsafe_active = False
        print("[LED] Armed - Resetting Failsafe Flags")
    else:
        # Disarmed: background thread in fc_log_service waits briefly, then pulls the latest .bin
        # over QGC-style LOG_REQUEST_DATA chunks (same stack as HTTP /api/logs/latest).
        print("[LOG SERVICE] Disarm detected - triggering auto log download...")
        fc_log_service.auto_download_latest_log()


#--------------------------------------------------
#-------------- PARAMETERS  
#--------------------------------------------------
# DroneKit parameter writes can block startup (wait_ready() default 30s) on slow UART links.
# To match the instant log-download behavior (scripts/log_download.py), send MAVLink PARAM_SET
# directly on the already-open vehicle link and continue without waiting for a full param table.
def set_param_nonblocking(param_name: str, value: float, param_type: int) -> None:
    master = vehicle._master
    pid = param_name.encode("ascii", errors="ignore")[:16].ljust(16, b"\x00")
    master.mav.param_set_send(master.target_system, master.target_component, pid, float(value), int(param_type))
    print(f"[PARAM] Sent PARAM_SET {param_name}={value} type={param_type} (non-blocking)")

# PLND params (integers in ArduPilot; send as INT8 to avoid float-only semantics).
set_param_nonblocking("PLND_ENABLED", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
set_param_nonblocking("PLND_TYPE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)  # Mavlink landing backend

# vehicle.parameters['LAND_REPOSITION']   = 0 # !!!!!! ONLY FOR SITL IF NO RC IS CONNECTED


# vehicle.parameters['RNGFND_TYPE']       = 10
# vehicle.parameters['RNGFND_MIN_CM']     = 1
# vehicle.parameters['RNGFND_MAX_CM']     = 10000
# vehicle.parameters['RNGFND_GNDCLEAR']   = 5     

# --- Start Firebase Listener ---
init_firebase_listener()

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
# IMPORTANT: Update this ID to match your physical ArUco marker
id_to_find      = 132  # Changed from 72 to match detected marker
marker_size     = 17.8 #- [cm]
freq_send       = 10 #- Hz


#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
# Arducam 64MP OV64A40 camera resolution
# Native: 9248x6944 (64MP)
# Available modes: 1920x1080@45fps, 2312x1736@26fps, 3840x2160@14fps, 4624x3472@7fps
# Using 1920x1080@45fps for optimal balance between resolution and frame rate for precision landing
# Higher resolution allows marker detection from greater altitudes with continuous autofocus
camera_resolution   = [1920, 1080]  # 16:9 aspect ratio, 45fps with link-frequency=360000000
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True, axis_scale=0.01,
                camera_matrix=camera_matrix, camera_distortion=camera_distortion, camera_size=camera_resolution)
                
                
time_0 = time.time()

#--- Check mavlink standard
mavlink20 = 'MAVLINK20' in os.environ

#--------------------------------------------------
#-------------- ROLLING STABILITY BUFFER
#--------------------------------------------------
# Initialize detection stability buffer with maxlen=7
# This prevents jerky movements when marker detection drops temporarily
detection_buffer = deque(maxlen=7)
last_known_position = None  # Store last known valid position (x_cm, y_cm, z_cm)
confidence_threshold =20.0  # Minimum confidence percentage to send position data

print(f"Rolling Stability Buffer initialized (size: 7, threshold: {confidence_threshold}%)")

while True:                
    # Detect marker in current frame
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    
    # Update detection buffer
    if marker_found:
        # Marker detected - append 1 and update last known position
        detection_buffer.append(1)
        x_cm, y_cm = camera_to_uav(x_cm, y_cm)
        z_cm = max(1.0, float(z_cm))
        last_known_position = (x_cm, y_cm, z_cm)
    else:
        # Marker not detected - append 0, keep last known position
        detection_buffer.append(0)
    
    # Calculate confidence score (percentage of successful detections in buffer)
    if len(detection_buffer) > 0:
        confidence_score = (sum(detection_buffer) / len(detection_buffer)) * 100.0
    else:
        confidence_score = 0.0
    
    # Send position data only if confidence exceeds threshold and we have a valid position
    if confidence_score >= confidence_threshold and last_known_position is not None:
        x_cm, y_cm, z_cm = last_known_position
        angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)
        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            status = "DETECTED" if marker_found else "TRACKING"
            print(f"[{status}] Confidence: {confidence_score:.1f}% | x={x_cm:5.0f}cm y={y_cm:5.0f}cm z={z_cm:5.0f}cm | angles=({angle_x:.3f}, {angle_y:.3f})")
            # send_land_message(x_m=x_cm*0.01, y_m=y_cm*0.01, z_m=z_cm*0.01)
            send_land_message_v2(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
    else:
        # Low confidence or no position data - do not send
        # No print statements to avoid log spam
        pass
    
    #--------------------------------------------------
    #-------------- LED CONTROL UPDATE
    #--------------------------------------------------
    # Determine the target LED state based on priorities
    target_led_state = DroneLEDController.STATE_DISARMED # Default
    
    # Check for Failsafes first (Highest Priority)
    if battery_failsafe_active:
        target_led_state = DroneLEDController.STATE_BAT_FAILSAFE
    elif other_failsafe_active:
        target_led_state = DroneLEDController.STATE_FAILSAFE
    
    # Check Flight Modes
    elif vehicle.mode.name == 'LAND':
        target_led_state = DroneLEDController.STATE_LAND
    elif vehicle.mode.name == 'RTL':
        target_led_state = DroneLEDController.STATE_RTL
        
    # Check Armed Status
    elif not vehicle.armed:
        target_led_state = DroneLEDController.STATE_DISARMED
    else:
        # Armed
        if vehicle.location.global_relative_frame.alt < 0.3: # Below 30cm -> Considered on Ground? Or verify logic
            # "ARMED and on the ground"
            # Detecting "on ground" reliably without a sensor can be tricky. 
            # Using altitude < 0.5m as a proxy or if motors are spinning but not taking off.
            # Ideally use vehicle.system_status.state or similar if available, 
            # but altitude is a common simple check.
            target_led_state = DroneLEDController.STATE_ARMED_GROUND
        else:
            target_led_state = DroneLEDController.STATE_FLYING
            
    # Update LED Controller
    led_controller.set_state(target_led_state)
      
