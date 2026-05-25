import time
import threading
import queue
import traceback
import contextlib
from pymavlink import mavutil

# ArduCopter mode map fallback
COPTER_MODE_MAP = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
    5: 'LOITER', 6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT',
    13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD',
    17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS',
    21: 'SMART_RTL', 22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG',
    25: 'SYSTEM_ID', 26: 'AUTOROTATE', 27: 'AUTO_RTL'
}

class VehicleMode:
    """Emulates DroneKit's VehicleMode object."""
    def __init__(self, name="UNKNOWN"):
        self.name = name.upper()

    def __str__(self):
        return self.name

    def __repr__(self):
        return f"VehicleMode:{self.name}"

class LocationGlobal:
    """Emulates DroneKit's LocationGlobal object."""
    def __init__(self, lat=None, lon=None, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def __str__(self):
        return f"lat={self.lat},lon={self.lon},alt={self.alt}"

class LocationGlobalRelative:
    """Emulates DroneKit's LocationGlobalRelative object."""
    def __init__(self, lat=None, lon=None, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def __str__(self):
        return f"lat={self.lat},lon={self.lon},alt={self.alt}"

class LocationInfo:
    """Emulates DroneKit's LocationInfo container."""
    def __init__(self):
        self.global_frame = LocationGlobal()
        self.global_relative_frame = LocationGlobalRelative()

class BatteryInfo:
    """Emulates DroneKit's BatteryInfo object."""
    def __init__(self):
        self.voltage = None
        self.current = None
        self.level = None

class MavlinkVehicle:
    """
    Production-ready, thread-safe, pure PyMAVLink vehicle class.
    Provides transparent API emulation for DroneKit properties, callbacks, and methods.
    """
    def __init__(self, connection_path, baud=921600):
        # Parse connection path to see if it is a network connection
        is_network = (
            connection_path.startswith('udp') or
            connection_path.startswith('tcp') or
            connection_path.startswith('192.168.') or
            (':' in connection_path and not connection_path.startswith('/dev/'))
        )
        
        if is_network:
            print(f"[MAVLINK VEHICLE] Connecting to network address {connection_path}...")
            self._master = mavutil.mavlink_connection(connection_path)
        else:
            print(f"[MAVLINK VEHICLE] Connecting to serial port {connection_path} at {baud} baud...")
            self._master = mavutil.mavlink_connection(connection_path, baud=baud)
        
        # Target details (populated on first heartbeat)
        self.target_system = 0
        self.target_component = 0
        
        # State attributes
        self._armed = False
        self._mode = VehicleMode("UNKNOWN")
        self.heading = 0.0
        self.location = LocationInfo()
        self.battery = BatteryInfo()
        self.channels = {}
        
        # Thread synchronization
        self._msg_queues = []
        self._msg_queues_lock = threading.Lock()
        
        self._listeners = {}
        self._listeners_lock = threading.Lock()
        
        self._attribute_listeners = {}
        self._attribute_listeners_lock = threading.Lock()
        
        # message factory compatibility
        self.message_factory = self._master.mav
        
        # Wait for the initial heartbeat to set target IDs and mode maps.
        # If it is a network (especially udpout) connection, send GCS heartbeats
        # periodically so that the autopilot can discover our IP and port.
        print("[MAVLINK VEHICLE] Waiting for heartbeat...")
        hb = None
        start_wait = time.time()
        while time.time() - start_wait < 15.0:
            if is_network:
                try:
                    # Send a GCS heartbeat to let the autopilot discover us
                    self._master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                except Exception:
                    pass
            # Try to receive a heartbeat
            hb = self._master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if hb is not None:
                break

        if hb is None:
            raise TimeoutError("No heartbeat received from flight controller during startup!")
            
        self.target_system = self._master.target_system
        self.target_component = self._master.target_component
        print(f"[MAVLINK VEHICLE] Heartbeat established: system={self.target_system}, component={self.target_component}")
        
        # Populate initial reverse mode mapping
        self._mode_map_rev = {v: k for k, v in self._master.mode_mapping().items()}
        self._update_mode_from_heartbeat(hb)
        self._update_armed_from_heartbeat(hb)
        
        # Start the data streams from flight controller
        self.start_streams()
        
        # Start background read loop
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="MavlinkReader", daemon=True)
        self._reader_thread.start()
        print("[MAVLINK VEHICLE] Background telemetry and reader thread started.")

    @property
    def armed(self):
        return self._armed

    @armed.setter
    def armed(self, value):
        if value:
            self.arm()
        else:
            self.disarm()

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        mode_name = value if isinstance(value, str) else value.name
        self.set_mode(mode_name)

    def arm(self, timeout=5.0):
        """Send arm command and wait for confirmation."""
        print("[MAVLINK VEHICLE] Sending ARM command...")
        self._master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1=arm
            0, 0, 0, 0, 0, 0
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._armed:
                print("[MAVLINK VEHICLE] ARM successful!")
                return True
            time.sleep(0.1)
        print("[MAVLINK VEHICLE] ARM timed out or failed!")
        return False

    def disarm(self, timeout=5.0):
        """Send disarm command and wait for confirmation."""
        print("[MAVLINK VEHICLE] Sending DISARM command...")
        self._master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0=disarm
            0, 0, 0, 0, 0, 0
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._armed:
                print("[MAVLINK VEHICLE] DISARM successful!")
                return True
            time.sleep(0.1)
        print("[MAVLINK VEHICLE] DISARM timed out or failed!")
        return False

    def set_mode(self, mode_name, timeout=5.0):
        """Switch vehicle mode via PyMAVLink."""
        mode_name = mode_name.upper()
        
        # Try to find the mode ID in COPTER_MODE_MAP first since this is a Copter
        mode_id = next((k for k, v in COPTER_MODE_MAP.items() if v == mode_name), None)
        if mode_id is not None:
            self._master.set_mode(mode_id)
            print(f"[MAVLINK VEHICLE] Mode switch command sent: {mode_name} ({mode_id})")
        else:
            # Fall back to self._master.mode_mapping()
            mode_map = self._master.mode_mapping()
            if mode_name in mode_map:
                mode_id = mode_map[mode_name]
                self._master.set_mode(mode_id)
                print(f"[MAVLINK VEHICLE] Mode switch command sent via fallback: {mode_name} ({mode_id})")
            else:
                raise ValueError(f"Unknown flight mode: {mode_name}")
        
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._mode.name == mode_name:
                print(f"[MAVLINK VEHICLE] Mode switch verified: {mode_name}")
                return True
            time.sleep(0.1)
        print(f"[MAVLINK VEHICLE] Mode switch to {mode_name} timed out!")
        return False

    def takeoff(self, altitude):
        """Perform simple guided takeoff."""
        print(f"[MAVLINK VEHICLE] Initiating takeoff to {altitude}m...")
        self._master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            float(altitude)
        )

    def simple_takeoff(self, altitude):
        """DroneKit simple_takeoff compatibility alias."""
        self.takeoff(altitude)

    def send_mavlink(self, msg):
        """Send a raw MAVLink message to the flight controller."""
        self._master.mav.send(msg)

    def add_message_listener(self, name, callback):
        """Register a message callback (DroneKit compatibility)."""
        with self._listeners_lock:
            if name not in self._listeners:
                self._listeners[name] = []
            self._listeners[name].append(callback)

    def remove_message_listener(self, name, callback):
        """Unregister a message callback."""
        with self._listeners_lock:
            if name in self._listeners and callback in self._listeners[name]:
                self._listeners[name].remove(callback)

    def on_message(self, name):
        """Decorator for registering message listeners."""
        def decorator(callback):
            self.add_message_listener(name, callback)
            return callback
        return decorator

    def on_attribute(self, name):
        """Decorator for registering attribute change listeners."""
        def decorator(callback):
            with self._attribute_listeners_lock:
                if name not in self._attribute_listeners:
                    self._attribute_listeners[name] = []
                self._attribute_listeners[name].append(callback)
            return callback
        return decorator

    def flush(self):
        """Dummy compatibility function."""
        pass

    def close(self):
        """Stop threads and close connection."""
        self._stop_event.set()
        self._master.close()

    @contextlib.contextmanager
    def _register_response_queue(self, q):
        """Register a queue to receive thread-safe synchronous replies."""
        with self._msg_queues_lock:
            self._msg_queues.append(q)
        try:
            yield
        finally:
            with self._msg_queues_lock:
                if q in self._msg_queues:
                    self._msg_queues.remove(q)

    def wait_for_message(self, q, msg_types, timeout=5.0):
        """Synchronously wait for specific message types in a thread-safe manner."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = q.get(timeout=max(0.01, deadline - time.time()))
                if msg.get_type() in msg_types:
                    return msg
            except queue.Empty:
                break
        raise TimeoutError(f"Timed out waiting for message types {msg_types}")

    def start_streams(self, rate=4):
        """Send data stream requests to ArduPilot."""
        print(f"[MAVLINK VEHICLE] Sending stream request MAV_DATA_STREAM_ALL at {rate}Hz...")
        try:
            self._master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate,
                1  # 1 = start
            )
        except Exception as e:
            print(f"[MAVLINK VEHICLE] Error requesting streams: {e}")

    def upload_mission(self, mission_items, timeout=15.0):
        """
        Upload mission items using robust response queue synchronization
        following the modern MAVLink MISSION_ITEM_INT protocol.
        """
        count = len(mission_items)
        print(f"[MAVLINK VEHICLE] Uploading {count} mission items...")
        
        q = queue.Queue()
        with self._register_response_queue(q):
            # 1. Clear existing mission
            self._master.mav.mission_clear_all_send(self.target_system, self.target_component)
            try:
                ack = self.wait_for_message(q, ["MISSION_ACK"], timeout=3.0)
                print(f"[MAVLINK VEHICLE] Clear mission ACK: type={ack.type}")
            except TimeoutError:
                print("[MAVLINK VEHICLE] Warning: Clear ACK timed out, continuing...")
            
            # 2. Send count
            self._master.mav.mission_count_send(
                self.target_system,
                self.target_component,
                count
            )
            
            sent_sequences = set()
            deadline = time.time() + timeout
            
            while time.time() < deadline:
                msg = self.wait_for_message(q, ["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], timeout=deadline - time.time())
                msg_type = msg.get_type()
                
                if msg_type == "MISSION_ACK":
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        print("[MAVLINK VEHICLE] Mission upload fully accepted by Flight Controller!")
                        return True
                    else:
                        raise RuntimeError(f"Mission upload rejected by FC: ACK type {msg.type}")
                
                seq = int(msg.seq)
                if seq < 0 or seq >= count:
                    raise IndexError(f"Flight controller requested invalid mission index: {seq}")
                
                item = mission_items[seq]
                
                # Check if this frame uses standard float scaling or int scaling (* 1e7)
                is_int_frame = item.frame in [
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
                ]
                x_int = int(round(float(item.x) * 1e7)) if is_int_frame else int(round(float(item.x)))
                y_int = int(round(float(item.y) * 1e7)) if is_int_frame else int(round(float(item.y)))
                
                # Send the specific requested item
                self._master.mav.mission_item_int_send(
                    self.target_system,
                    self.target_component,
                    seq,
                    item.frame,
                    item.command_id,
                    item.current,
                    item.autocontinue,
                    item.param1,
                    item.param2,
                    item.param3,
                    item.param4,
                    x_int,
                    y_int,
                    item.z
                )
                sent_sequences.add(seq)
            
            raise TimeoutError("Mission item upload transaction timed out!")

    def _update_mode_from_heartbeat(self, msg):
        custom_mode = msg.custom_mode
        # First, try to look up in the COPTER_MODE_MAP since this is a copter system
        mode_name = COPTER_MODE_MAP.get(custom_mode)
        if not mode_name:
            # Fall back to the active MAVLink connection mode mapping
            mode_name = self._mode_map_rev.get(custom_mode, "UNKNOWN")
        
        old_mode_name = self._mode.name
        if mode_name != old_mode_name:
            self._mode = VehicleMode(mode_name)

    def _update_armed_from_heartbeat(self, msg):
        new_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        old_armed = self._armed
        if new_armed != old_armed:
            self._armed = new_armed
            self._notify_attribute('armed', new_armed)

    def _notify_attribute(self, name, value):
        with self._attribute_listeners_lock:
            callbacks = list(self._attribute_listeners.get(name, []))
        for cb in callbacks:
            try:
                cb(self, name, value)
            except Exception as e:
                print(f"[MAVLINK VEHICLE] Error invoking attribute callback: {e}")

    def _reader_loop(self):
        """Background thread that continually polls the serial link."""
        last_stream_request = time.time()
        while not self._stop_event.is_set():
            try:
                # Send periodic stream requests every 5 seconds to ensure streams stay active
                if time.time() - last_stream_request > 5.0:
                    self.start_streams()
                    last_stream_request = time.time()

                # Read message with a short timeout to check self._stop_event
                msg = self._master.recv_match(blocking=True, timeout=0.05)
                if msg is None:
                    continue
                
                msg_type = msg.get_type()
                
                # Process core state changes
                if msg_type == 'HEARTBEAT':
                    self._update_mode_from_heartbeat(msg)
                    self._update_armed_from_heartbeat(msg)
                
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.location.global_frame.lat = msg.lat / 1e7
                    self.location.global_frame.lon = msg.lon / 1e7
                    self.location.global_frame.alt = msg.alt / 1000.0
                    self.location.global_relative_frame.lat = msg.lat / 1e7
                    self.location.global_relative_frame.lon = msg.lon / 1e7
                    self.location.global_relative_frame.alt = msg.relative_alt / 1000.0
                    self.heading = msg.hdg / 100.0
                
                elif msg_type == 'SYS_STATUS':
                    if msg.voltage_battery > 0:
                        self.battery.voltage = msg.voltage_battery / 1000.0
                    if msg.current_battery >= 0:
                        self.battery.current = msg.current_battery / 100.0
                    if msg.battery_remaining >= 0:
                        self.battery.level = msg.battery_remaining
                
                elif msg_type == 'BATTERY_STATUS':
                    if hasattr(msg, 'voltages'):
                        valid_v = [v for v in msg.voltages if v < 65535]
                        if valid_v:
                            self.battery.voltage = sum(valid_v) / 1000.0
                    if msg.current_battery >= 0:
                        self.battery.current = msg.current_battery / 100.0
                
                elif msg_type == 'RC_CHANNELS':
                    for i in range(1, 19):
                        ch_val = getattr(msg, f'chan{i}_raw', 0)
                        self.channels[str(i)] = ch_val
                
                # Distribute message to synchronous response queues
                with self._msg_queues_lock:
                    for q in self._msg_queues:
                        q.put(msg)
                
                # Distribute message to registered DroneKit callbacks
                with self._listeners_lock:
                    callbacks = list(self._listeners.get(msg_type, []))
                for cb in callbacks:
                    try:
                        cb(self, msg_type, msg)
                    except Exception as le:
                        print(f"[MAVLINK VEHICLE] Listener error for {msg_type}: {le}")
                        
            except Exception as e:
                # Avoid crashing loop on minor disconnect or packet corruption
                if not self._stop_event.is_set():
                    time.sleep(0.01)
