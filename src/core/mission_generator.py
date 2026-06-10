from pymavlink import mavutil
import math

class LatLng:
    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng

class MissionItem:
    def __init__(self, seq, command_id, x, y, z, param1=0, param2=0, param3=0, param4=0, frame=3, current=0, autocontinue=1):
        self.seq = seq
        self.command_id = command_id
        self.frame = frame # 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.x = x # lat
        self.y = y # lon
        self.z = z # alt
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.current = current
        self.autocontinue = autocontinue

class TemplateCommand:
    def __init__(self, type_name, params=None):
        self.type = type_name
        self.params = params if params else {}

class DeliveryTemplate:
    def __init__(self, commands, default_values=None):
        self.commands = commands
        self.default_values = default_values if default_values else {}

    @staticmethod
    def get_default_template():
        return DeliveryTemplate(
            commands=[
                TemplateCommand('home', {'location': '{HOME_LOCATION}', 'alt': 0}),
                TemplateCommand('takeoff', {'alt': '{TAKEOFF_ALT}'}),
                TemplateCommand('changeSpeed', {'speed': '{FLY_SPEED}'}),
                TemplateCommand('waypoint', {
                    'location': '{DELIVERY_LOCATION}',
                    'alt': '{CRUISE_ALT}'
                }),
                TemplateCommand('land', {
                    'location': '{DELIVERY_LOCATION}'
                }),
                TemplateCommand('doSetServo', {
                    'servo': '{SERVO_NUM}',
                    'pwm': '{LID_UNLOCK_PWM}'
                }),
                TemplateCommand('delay', {
                    'seconds': '{DROP_DELAY}'
                }),
                TemplateCommand('takeoff', {'alt': '{TAKEOFF_ALT}'}),
                TemplateCommand('doSetServo', {
                    'servo': '{SERVO_NUM}',
                    'pwm': '{LID_LOCK_PWM}'
                }),
                TemplateCommand('delay', {
                    'seconds': 5.0
                }),
                TemplateCommand('rtl', {}),
            ],
            default_values={
                'TAKEOFF_ALT': 30.0,
                'CRUISE_ALT': 30.0,
                'LID_UNLOCK_PWM': 1000,
                'LID_LOCK_PWM': 1900,
                'SERVO_NUM': 6,
                'DROP_DELAY': 5.0,
                'FLY_SPEED': 0.0,
            }
        )

    def generate_mission(self, home_location, delivery_location, override_values=None):
        values = self.default_values.copy()
        if override_values:
            normalized_overrides = override_values.copy()
            if 'SERVO_OPEN_PWM' in normalized_overrides and 'LID_UNLOCK_PWM' not in normalized_overrides:
                normalized_overrides['LID_UNLOCK_PWM'] = normalized_overrides['SERVO_OPEN_PWM']
            if 'SERVO_CLOSE_PWM' in normalized_overrides and 'LID_LOCK_PWM' not in normalized_overrides:
                normalized_overrides['LID_LOCK_PWM'] = normalized_overrides['SERVO_CLOSE_PWM']
            values.update(normalized_overrides)

        # Force SERVO_NUM to 6 as we only handle servo 6 on the drone
        values['SERVO_NUM'] = 6

        mission_items = []
        seq = 0
        last_nav_location = home_location
        first_executable_seq = 1
        
        # MAVLink Command Mapping
        CMD_MAP = {
            'home': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # 16
            'waypoint': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # 16
            'takeoff': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # 22
            'land': mavutil.mavlink.MAV_CMD_NAV_LAND, # 21
            'rtl': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, # 20
            'doSetServo': mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 183
            'delay': mavutil.mavlink.MAV_CMD_NAV_DELAY, # 93
            'changeSpeed': mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # 178
        }

        for cmd in self.commands:
            command_id = CMD_MAP.get(cmd.type, 16)
            
            # Init params
            p1, p2, p3, p4 = 0.0, 0.0, 0.0, 0.0
            x, y, z = 0.0, 0.0, 0.0
            
            # 1. Substitute variables logic
            temp_params = {} 
            for key, value in cmd.params.items():
                final_val = value
                if isinstance(value, str) and value.startswith('{') and value.endswith('}'):
                    var_name = value[1:-1]
                    if var_name == 'HOME_LOCATION' and home_location:
                        final_val = home_location
                    elif var_name == 'DELIVERY_LOCATION' and delivery_location:
                        final_val = delivery_location
                    elif var_name in values:
                        final_val = values[var_name]
                    else:
                         final_val = 0 # Default if missing
                temp_params[key] = final_val

            # 2. Map params to MAVLink fields
            # Positional params depends on command type
            
            if cmd.type in ['home', 'waypoint', 'land']:
                loc = temp_params.get('location')
                if isinstance(loc, LatLng):
                    x = loc.latitude
                    y = loc.longitude
                    last_nav_location = loc
                z = float(temp_params.get('alt', 0))
                
            elif cmd.type == 'takeoff':
                loc = temp_params.get('location', last_nav_location)
                # For Copter, NAV_TAKEOFF should climb from the current position.
                # Sending 0/0 avoids ArduPilot storing it as a normal waypoint.
                if seq != 0 and isinstance(loc, LatLng):
                    x = loc.latitude
                    y = loc.longitude
                z = float(temp_params.get('alt', 0))
                
            elif cmd.type == 'doSetServo':
                p1 = float(temp_params.get('servo', 0))
                p2 = float(temp_params.get('pwm', 0))
                
            elif cmd.type == 'delay':
                p1 = float(temp_params.get('seconds', 0))

            elif cmd.type == 'changeSpeed':
                speed = float(temp_params.get('speed', 0))
                if speed <= 0:
                    continue
                p1 = 1.0
                p2 = speed
                p3 = -1.0 # -1 indicates no change to throttle. 0.0 would kill the motors!

            # Create Item
            item = MissionItem(
                seq=seq,
                command_id=command_id,
                x=x, y=y, z=z,
                param1=p1, param2=p2, param3=p3, param4=p4,
                frame=(
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                    if command_id in [
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                    ]
                    else mavutil.mavlink.MAV_FRAME_MISSION
                ),
                current=1 if seq == first_executable_seq else 0
            )
            mission_items.append(item)
            seq += 1

        if len(mission_items) < 2 or mission_items[0].command_id != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            raise ValueError("Generated mission must include ArduPilot home item at seq 0")
        if mission_items[1].command_id != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            raise ValueError("Generated mission executable seq 1 must be MAV_CMD_NAV_TAKEOFF")

        return mission_items
