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
                TemplateCommand('waypoint', {
                    'location': '{DELIVERY_LOCATION}',
                    'alt': '{CRUISE_ALT}'
                }),
                TemplateCommand('land', {
                    'location': '{DELIVERY_LOCATION}'
                }),
                TemplateCommand('doSetServo', {
                    'servo': '{SERVO_NUM}',
                    'pwm': '{SERVO_OPEN_PWM}'
                }),
                TemplateCommand('delay', {
                    'seconds': '{DROP_DELAY}'
                }),
                TemplateCommand('takeoff', {'alt': '{TAKEOFF_ALT}'}),
                TemplateCommand('doSetServo', {
                    'servo': '{SERVO_NUM}',
                    'pwm': '{SERVO_CLOSE_PWM}'
                }),
                TemplateCommand('rtl', {}),
            ],
            default_values={
                'TAKEOFF_ALT': 30.0,
                'CRUISE_ALT': 30.0,
                'SERVO_NUM': 15,
                'SERVO_OPEN_PWM': 1900,
                'SERVO_CLOSE_PWM': 1000,
                'DROP_DELAY': 10.0,
            }
        )

    def generate_mission(self, home_location, delivery_location, override_values=None):
        values = self.default_values.copy()
        if override_values:
            values.update(override_values)

        mission_items = []
        seq = 0
        
        # MAVLink Command Mapping
        CMD_MAP = {
            'home': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # 16
            'waypoint': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # 16
            'takeoff': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # 22
            'land': mavutil.mavlink.MAV_CMD_NAV_LAND, # 21
            'rtl': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, # 20
            'doSetServo': mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 183
            'delay': mavutil.mavlink.MAV_CMD_CONDITION_DELAY, # 112
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
            
            if cmd.type in ['waypoint', 'land', 'home']:
                loc = temp_params.get('location')
                if isinstance(loc, LatLng):
                    x = loc.latitude
                    y = loc.longitude
                z = float(temp_params.get('alt', 0))
                
            elif cmd.type == 'takeoff':
                z = float(temp_params.get('alt', 0))
                
            elif cmd.type == 'doSetServo':
                p1 = float(temp_params.get('servo', 0))
                p2 = float(temp_params.get('pwm', 0))
                
            elif cmd.type == 'delay':
                p1 = float(temp_params.get('seconds', 0))

            # Create Item
            item = MissionItem(
                seq=seq,
                command_id=command_id,
                x=x, y=y, z=z,
                param1=p1, param2=p2, param3=p3, param4=p4,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            )
            mission_items.append(item)
            seq += 1
            
        return mission_items
