import time
import threading
import board
import neopixel

# LED strip configuration
S1_PIN         = board.D10    # Left Ring (SPI MOSI - GPIO 10)
S2_PIN         = board.D18    # Right Ring
S3_PIN         = board.D12    # Eyes
RING_COUNT     = 8
EYE_COUNT      = 21           # Total pixels: 11 (Eyes) + 4 (Sides) + 6 (Box)
LED_BRIGHTNESS = 0.9          # Set to 0.0 to 1.0
LED_ORDER      = neopixel.GRB  # Standard for WS2812B

# Colors (R, G, B) Tuples
RED     = (255, 0, 0)
GREEN   = (0, 255, 0)
BLUE    = (0, 0, 255)
YELLOW  = (255, 255, 0)
PURPLE  = (128, 0, 128)
OFF     = (0, 0, 0)

def wheel(pos):
    """Input a value 0 to 255 to get a color value. The colours are a transition r - g - b - back to r."""
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    if pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    pos -= 170
    return (pos * 3, 0, 255 - pos * 3)

class DroneLEDController(threading.Thread):
    # States
    STATE_DISARMED       = "DISARMED"
    STATE_ARMED_GROUND   = "ARMED_GROUND"
    STATE_FLYING         = "FLYING"
    STATE_PRECISION_LAND = "PRECISION_LAND"
    STATE_LAND           = "LAND"
    STATE_RTL            = "RTL"
    STATE_BAT_FAILSAFE   = "BAT_FAILSAFE"
    STATE_FAILSAFE       = "FAILSAFE"
    STATE_CRASH          = "CRASH"

    def __init__(self):
        super().__init__()
        self.daemon = True
        self.stop_event = threading.Event()
        self.current_state = self.STATE_DISARMED
        self.lock = threading.Lock()
        # Prevent concurrent access to NeoPixel C-extension objects.
        # Only one thread should touch `fill()`, pixel assignment, or `show()` at a time.
        self.strip_lock = threading.Lock()
        # State changes can request an immediate clear on the next LED loop tick.
        self._clear_requested = False
        
        # Initialize Strips
        try:
            # S1: Left Ring
            self.left_ring = neopixel.NeoPixel(S1_PIN, RING_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)
            # S2: Right Ring
            self.right_ring = neopixel.NeoPixel(S2_PIN, RING_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)
            # S3: Eyes
            self.eyes = neopixel.NeoPixel(S3_PIN, EYE_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)
            self.strips = [self.left_ring, self.right_ring, self.eyes]
        except Exception as e:
            print(f"[LED] Error initializing NeoPixels: {e}")
            self.left_ring = self.right_ring = self.eyes = None
            self.strips = []
            
        # Segments for Eyes and Sides
        self.eye_left_indices = [0, 1, 2, 3, 4, 5]
        self.eye_right_indices = [6, 7, 8, 9, 10]
        self.sides_indices = [11, 12, 13, 14]
        self.box_indices = [15, 16, 17, 18, 19, 20]

    def set_state(self, new_state):
        with self.lock:
            if self.current_state != new_state:
                self.current_state = new_state
                # Don't touch hardware from outside the LED thread.
                # Request a clear that will be executed safely inside `run()`.
                self._clear_requested = True

    def get_state(self):
        with self.lock:
            return self.current_state

    def clear_all(self):
        with self.strip_lock:
            for strip in self.strips:
                if strip:
                    strip.fill(OFF)
                    strip.show()

    def show_all(self):
        with self.strip_lock:
            for strip in self.strips:
                if strip:
                    strip.show()

    def _get_breathe_color(self, base_color, phase):
        # phase varies 0 to 1
        import math
        brightness = (math.sin(phase * 2 * math.pi) + 1) / 2 # 0 to 1
        # Apply brightness (minimum 10% to prevent total off)
        factor = 0.1 + (brightness * 0.9)
        return (int(base_color[0] * factor), int(base_color[1] * factor), int(base_color[2] * factor))

    def run(self):
        print("[LED] Controller started with Multi-Pin support")
        
        last_time = time.time()
        phase = 0.0
        
        while not self.stop_event.is_set():
            state = self.get_state()
            now = time.time()
            dt = now - last_time
            last_time = now
            
            if not self.strips:
                time.sleep(1)
                continue

            # Update animation phase (0 to 1 cycle)
            phase = (phase + dt) % 2.0 # 2 second cycle for defaults

            # Apply any pending "clear" requests caused by state transitions.
            # This keeps all NeoPixel IO inside this thread, preventing crashes in rpi_ws281x.
            if self._clear_requested:
                with self.lock:
                    self._clear_requested = False
                self.clear_all()
            
            if state == self.STATE_DISARMED:
                # Breathing BLUE on left and right rings (only indices 0, 1, 6, 7)
                # Indices 2, 3, 4, 5 are constant 50% White
                breathe_phase = (now % 2.0) / 2.0
                ring_color = self._get_breathe_color(BLUE, breathe_phase)
                dim_white = (127, 127, 127)
                
                with self.strip_lock:
                    if self.left_ring:
                        for i in [0, 1, 6, 7]:
                            self.left_ring[i] = ring_color
                        for i in [2, 3, 4, 5]:
                            self.left_ring[i] = dim_white
                            
                    if self.right_ring:
                        for i in [0, 1, 6, 7]:
                            self.right_ring[i] = ring_color
                        for i in [2, 3, 4, 5]:
                            self.right_ring[i] = dim_white
                    
                    if self.eyes:
                        # Breathing BLUE on Front Eyes (first 11 pixels)
                        for i in range(11):
                            self.eyes[i] = ring_color
                        
                        # Breathing RED on Sides (next 4 pixels)
                        side_breathe = self._get_breathe_color(RED, breathe_phase)
                        for i in range(11, 15):
                            self.eyes[i] = side_breathe
                        
                        # Box LEDs breathe White (last 6 pixels)
                        box_breathe = self._get_breathe_color((255, 255, 255), breathe_phase)
                        for i in range(15, 21):
                            self.eyes[i] = box_breathe
                    
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)

            elif state in [self.STATE_ARMED_GROUND, self.STATE_FLYING]:
                # Flash Red (Left) and Green (Right) on indices 0, 1, 6, 7 - 2Hz (0.5s cycle)
                # Indices 2, 3, 4, 5 are constant 50% White
                is_on = (now % 0.5) < 0.25
                breathe_phase = (now % 2.0) / 2.0
                dim_white = (127, 127, 127)
                flash_left = RED if is_on else OFF
                flash_right = GREEN if is_on else OFF
                
                with self.strip_lock:
                    if self.left_ring:
                        for i in [0, 1, 6, 7]:
                            self.left_ring[i] = flash_left
                        for i in [2, 3, 4, 5]:
                            self.left_ring[i] = dim_white
                            
                    if self.right_ring:
                        for i in [0, 1, 6, 7]:
                            self.right_ring[i] = flash_right
                        for i in [2, 3, 4, 5]:
                            self.right_ring[i] = dim_white
                            
                    # Eyes & Sides: Keeping purple as flight indicator (flashing with flash rate)
                    if self.eyes:
                        eyes_color = PURPLE if is_on else OFF
                        for i in range(15):
                            self.eyes[i] = eyes_color
                    
                    # Box LEDs breathe White
                    if self.eyes:
                        box_breathe = self._get_breathe_color((255, 255, 255), breathe_phase)
                        for i in range(15, 21):
                            self.eyes[i] = box_breathe
                            
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)

            elif state in [self.STATE_PRECISION_LAND, self.STATE_LAND, self.STATE_RTL]:
                # Chaser Red (Left) and Green (Right)
                # 8 LEDs, cycle every 0.8s
                chaser_idx = int((now % 0.8) / 0.1) % 8
                breathe_phase = (now % 2.0) / 2.0
                
                with self.strip_lock:
                    if self.left_ring:
                        self.left_ring.fill(OFF)
                        self.left_ring[chaser_idx] = RED
                    if self.right_ring:
                        self.right_ring.fill(OFF)
                        self.right_ring[chaser_idx] = GREEN
                    if self.eyes:
                        # Steady yellow for landing/RTL indicators on Eyes & Sides
                        for i in range(15):
                            self.eyes[i] = YELLOW
                        # Box LEDs breathe White
                        box_breathe = self._get_breathe_color((255, 255, 255), breathe_phase)
                        for i in range(15, 21):
                            self.eyes[i] = box_breathe
                            
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)

            elif state == self.STATE_FAILSAFE:
                # Fast Flash RED on all - 5Hz
                is_on = (now % 0.2) < 0.1
                color = RED if is_on else OFF
                with self.strip_lock:
                    if self.left_ring: self.left_ring.fill(color)
                    if self.right_ring: self.right_ring.fill(color)
                    if self.eyes:
                        # Eyes, Sides, and Box flash RED
                        for i in range(21):
                            self.eyes[i] = color
                            
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.02)

            elif state == self.STATE_BAT_FAILSAFE:
                # Oscillate Red/Blue quickly
                is_red = (now % 0.4) < 0.2
                color = RED if is_red else BLUE
                with self.strip_lock:
                    if self.left_ring: self.left_ring.fill(color)
                    if self.right_ring: self.right_ring.fill(color)
                    if self.eyes:
                        # Eyes, Sides, and Box oscillate Red/Blue
                        for i in range(21):
                            self.eyes[i] = color
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)

            elif state == self.STATE_CRASH:
                # Solid RED on all (Crash indicator)
                with self.strip_lock:
                    if self.left_ring: self.left_ring.fill(RED)
                    if self.right_ring: self.right_ring.fill(RED)
                    if self.eyes:
                        # Eyes, Sides, and Box solid RED
                        for i in range(21):
                            self.eyes[i] = RED
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)
            
            else:
                self.clear_all()
                time.sleep(0.1)
                
    def stop(self):
        self.stop_event.set()
        self.join()
        self.clear_all()

# --- Buzzer Alarm Controller ---
ALARM_PIN = board.D23

class DroneAlarmController(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.stop_event = threading.Event()
        self.current_state = DroneLEDController.STATE_DISARMED
        self.current_alt = 0.0
        self.takeoff_until = 0.0
        self.buzzer_is_high = False
        self.next_toggle_time = 0.0
        self.last_buzzer_reason = None
        self.lock = threading.Lock()
        self.failsafe_start_time = 0.0
        
        try:
            import digitalio
            self.buzzer = digitalio.DigitalInOut(ALARM_PIN)
            self.buzzer.direction = digitalio.Direction.OUTPUT
            self.buzzer.value = False
        except Exception as e:
            print(f"[ALARM] Error initializing Buzzer on GPIO 23: {e}")
            self.buzzer = None

    def set_state(self, new_state):
        with self.lock:
            # Trigger takeoff buzzer for 5 seconds when transitioning to STATE_FLYING from GROUND/DISARMED
            if new_state == DroneLEDController.STATE_FLYING and self.current_state in [
                DroneLEDController.STATE_DISARMED,
                DroneLEDController.STATE_ARMED_GROUND
            ]:
                self.takeoff_until = time.time() + 5.0
                self._set_buzzer_reason("takeoff: transitioned from ground/disarmed to flying")
            
            # Reset takeoff timer if disarmed
            if new_state == DroneLEDController.STATE_DISARMED:
                self.takeoff_until = 0.0
                self._set_buzzer_reason(None)
                
            # If entering a failsafe state, record start time for a 10s maximum beeping cap
            if new_state in [DroneLEDController.STATE_FAILSAFE, DroneLEDController.STATE_BAT_FAILSAFE] and self.current_state not in [DroneLEDController.STATE_FAILSAFE, DroneLEDController.STATE_BAT_FAILSAFE]:
                self.failsafe_start_time = time.time()
                
            self.current_state = new_state

    def _set_buzzer_reason(self, reason):
        if reason == self.last_buzzer_reason:
            return
        self.last_buzzer_reason = reason
        if reason:
            print(f"[ALARM] Buzzer active: {reason}")
        else:
            print("[ALARM] Buzzer inactive")

    def get_state(self):
        with self.lock:
            return self.current_state

    def set_altitude(self, alt):
        with self.lock:
            self.current_alt = alt

    def get_altitude(self):
        with self.lock:
            return self.current_alt

    def run(self):
        print("[ALARM] Controller started on GPIO 23")
        while not self.stop_event.is_set():
            state = self.get_state()
            now = time.time()
            
            if not self.buzzer:
                time.sleep(1)
                continue

            # 1. Crash Alarm (Highest Priority)
            if state == DroneLEDController.STATE_CRASH:
                self._set_buzzer_reason("crash alarm state")
                cycle_time = 0.3
                is_high = (now % cycle_time) < 0.2
                self.buzzer.value = is_high
                self.buzzer_is_high = is_high
                self.next_toggle_time = 0.0
                time.sleep(0.05)

            # 2. Failsafe Alarms (capped at 10 seconds per event)
            elif state in [DroneLEDController.STATE_FAILSAFE, DroneLEDController.STATE_BAT_FAILSAFE]:
                failsafe_elapsed = now - self.failsafe_start_time
                if failsafe_elapsed < 10.0:
                    if state == DroneLEDController.STATE_BAT_FAILSAFE:
                        self._set_buzzer_reason("battery failsafe alarm active")
                    else:
                        self._set_buzzer_reason("failsafe alarm active")
                    cycle_time = 0.4
                    is_high = (now % cycle_time) < 0.1
                    self.buzzer.value = is_high
                    self.buzzer_is_high = is_high
                    self.next_toggle_time = 0.0
                    time.sleep(0.05)
                else:
                    self.buzzer.value = False
                    self.buzzer_is_high = False
                    self._set_buzzer_reason("failsafe alarm - beeping timed out (10s reached)")
                    time.sleep(0.1)

            # 3. Takeoff Sequence (5 seconds fast beep)
            elif self.takeoff_until > now:
                self._set_buzzer_reason("takeoff: 5 second fast beep window")
                cycle_time = 0.2
                is_high = (now % cycle_time) < 0.1
                self.buzzer.value = is_high
                self.buzzer_is_high = is_high
                self.next_toggle_time = 0.0
                time.sleep(0.05)

            # 4. Landing/RTL proximity beep (ALT < 4m, stops at < 0.8m)
            elif state in [DroneLEDController.STATE_PRECISION_LAND, DroneLEDController.STATE_LAND, DroneLEDController.STATE_RTL]:
                alt = self.get_altitude()
                if 0.8 <= alt < 4.0:
                    self._set_buzzer_reason(f"landing/RTL proximity: altitude {alt:.1f}m")
                    if now >= self.next_toggle_time:
                        self.buzzer_is_high = not self.buzzer_is_high
                        if self.buzzer_is_high:
                            # Fixed active beep duration
                            self.buzzer.value = True
                            self.next_toggle_time = now + 0.1
                        else:
                            # Dynamic gap length based on altitude
                            self.buzzer.value = False
                            clamped_alt = max(0.8, min(4.0, alt))
                            # Interpolate gap (low time) from 0.1s (at 0.8m) to 1.0s (at 4.0m)
                            low_time = 0.1 + (clamped_alt - 0.8) * (0.9 / 3.2)
                            self.next_toggle_time = now + low_time
                    time.sleep(0.01)
                else:
                    self.buzzer.value = False
                    self.buzzer_is_high = False
                    self.next_toggle_time = 0.0
                    self._set_buzzer_reason(None)
                    time.sleep(0.1)
                
            else:
                self.buzzer.value = False
                self.buzzer_is_high = False
                self.next_toggle_time = 0.0
                self._set_buzzer_reason(None)
                time.sleep(0.1)

    def stop(self):
        self.stop_event.set()
        if self.buzzer:
            self.buzzer.value = False
        self.join()

def run_test():
    # Helper to test the class independently
    print("Testing LED Controller (Adafruit NeoPixel)...")
    try:
        led = DroneLEDController()
        alarm = DroneAlarmController()
        led.start()
        alarm.start()
        
        try:
            print("Test: DISARMED (Breathing Blue)")
            led.set_state(DroneLEDController.STATE_DISARMED)
            alarm.set_state(DroneLEDController.STATE_DISARMED)
            time.sleep(4)
            
            print("Test: ARMED_GROUND (Flashing Red/Green)")
            led.set_state(DroneLEDController.STATE_ARMED_GROUND)
            time.sleep(4)
            
            print("Test: FAILSAFE (Fast Red Flash & Beep -___)")
            led.set_state(DroneLEDController.STATE_FAILSAFE)
            alarm.set_state(DroneLEDController.STATE_FAILSAFE)
            time.sleep(5)
            
            print("Test: CRASH (Beep --_--_)")
            led.set_state(DroneLEDController.STATE_CRASH)
            alarm.set_state(DroneLEDController.STATE_CRASH)
            time.sleep(5)
            
        except KeyboardInterrupt:
            print("Interrupted")
        finally:
            led.stop()
            alarm.stop()
            print("Done")
    except Exception as e:
        print(f"Test Failed: {e}")

if __name__ == "__main__":
    # If run directly, perform a test sequence
    # Need sudo to run this effectively on Pi
    run_test()
