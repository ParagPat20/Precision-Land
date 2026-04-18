import time
import threading
import board
import neopixel

# LED strip configuration
S1_PIN         = board.D13    # Left Ring
S2_PIN         = board.D18    # Right Ring
S3_PIN         = board.D12    # Eyes
RING_COUNT     = 8
EYE_COUNT      = 8            # Total pixels — left 0..3, right 4..7
LED_BRIGHTNESS = 0.9          # Set to 0.0 to 1.0
LED_ORDER      = neopixel.GRB  # Standard for WS2812B

# Colors (R, G, B) Tuples
RED     = (255, 0, 0)
GREEN   = (0, 255, 0)
BLUE    = (0, 0, 255)
YELLOW  = (255, 255, 0)
PURPLE  = (128, 0, 128)
OFF     = (0, 0, 0)

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
            
        # Segments for Eyes
        self.eye_left_indices = [0, 1, 2, 3]
        self.eye_right_indices = [4, 5, 6, 7]

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
                # Breathing BLUE on everything
                breathe_phase = (now % 2.0) / 2.0
                color = self._get_breathe_color(BLUE, breathe_phase)
                with self.strip_lock:
                    for strip in self.strips:
                        if strip:
                            strip.fill(color)
                    for strip in self.strips:
                        if strip:
                            strip.show()
                time.sleep(0.05)

            elif state in [self.STATE_ARMED_GROUND, self.STATE_FLYING]:
                # Flash Red (Left) and Green (Right) - 2Hz (0.5s cycle)
                is_on = (now % 0.5) < 0.25
                with self.strip_lock:
                    if is_on:
                        if self.left_ring: self.left_ring.fill(RED)
                        if self.right_ring: self.right_ring.fill(GREEN)
                        # Eyes: Keeping purple as flight indicator
                        if self.eyes: self.eyes.fill(PURPLE)
                    else:
                        if self.left_ring: self.left_ring.fill(OFF)
                        if self.right_ring: self.right_ring.fill(OFF)
                        if self.eyes: self.eyes.fill(OFF)
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)

            elif state in [self.STATE_PRECISION_LAND, self.STATE_LAND, self.STATE_RTL]:
                # Chaser Red (Left) and Green (Right)
                # 8 LEDs, cycle every 0.8s
                chaser_idx = int((now % 0.8) / 0.1) % 8
                
                with self.strip_lock:
                    if self.left_ring:
                        self.left_ring.fill(OFF)
                        self.left_ring[chaser_idx] = RED
                    if self.right_ring:
                        self.right_ring.fill(OFF)
                        self.right_ring[chaser_idx] = GREEN
                    if self.eyes:
                        # Steady yellow for landing/RTL indicators
                        self.eyes.fill(YELLOW)
                    if self.left_ring: self.left_ring.show()
                    if self.right_ring: self.right_ring.show()
                    if self.eyes: self.eyes.show()
                time.sleep(0.05)

            elif state == self.STATE_FAILSAFE:
                # Fast Flash RED on all - 5Hz
                is_on = (now % 0.2) < 0.1
                color = RED if is_on else OFF
                with self.strip_lock:
                    for strip in self.strips:
                        if strip:
                            strip.fill(color)
                    for strip in self.strips:
                        if strip:
                            strip.show()
                time.sleep(0.02)

            elif state == self.STATE_BAT_FAILSAFE:
                # Oscillate Red/Blue quickly
                is_red = (now % 0.4) < 0.2
                color = RED if is_red else BLUE
                with self.strip_lock:
                    for strip in self.strips:
                        if strip:
                            strip.fill(color)
                    for strip in self.strips:
                        if strip:
                            strip.show()
                time.sleep(0.05)
            
            else:
                self.clear_all()
                time.sleep(0.1)
                
    def stop(self):
        self.stop_event.set()
        self.join()
        self.clear_all()

def run_test():
    # Helper to test the class independently
    print("Testing LED Controller (Adafruit NeoPixel)...")
    try:
        led = DroneLEDController()
        led.start()
        
        try:
            print("Test: DISARMED (Breathing Blue)")
            led.set_state(DroneLEDController.STATE_DISARMED)
            time.sleep(4)
            
            print("Test: ARMED_GROUND (Flashing Red/Green)")
            led.set_state(DroneLEDController.STATE_ARMED_GROUND)
            time.sleep(4)
            
            print("Test: FLYING (Flashing Red/Green)")
            led.set_state(DroneLEDController.STATE_FLYING)
            time.sleep(4)
            
            print("Test: PRECISION_LAND (Chaser Red/Green)")
            led.set_state(DroneLEDController.STATE_PRECISION_LAND)
            time.sleep(5)
            
            print("Test: LAND (Chaser Red/Green)")
            led.set_state(DroneLEDController.STATE_LAND)
            time.sleep(3)
            
            print("Test: RTL (Chaser Red/Green)")
            led.set_state(DroneLEDController.STATE_RTL)
            time.sleep(3)
            
            print("Test: BAT_FAILSAFE (Red/Blue Oscillate)")
            led.set_state(DroneLEDController.STATE_BAT_FAILSAFE)
            time.sleep(5)
            
            print("Test: FAILSAFE (Fast Red Flash)")
            led.set_state(DroneLEDController.STATE_FAILSAFE)
            time.sleep(5)
            
        except KeyboardInterrupt:
            print("Interrupted")
        finally:
            led.stop()
            print("Done")
    except Exception as e:
        print(f"Test Failed: {e}")

if __name__ == "__main__":
    # If run directly, perform a test sequence
    # Need sudo to run this effectively on Pi
    run_test()
