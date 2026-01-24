import time
import threading
import board
import neopixel

# LED strip configuration:
LED_COUNT      = 12      # Number of LED pixels.
LED_PIN        = board.D18 # GPIO pin led connects to (18 uses PWM!).
LED_BRIGHTNESS = 0.5     # Set to 0.0 to 1.0
LED_ORDER      = neopixel.GRB # Standard for WS2812B

# Colors (R, G, B) Tuples
RED     = (255, 0, 0)
GREEN   = (0, 255, 0)
BLUE    = (0, 0, 255)
YELLOW  = (255, 255, 0)
PURPLE  = (128, 0, 128)
OFF     = (0, 0, 0)

class DroneLEDController(threading.Thread):
    # States
    STATE_DISARMED     = "DISARMED"
    STATE_ARMED_GROUND = "ARMED_GROUND"
    STATE_FLYING       = "FLYING" # Default/Direction Colors
    STATE_LAND         = "LAND"
    STATE_RTL          = "RTL"
    STATE_BAT_FAILSAFE = "BAT_FAILSAFE"
    STATE_FAILSAFE     = "FAILSAFE"

    def __init__(self):
        super().__init__()
        self.daemon = True
        self.stop_event = threading.Event()
        self.current_state = self.STATE_DISARMED
        self.lock = threading.Lock()
        
        # Initialize Strip
        # Using the logic confirmed by user in test_leds.py
        try:
            self.strip = neopixel.NeoPixel(
                LED_PIN, 
                LED_COUNT, 
                brightness=LED_BRIGHTNESS, 
                auto_write=False, 
                pixel_order=LED_ORDER
            )
        except Exception as e:
            print(f"[LED] Error initializing NeoPixel: {e}")
            print("[LED] Ensure you are running with root privileges (sudo)")
            # We don't raise here to prevent crashing main thread, 
            # but LEDs won't work.
            self.strip = None
            
        # Segment definitions
        # Left: 0,1,2 (3 LEDs)
        # Middle: 3,4,5,6,7,8 (6 LEDs)
        # Right: 9,10,11 (3 LEDs)
        self.left_indices = [0, 1, 2]
        self.middle_indices = [3, 4, 5, 6, 7, 8]
        self.right_indices = [9, 10, 11]

    def set_state(self, new_state):
        with self.lock:
            if self.current_state != new_state:
                # print(f"[LED] Switching state: {self.current_state} -> {new_state}")
                self.current_state = new_state
                self.clear_strip()

    def get_state(self):
        with self.lock:
            return self.current_state

    def clear_strip(self):
        if self.strip:
            self.strip.fill(OFF)
            self.strip.show()

    def set_all(self, color):
        if self.strip:
            self.strip.fill(color)
            self.strip.show()

    def set_segment(self, indices, color):
        if self.strip:
            for i in indices:
                if i < LED_COUNT:
                    self.strip[i] = color

    def show(self):
        if self.strip:
            self.strip.show()

    def run(self):
        print("[LED] Controller started")
        if not self.strip:
             print("[LED] No strip detected, thread running no-op")
        
        while not self.stop_event.is_set():
            state = self.get_state()
            
            if not self.strip:
                time.sleep(1)
                continue

            if state == self.STATE_DISARMED:
                # Solid PURPLE
                self.set_all(PURPLE)
                time.sleep(0.1)

            elif state == self.STATE_ARMED_GROUND:
                # Solid GREEN
                self.set_all(GREEN)
                time.sleep(0.1)

            elif state == self.STATE_LAND:
                # Solid RED
                self.set_all(RED)
                time.sleep(0.1)
                
            elif state == self.STATE_RTL:
                # Blinking between YELLOW -> Direction Colors -> YELLOW
                # Step 1: Yellow
                self.set_all(YELLOW)
                time.sleep(0.5)
                if self.get_state() != self.STATE_RTL: continue
                self.set_all(OFF)
                time.sleep(0.1)
                
                # Step 2: Direction Colors (Left Red, Right Green)
                self.set_segment(self.left_indices, RED)
                self.set_segment(self.right_indices, GREEN)
                self.set_segment(self.middle_indices, OFF)
                self.show()
                time.sleep(0.5)
                if self.get_state() != self.STATE_RTL: continue
                self.set_all(OFF)
                time.sleep(0.1)

            elif state == self.STATE_BAT_FAILSAFE:
                 # Blink 3 times RED and 1 time Blue and then pause for 2 seconds
                 
                 # 3x RED
                 for _ in range(3):
                     if self.get_state() != self.STATE_BAT_FAILSAFE: break
                     self.set_all(RED)
                     time.sleep(0.2)
                     self.set_all(OFF)
                     time.sleep(0.2)
                 
                 if self.get_state() != self.STATE_BAT_FAILSAFE: continue

                 # 1x BLUE
                 self.set_all(BLUE)
                 time.sleep(0.2)
                 self.set_all(OFF)
                 
                 if self.get_state() != self.STATE_BAT_FAILSAFE: continue

                 # Pause 2s
                 time.sleep(2.0)

            elif state == self.STATE_FAILSAFE:
                 # Blink 2 times red in mid intervals
                 for _ in range(2):
                     if self.get_state() != self.STATE_FAILSAFE: break
                     self.set_all(RED)
                     time.sleep(0.3)
                     self.set_all(OFF)
                     time.sleep(0.3)
                 
                 # pause
                 time.sleep(1.0) 

            elif state == self.STATE_FLYING:
                # Direction Colors: Blinking Left RED and Right GREEN in 3-LED segments
                
                # ON
                self.set_segment(self.left_indices, RED)
                self.set_segment(self.right_indices, GREEN)
                self.set_segment(self.middle_indices, OFF)
                self.show()
                time.sleep(0.5) # On duration
                
                # OFF
                self.set_all(OFF)
                time.sleep(0.5) # Off duration
            
            else:
                # Unknown state, default to OFF
                self.set_all(OFF)
                time.sleep(0.1)
                
    def stop(self):
        self.stop_event.set()
        self.join()
        self.clear_strip()

def run_test():
    # Helper to test the class independently
    print("Testing LED Controller (Adafruit NeoPixel)...")
    try:
        led = DroneLEDController()
        led.start()
        
        try:
            print("Test: DISARMED (Purple)")
            led.set_state(DroneLEDController.STATE_DISARMED)
            time.sleep(3)
            
            print("Test: ARMED_GROUND (Green)")
            led.set_state(DroneLEDController.STATE_ARMED_GROUND)
            time.sleep(3)
            
            print("Test: FLYING (Left Red, Right Green Blink)")
            led.set_state(DroneLEDController.STATE_FLYING)
            time.sleep(5)
            
            print("Test: LAND (Solid Red)")
            led.set_state(DroneLEDController.STATE_LAND)
            time.sleep(3)
            
            print("Test: RTL (Yellow -> Direction -> Yellow)")
            led.set_state(DroneLEDController.STATE_RTL)
            time.sleep(6)
            
            print("Test: BAT_FAILSAFE (3x Red, 1x Blue, Pause)")
            led.set_state(DroneLEDController.STATE_BAT_FAILSAFE)
            time.sleep(8)
            
            print("Test: FAILSAFE (2x Red)")
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
