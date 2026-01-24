import time
import threading
from rpi_ws281x import *
import argparse

# LED strip configuration:
LED_COUNT      = 12      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin led connects to (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

# Colors
COLOR_RED    = Color(0, 255, 0) # GRB order for some strips, adjust if needed (usually Green, Red, Blue)
# Note: rpi_ws281x usually uses GRB or RGB. Let's define R, G, B helpers.
# Wait, Color(r, g, b) often maps to GRB on the hardware. 
# Safe bet: Color(red, green, blue) passed to the lib usually handles it, 
# BUT many strips are physically GRB.
# Let's assume standard WS2812B GRB format, where the library wrapper Color(r,g,b) produces
# a 24-bit integer. 
# If colors are swapped, we can fix later. 
# Standard assumption: Library takes Color(r,g,b).
# If strip is GRB: Color(r, g, b) might result in Red->Green on strip.
# Let's use helper assuming standard RGB int packing.

def make_color(r, g, b):
    return Color(r, g, b)

RED     = make_color(255, 0, 0)
GREEN   = make_color(0, 255, 0)
BLUE    = make_color(0, 0, 255)
YELLOW  = make_color(255, 255, 0)
PURPLE  = make_color(128, 0, 128)
OFF     = make_color(0, 0, 0)

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
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()
        
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
                print(f"[LED] Switching state: {self.current_state} -> {new_state}")
                self.current_state = new_state
                # Clear strip immediately on state change for clean transition? 
                # Or let next loop handle it. Let's clear to be safe.
                self.clear_strip()

    def get_state(self):
        with self.lock:
            return self.current_state

    def clear_strip(self):
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, OFF)
        self.strip.show()

    def set_all(self, color):
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
        self.strip.show()

    def set_segment(self, indices, color):
        for i in indices:
            self.strip.setPixelColor(i, color)

    def run(self):
        print("[LED] Controller started")
        while not self.stop_event.is_set():
            state = self.get_state()

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
                
                # Step 2: Direction Colors (Left Red, Right Green) - Blink once? 
                # "Blinking Left RED and Right Green"
                # Let's do a quick blink of direction colors
                self.set_segment(self.left_indices, RED)
                self.set_segment(self.right_indices, GREEN)
                self.set_segment(self.middle_indices, OFF)
                self.strip.show()
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
                 # mid interval -> assume standard blink speed?
                 for _ in range(2):
                     if self.get_state() != self.STATE_FAILSAFE: break
                     self.set_all(RED)
                     time.sleep(0.3)
                     self.set_all(OFF)
                     time.sleep(0.3)
                 
                 # pause a bit to make it "sometimes only" or just regular?
                 # "mid intervals sometimes only" -> Implies a pause between the double blinks
                 time.sleep(1.0) 

            elif state == self.STATE_FLYING:
                # Direction Colors: Blinking Left RED and Right GREEN
                # "Blinking Left RED and Right Green just like airplane"
                # Usually airplanes have solid nav lights and blinking strobes. 
                # User asked for "Blinking Left RED and Right Green".
                
                # ON
                self.set_segment(self.left_indices, RED)
                self.set_segment(self.right_indices, GREEN)
                self.set_segment(self.middle_indices, OFF)
                self.strip.show()
                time.sleep(0.5) # On duration
                
                # OFF
                self.set_all(OFF)
                time.sleep(0.5) # Off duration
            
            else:
                # Unknown state, default to OFF
                self.set_all(OFF)
                time.sleep(0.1)
                
    def stop(self):
        self.stop_event.set_()
        self.join()
        self.clear_strip()

def run_test():
    # Helper to test the class independently
    print("Testing LED Controller...")
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

if __name__ == "__main__":
    # If run directly, perform a test sequence
    # Need sudo to run this effectively on Pi
    run_test()
