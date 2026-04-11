#!/usr/bin/env python3
"""
NeoPixel LED Test Script — Two strips on GPIO 18 & GPIO 23 (4 LEDs each).
Automatically cycles through multiple lighting modes on a timer.

Usage:  sudo python3 test_led.py
        sudo python3 test_led.py --duration 8   (seconds per mode)
        sudo python3 test_led.py --brightness 0.3

GPIO Note:
  GPIO 18 uses PWM0 — fully supported by rpi_ws281x.
  GPIO 23 is NOT a hardware-PWM/PCM/SPI pin. If your board or wiring
  routes the second strip to GPIO 13 (PWM1) or GPIO 10 (SPI) instead,
  update STRIP2_PIN below.  The script gracefully skips any strip that
  fails to initialise.
"""

import time
import math
import argparse
import board
import neopixel

# ===================== Configuration =====================
STRIP1_PIN  = board.D18        # GPIO 18 — PWM0 (hardware-supported)
STRIP2_PIN  = board.D13        # GPIO 23 — change to board.D13 / board.D10 if needed
NUM_LEDS    = 8                # LEDs per strip
BRIGHTNESS  = 0.5
LED_ORDER   = neopixel.GRB     # WS2812B standard
MODE_DURATION = 5              # Default seconds per mode

# ===================== Colour Palette =====================
RED     = (255, 0,   0)
GREEN   = (0,   255, 0)
BLUE    = (0,   0,   255)
WHITE   = (255, 255, 255)
YELLOW  = (255, 255, 0)
CYAN    = (0,   255, 255)
MAGENTA = (255, 0,   255)
ORANGE  = (255, 80,  0)
PURPLE  = (128, 0,   128)
OFF     = (0,   0,   0)


# --------------- Helpers ---------------

def wheel(pos):
    """Map a 0-255 position to an RGB rainbow colour."""
    pos = pos & 255
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)


def dim(color, factor):
    """Scale an RGB tuple by *factor* (0.0 – 1.0)."""
    return tuple(int(c * factor) for c in color)


def init_strip(pin, count, brightness):
    """Try to create a NeoPixel strip; return None on failure."""
    try:
        strip = neopixel.NeoPixel(
            pin, count,
            brightness=brightness,
            auto_write=False,
            pixel_order=LED_ORDER,
        )
        print(f"  [OK]   Strip on {pin} initialised ({count} LEDs)")
        return strip
    except Exception as e:
        print(f"  [SKIP] Strip on {pin}: {e}")
        return None


# ===================== Test Runner =====================

class LEDTester:
    """Drives two NeoPixel strips through a sequence of visual modes."""

    def __init__(self, brightness=BRIGHTNESS):
        print("Initialising NeoPixel strips …")
        self.strip1 = init_strip(STRIP1_PIN, NUM_LEDS, brightness)
        self.strip2 = init_strip(STRIP2_PIN, NUM_LEDS, brightness)
        self.strips = [s for s in (self.strip1, self.strip2) if s is not None]
        if not self.strips:
            raise RuntimeError("No strips could be initialised! "
                               "Run with sudo and check wiring/pin numbers.")
        print(f"{len(self.strips)} strip(s) ready.\n")

    # --------------- Low-level helpers ---------------

    def fill_all(self, color):
        for s in self.strips:
            s.fill(color)
            s.show()

    def clear_all(self):
        self.fill_all(OFF)

    def show_all(self):
        for s in self.strips:
            s.show()

    # =================== MODES ===================

    # --- 1. Static Colors: hold a fixed colour, rotate through the palette ---
    def mode_static(self, duration):
        palette = [RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, PURPLE, ORANGE]
        end = time.time() + duration
        idx = 0
        while time.time() < end:
            self.fill_all(palette[idx % len(palette)])
            time.sleep(1.0)
            idx += 1

    # --- 2. Blazing Red: sine-wave breathing/pulsing red glow ---
    def mode_blazing_red(self, duration):
        end = time.time() + duration
        while time.time() < end:
            # Smooth sine ramp between 10 % and 100 % intensity
            brightness = (math.sin(time.time() * 4.0) + 1.0) / 2.0
            brightness = 0.10 + brightness * 0.90
            r = int(255 * brightness)
            self.fill_all((r, 0, 0))
            time.sleep(0.02)

    # --- 3. Running Light: single bright pixel chases across the strip with a fading tail ---
    def mode_running_light(self, duration):
        total = NUM_LEDS
        end = time.time() + duration
        pos = 0
        while time.time() < end:
            for s in self.strips:
                for i in range(total):
                    # Distance from the "head" pixel, wrapping around
                    dist = (total + i - (pos % total)) % total
                    if dist == 0:
                        s[i] = WHITE
                    elif dist == 1:
                        s[i] = dim(CYAN, 0.45)
                    elif dist == 2:
                        s[i] = dim(BLUE, 0.15)
                    else:
                        s[i] = OFF
                s.show()
            pos += 1
            time.sleep(0.12)

    # --- 4. Rainbow: smooth colour-wheel gradient scrolling across all LEDs ---
    def mode_rainbow(self, duration):
        end = time.time() + duration
        offset = 0
        while time.time() < end:
            for s in self.strips:
                for i in range(NUM_LEDS):
                    pixel_pos = (i * 256 // NUM_LEDS + offset) & 255
                    s[i] = wheel(pixel_pos)
                s.show()
            offset = (offset + 2) & 255
            time.sleep(0.025)

    # --- 5. Color Wipe: fill LEDs one-by-one with a colour, then clear ---
    def mode_color_wipe(self, duration):
        wipe_colors = [RED, GREEN, BLUE, ORANGE, PURPLE]
        end = time.time() + duration
        cidx = 0
        while time.time() < end:
            color = wipe_colors[cidx % len(wipe_colors)]
            # Fill forward
            for i in range(NUM_LEDS):
                if time.time() >= end:
                    return
                for s in self.strips:
                    s[i] = color
                    s.show()
                time.sleep(0.15)
            time.sleep(0.3)
            # Wipe off in reverse
            for i in reversed(range(NUM_LEDS)):
                if time.time() >= end:
                    return
                for s in self.strips:
                    s[i] = OFF
                    s.show()
                time.sleep(0.10)
            time.sleep(0.15)
            cidx += 1

    # --- 6. Strobe: rapid white flash (use sparingly — very bright) ---
    def mode_strobe(self, duration):
        end = time.time() + duration
        while time.time() < end:
            self.fill_all(WHITE)
            time.sleep(0.04)
            self.fill_all(OFF)
            time.sleep(0.06)

    # --- 7. Alternating: the two strips swap between two colours ---
    def mode_alternating(self, duration):
        end = time.time() + duration
        toggle = False
        while time.time() < end:
            if self.strip1:
                self.strip1.fill(RED if toggle else BLUE)
                self.strip1.show()
            if self.strip2:
                self.strip2.fill(BLUE if toggle else RED)
                self.strip2.show()
            toggle = not toggle
            time.sleep(0.5)

    # --- 8. Breathing Rainbow: each LED breathes through the colour wheel ---
    def mode_breathing_rainbow(self, duration):
        end = time.time() + duration
        while time.time() < end:
            hue = int(time.time() * 40) & 255
            brightness = (math.sin(time.time() * 3.0) + 1.0) / 2.0
            brightness = 0.08 + brightness * 0.92
            base = wheel(hue)
            color = dim(base, brightness)
            self.fill_all(color)
            time.sleep(0.025)

    # --- 9. Ping-Pong: a single pixel bounces back and forth ---
    def mode_ping_pong(self, duration):
        end = time.time() + duration
        pos = 0
        direction = 1
        while time.time() < end:
            for s in self.strips:
                s.fill(OFF)
                s[pos] = GREEN
                s.show()
            pos += direction
            if pos >= NUM_LEDS - 1 or pos <= 0:
                direction *= -1
            time.sleep(0.14)

    # --- 10. Fire Flicker: randomised warm orange/red flicker ---
    def mode_fire_flicker(self, duration):
        import random
        end = time.time() + duration
        while time.time() < end:
            for s in self.strips:
                for i in range(NUM_LEDS):
                    flicker = random.randint(100, 255)
                    s[i] = (flicker, flicker // 4, 0)
                s.show()
            time.sleep(0.06)

    # =================== Main Loop ===================

    def run(self, duration_per_mode):
        modes = [
            ("STATIC COLORS",     self.mode_static),
            ("BLAZING RED",       self.mode_blazing_red),
            ("RUNNING LIGHT",     self.mode_running_light),
            ("RAINBOW",           self.mode_rainbow),
            ("COLOR WIPE",        self.mode_color_wipe),
            ("STROBE",            self.mode_strobe),
            ("ALTERNATING",       self.mode_alternating),
            ("BREATHING RAINBOW", self.mode_breathing_rainbow),
            ("PING PONG",         self.mode_ping_pong),
            ("FIRE FLICKER",      self.mode_fire_flicker),
        ]

        banner = (
            "=" * 48 + "\n"
            "  NeoPixel LED Test\n"
            f"  Strips  : GPIO 18 & GPIO 23  |  {NUM_LEDS} LEDs each\n"
            f"  Duration: {duration_per_mode}s per mode  |  "
            f"{len(modes)} modes\n"
            "=" * 48
        )
        print(banner)

        try:
            cycle = 1
            while True:
                print(f"\n─── Cycle {cycle} ───")
                for name, func in modes:
                    print(f"  ▸ {name}")
                    func(duration_per_mode)
                    self.clear_all()
                    time.sleep(0.4)
                cycle += 1

        except KeyboardInterrupt:
            print("\n\nInterrupted — shutting down.")
        finally:
            self.clear_all()
            print("LEDs cleared. Done.")


# ===================== Entry Point =====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NeoPixel LED test on GPIO 18 & 23")
    parser.add_argument("--duration",   type=float, default=MODE_DURATION,
                        help="Seconds each mode runs (default: 5)")
    parser.add_argument("--brightness", type=float, default=BRIGHTNESS,
                        help="LED brightness 0.0–1.0 (default: 0.5)")
    args = parser.parse_args()

    tester = LEDTester(brightness=args.brightness)
    tester.run(args.duration)
