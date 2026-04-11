#!/usr/bin/env python3
"""
NeoPixel LED Test Script — Single strip on GPIO 13 (8 LEDs, 4 per drone eye).
Breathing red-blue drone-eyes effect.

Usage:  sudo python3 test_led.py
        sudo python3 test_led.py --duration 20
        sudo python3 test_led.py --brightness 0.3
"""

import time
import math
import argparse
import board
import neopixel

# ===================== Configuration =====================
STRIP_PIN    = board.D13       # GPIO 13 — PWM1
NUM_LEDS     = 8               # Total LEDs on the strip
LEDS_PER_EYE = 4               # First 4 = left eye, last 4 = right eye
BRIGHTNESS   = 0.8
LED_ORDER    = neopixel.GRB    # WS2812B standard
MODE_DURATION = 5              # Default seconds per mode

OFF = (0, 0, 0)


# ===================== Test Runner =====================

class LEDTester:
    """Drives a single NeoPixel strip (two drone eyes) through visual modes."""

    def __init__(self, brightness=BRIGHTNESS):
        print("Initialising NeoPixel strip …")
        self.strip = neopixel.NeoPixel(
            STRIP_PIN, NUM_LEDS,
            brightness=brightness,
            auto_write=False,
            pixel_order=LED_ORDER,
        )
        print(f"  [OK] GPIO 13 — {NUM_LEDS} LEDs ({LEDS_PER_EYE} per eye)\n")

    # --------------- Low-level helpers ---------------

    def fill_eye(self, eye, color):
        """Fill one eye: eye 0 = LEDs 0..3, eye 1 = LEDs 4..7."""
        start = eye * LEDS_PER_EYE
        for i in range(start, start + LEDS_PER_EYE):
            self.strip[i] = color

    def fill_all(self, color):
        self.strip.fill(color)
        self.strip.show()

    def clear_all(self):
        self.fill_all(OFF)

    def show(self):
        self.strip.show()

    # =================== MODES ===================

    # --- Breathing Red-Blue: drone-eyes, opposite colours per eye, smooth swap at trough ---
    def mode_breathing_red_blue(self, duration):
        PERIOD = 3.0                          # seconds per single breath (dim → bright → dim)
        start = time.time()
        end   = start + duration
        while time.time() < end:
            elapsed = time.time() - start
            angle   = (elapsed / PERIOD) * 2.0 * math.pi
            # Brightness oscillates between 50 % (trough) and 100 % (peak)
            brightness = 0.75 - 0.25 * math.cos(angle)
            val = int(255 * brightness)
            # Colour flips at each trough — swap happens at dimmest point for seamless transition
            is_swapped = int(elapsed / PERIOD) % 2
            red_color  = (val, 0, 0)
            blue_color = (0, 0, val)
            # Eye 0 (LEDs 0-3) and Eye 1 (LEDs 4-7) always show opposite colours
            self.fill_eye(0, red_color  if is_swapped == 0 else blue_color)
            self.fill_eye(1, blue_color if is_swapped == 0 else red_color)
            self.show()
            time.sleep(0.02)

    # =================== Main Loop ===================

    def run(self, duration_per_mode):
        modes = [
            ("BREATHING RED-BLUE", self.mode_breathing_red_blue),
        ]

        banner = (
            "=" * 48 + "\n"
            "  NeoPixel LED Test — Drone Eyes\n"
            f"  Strip : GPIO 13  |  {NUM_LEDS} LEDs ({LEDS_PER_EYE} per eye)\n"
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
    parser = argparse.ArgumentParser(description="NeoPixel LED test — drone eyes on GPIO 13")
    parser.add_argument("--duration",   type=float, default=MODE_DURATION,
                        help="Seconds each mode runs (default: 5)")
    parser.add_argument("--brightness", type=float, default=BRIGHTNESS,
                        help="LED brightness 0.0–1.0 (default: 0.5)")
    args = parser.parse_args()

    tester = LEDTester(brightness=args.brightness)
    tester.run(args.duration)
