#!/usr/bin/env python3
"""
Test/Sample Buzzer Control Script for JeCH AeroTECH
Uses board and digitalio (Adafruit Blinka / CircuitPython) to toggle GPIO 23,
which controls the Piezo Active Buzzer with MOSFET.

Includes interactive testing for all drone flight buzzer patterns:
1. Takeoff Alarm Beeps (-_-_-_-_-_-_-)
2. Failsafe Alarm Beeps (-___-___-___-___)
3. Crash Alarm Beeps ( .--_--_--_--_ )
4. Landing Proximity Beeps (Gap size scales with simulated altitude)
"""

import sys
import time
import threading

# Configuration Constants
BUZZER_GPIO_PIN = 23  # GPIO 23 / board.D23

try:
    import board
    import digitalio
    BUZZER_AVAILABLE = True
except ImportError:
    BUZZER_AVAILABLE = False
    print("Warning: CircuitPython libraries ('board' / 'digitalio') not detected.")
    print("Running in SIMULATION mode with stdout printing instead of physical GPIO toggling.\n")

class SampleBuzzerController:
    def __init__(self, pin_num=BUZZER_GPIO_PIN):
        self.pin_num = pin_num
        self.buzzer = None
        self._active_thread = None
        self._stop_event = threading.Event()

        if BUZZER_AVAILABLE:
            try:
                # Map pin number dynamically
                pin_attr = f"D{pin_num}"
                if hasattr(board, pin_attr):
                    pin = getattr(board, pin_attr)
                else:
                    raise AttributeError(f"Pin D{pin_num} not found on this board mapping.")
                
                self.buzzer = digitalio.DigitalInOut(pin)
                self.buzzer.direction = digitalio.Direction.OUTPUT
                self.buzzer.value = False
                print(f"[BUZZER] Successfully initialized hardware buzzer on GPIO {pin_num}")
            except Exception as e:
                print(f"[BUZZER] Error initializing hardware: {e}")
                print("[BUZZER] Falling back to SIMULATION mode.")
                self.buzzer = None

    def set_value(self, state: bool):
        """Set the buzzer to HIGH (True) or LOW (False)."""
        if self.buzzer:
            self.buzzer.value = state
        else:
            # Console simulation output
            if state:
                sys.stdout.write("BEEP! ")
                sys.stdout.flush()

    def stop_active_sequence(self):
        """Halts any currently running background beeping thread."""
        self._stop_event.set()
        if self._active_thread and self._active_thread.is_alive():
            self._active_thread.join(timeout=1.0)
        self._stop_event.clear()
        self.set_value(False)

    def trigger_takeoff(self, duration_sec=5.0):
        """
        Takeoff buzzer pattern: Fast, rapid beeps for 5 seconds.
        Pattern: -_-_-_-_-_-_-
        """
        self.stop_active_sequence()
        print(f"\n[BUZZER] Starting Takeoff beeps for {duration_sec}s...")
        
        def run():
            end_time = time.time() + duration_sec
            cycle_time = 0.2  # 200ms total cycle
            while time.time() < end_time and not self._stop_event.is_set():
                now = time.time()
                is_high = (now % cycle_time) < 0.1  # 100ms high, 100ms low
                self.set_value(is_high)
                time.sleep(0.02)
            self.set_value(False)
            print("\n[BUZZER] Takeoff sequence complete.")

        self._active_thread = threading.Thread(target=run, daemon=True)
        self._active_thread.start()

    def trigger_failsafe(self):
        """
        Failsafe buzzer pattern: Slow, long beeps.
        Pattern: -___-___-___-___
        """
        self.stop_active_sequence()
        print("\n[BUZZER] Starting Failsafe alarm. Press Enter or choose another option to stop...")

        def run():
            cycle_time = 0.8  # 800ms total cycle
            while not self._stop_event.is_set():
                now = time.time()
                is_high = (now % cycle_time) < 0.2  # 200ms high, 600ms low
                self.set_value(is_high)
                time.sleep(0.02)

        self._active_thread = threading.Thread(target=run, daemon=True)
        self._active_thread.start()

    def trigger_crash(self):
        """
        Crash buzzer pattern: Urgent double/stutter beeps.
        Pattern: .--_--_--_--_
        """
        self.stop_active_sequence()
        print("\n[BUZZER] Starting Crash alarm. Press Enter or choose another option to stop...")

        def run():
            cycle_time = 0.3  # 300ms total cycle
            while not self._stop_event.is_set():
                now = time.time()
                is_high = (now % cycle_time) < 0.2  # 200ms high, 100ms low
                self.set_value(is_high)
                time.sleep(0.02)

        self._active_thread = threading.Thread(target=run, daemon=True)
        self._active_thread.start()

    def trigger_landing_proximity(self, start_alt=4.0, stop_alt=0.8, steps=10):
        """
        Landing buzzer pattern: Alt < 4m, speed of beeping increases as alt drops.
        Once Alt < 0.8m, the buzzer stops.
        Pattern: -______-____-___-__-_-
        """
        self.stop_active_sequence()
        print("\n[BUZZER] Starting Landing proximity simulation...")

        def run():
            # Interpolate simulated landing from start_alt down to stop_alt over 15 seconds
            total_duration = 15.0
            start_time = time.time()
            buzzer_is_high = False
            next_toggle_time = 0.0
            
            while not self._stop_event.is_set():
                now = time.time()
                elapsed = now - start_time
                if elapsed >= total_duration:
                    break
                
                # Calculate simulated current altitude decreasing linearly
                progress = elapsed / total_duration
                current_alt = start_alt - (start_alt - stop_alt) * progress
                
                # Below 0.8m the buzzer stops
                if current_alt < 0.8:
                    print(f"\n[BUZZER] Altitude {current_alt:.2f}m is < 0.8m. Stopping.")
                    break
                
                if now >= next_toggle_time:
                    buzzer_is_high = not buzzer_is_high
                    if buzzer_is_high:
                        self.set_value(True)
                        next_toggle_time = now + 0.1  # 100ms beep duration
                    else:
                        self.set_value(False)
                        # Interp low time from 0.1s (at 0.8m) to 1.0s (at 4.0m)
                        low_time = 0.1 + (current_alt - 0.8) * (0.9 / (4.0 - 0.8))
                        next_toggle_time = now + low_time
                        
                sys.stdout.write(f"\rSimulating descent: Altitude = {current_alt:.2f}m")
                sys.stdout.flush()
                time.sleep(0.01)
                
            self.set_value(False)
            print("\n[BUZZER] Landing proximity complete.")

        self._active_thread = threading.Thread(target=run, daemon=True)
        self._active_thread.start()

def main():
    print("====================================================")
    print("         JECH AEROTECH BUZZER TEST INTERFACE        ")
    print("====================================================")
    print(f"GPIO PIN Selected: {BUZZER_GPIO_PIN}")
    
    controller = SampleBuzzerController(BUZZER_GPIO_PIN)
    
    try:
        while True:
            print("\nChoose an option to test:")
            print("1. Takeoff sequence (Fast 5s beep -_-_-_-_-_-_-) ")
            print("2. Failsafe Alarm (Slow beeps -___-___-___-___)")
            print("3. Crash Alarm (Urgent stutter .--_--_--_--_)")
            print("4. Landing Proximity (Varying gap -______-____-___-__-_-)")
            print("5. Stop current beeping")
            print("q. Quit")
            
            choice = input("\nEnter your choice (1-5, q): ").strip().lower()
            
            if choice == '1':
                controller.trigger_takeoff(5.0)
            elif choice == '2':
                controller.trigger_failsafe()
            elif choice == '3':
                controller.trigger_crash()
            elif choice == '4':
                controller.trigger_landing_proximity()
            elif choice in ['5', 's']:
                controller.stop_active_sequence()
                print("[BUZZER] Halted all sequences.")
            elif choice == 'q':
                controller.stop_active_sequence()
                print("Exiting...")
                break
            else:
                print("Invalid option. Please try again.")
    except KeyboardInterrupt:
        controller.stop_active_sequence()
        print("\nExiting...")

if __name__ == '__main__':
    main()
