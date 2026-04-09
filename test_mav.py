"""
test_mav.py - Simple DroneKit serial connection test.
Lists available serial ports, lets you pick one,
connects at 115200 baud, and prints basic vehicle telemetry.
"""

import sys
import time
import serial.tools.list_ports
from dronekit import connect, VehicleMode


def list_serial_ports():
    """Return a list of available serial port device names."""
    ports = serial.tools.list_ports.comports()
    return sorted(ports, key=lambda p: p.device)


def select_port():
    """Display available serial ports and let the user choose one."""
    ports = list_serial_ports()

    if not ports:
        print("[ERROR] No serial ports detected.")
        sys.exit(1)

    print("\n===== Available Serial Ports =====")
    for idx, port in enumerate(ports):
        print(f"  [{idx}]  {port.device}  -  {port.description}")
    print("==================================\n")

    while True:
        choice = input(f"Select port number [0-{len(ports) - 1}]: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            selected = ports[int(choice)].device
            print(f"\n>> Selected: {selected}\n")
            return selected
        print("  Invalid selection, try again.")


def main():
    port = select_port()
    baud = 115200
    connection_string = port

    print(f"Connecting to vehicle on {connection_string} @ {baud} baud …")

    try:
        vehicle = connect(connection_string, baud=baud, wait_ready=True, heartbeat_timeout=30)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        sys.exit(1)

    # --- Print basic telemetry ---
    print("\n========== Vehicle Connected ==========")
    print(f"  Firmware  : {vehicle.version}")
    print(f"  Mode      : {vehicle.mode.name}")
    print(f"  Armed     : {vehicle.armed}")
    print(f"  System    : {vehicle.system_status.state}")
    print(f"  GPS       : {vehicle.gps_0}")
    print(f"  Battery   : {vehicle.battery}")
    print(f"  EKF OK    : {vehicle.ekf_ok}")
    print(f"  Heading   : {vehicle.heading}°")
    if vehicle.location.global_frame:
        loc = vehicle.location.global_frame
        print(f"  Location  : {loc.lat:.7f}, {loc.lon:.7f}, Alt {loc.alt:.1f} m")
    print("========================================\n")

    # --- Live telemetry loop (Ctrl+C to stop) ---
    print("Streaming attitude data… (Ctrl+C to stop)\n")
    try:
        while True:
            att = vehicle.attitude
            print(
                f"  Roll: {att.roll:+7.2f}  Pitch: {att.pitch:+7.2f}  Yaw: {att.yaw:+7.2f}",
                end="\r",
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nStopping telemetry stream.")

    # --- Cleanup ---
    print("Closing vehicle connection …")
    vehicle.close()
    print("Done.")


if __name__ == "__main__":
    main()
