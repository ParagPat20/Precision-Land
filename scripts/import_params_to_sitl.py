#!/usr/bin/env python3
"""
Import Parameters from DataFlash Log (.BIN) into SITL ArduPilot Drone
Extracts all logged parameters from a .BIN file and sends them to the running SITL instance via MAVLink.
"""

import os
import sys
import time
import argparse
from pymavlink import mavutil

# Parameters that should not be overwritten in SITL
SKIP_PARAMS = {
    "SYSID_THISMAV",
    "FORMAT_VERSION",
}

def extract_params_from_bin(bin_path):
    print(f"[EXTRACT] Reading DataFlash log: {bin_path} ...", flush=True)
    if not os.path.exists(bin_path):
        raise FileNotFoundError(f"Log file not found: {bin_path}")

    mlog = mavutil.mavlink_connection(bin_path)
    params = {}
    while True:
        msg = mlog.recv_msg()
        if msg is None:
            break
        if msg.get_type() == 'PARM':
            name = msg.Name.strip() if hasattr(msg, 'Name') else getattr(msg, 'Param', '')
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='ignore').strip('\x00')
            val = float(msg.Value)
            params[name] = val

    print(f"[EXTRACT] Successfully extracted {len(params)} unique parameters.", flush=True)
    return params

def import_params_to_sitl(params, connect_str, arming_check_zero=False):
    print(f"[MAVLINK] Connecting to SITL at {connect_str} ...", flush=True)
    master = mavutil.mavlink_connection(connect_str)
    master.wait_heartbeat(timeout=10)
    print(f"[MAVLINK] Connected! Target System={master.target_system}, Component={master.target_component}", flush=True)

    total = len(params)
    success_count = 0
    failed_params = []
    
    t_start = time.time()
    
    for idx, (param_name, param_val) in enumerate(params.items(), 1):
        if param_name in SKIP_PARAMS:
            continue
            
        if arming_check_zero and param_name == "ARMING_CHECK":
            param_val = 0.0

        pid = param_name.encode('ascii', errors='ignore')[:16].ljust(16, b'\x00')

        # Send PARAM_SET
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            pid,
            float(param_val),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        # Wait briefly for acknowledgment
        confirmed = False
        t0 = time.time()
        while time.time() - t0 < 0.08:
            msg = master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg:
                resp_name = msg.param_id.strip('\x00')
                if resp_name == param_name:
                    confirmed = True
                    break
            time.sleep(0.005)

        if confirmed:
            success_count += 1
        else:
            # Retry once
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                pid,
                float(param_val),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            t1 = time.time()
            while time.time() - t1 < 0.08:
                msg = master.recv_match(type='PARAM_VALUE', blocking=False)
                if msg and msg.param_id.strip('\x00') == param_name:
                    confirmed = True
                    break
                time.sleep(0.005)

            if confirmed:
                success_count += 1
            else:
                failed_params.append(param_name)

        if idx % 50 == 0 or idx == total:
            pct = (idx / total) * 100
            print(f"[{idx:4d}/{total}] ({pct:5.1f}%) Verified: {success_count} | Elapsed: {time.time()-t_start:.1f}s", flush=True)

    elapsed = time.time() - t_start
    print(f"\n[SUMMARY] Import completed in {elapsed:.1f}s.")
    print(f"  Total Params: {total}")
    print(f"  Successfully Verified: {success_count}")
    if failed_params:
        print(f"  Unconfirmed/Hardware-specific: {len(failed_params)} (e.g. {failed_params[:5]})")

    # Check key parameters in SITL
    print("\n[VERIFICATION] Checking key vehicle parameters in SITL:")
    check_keys = ["FRAME_CLASS", "FRAME_TYPE", "PLND_ENABLED", "PLND_TYPE", "ATC_ACCEL_P_MAX", "MOT_THST_EXPO", "BATT_CAPACITY"]
    for k in check_keys:
        pid = k.encode('ascii')[:16].ljust(16, b'\x00')
        master.mav.param_set_send(master.target_system, master.target_component, pid, float(params.get(k, 0)), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t_chk = time.time()
        while time.time() - t_chk < 0.3:
            msg = master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg and msg.param_id.strip('\x00') == k:
                print(f"  {k} = {msg.param_value} (from log: {params.get(k, 'N/A')})")
                break
            time.sleep(0.01)

    master.close()
    return success_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Import parameters from DataFlash log (.BIN) into SITL.")
    parser.add_argument("--bin", default=r"w:\JECH_UI\Precision-land\00000021.BIN", help="Path to .BIN log file")
    parser.add_argument("--connect", default="tcp:127.0.0.1:5763", help="SITL MAVLink connection string (default: tcp:127.0.0.1:5763)")
    parser.add_argument("--bypass-arming-check", action="store_true", help="Set ARMING_CHECK=0 so SITL can arm without physical sensors/RC")
    args = parser.parse_args()

    params = extract_params_from_bin(args.bin)
    import_params_to_sitl(params, args.connect, arming_check_zero=args.bypass_arming_check)
