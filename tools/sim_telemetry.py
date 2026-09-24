#!/usr/bin/env python3
"""Fake MAVLink telemetry pump for local testing of mavlink-gs.

Sends the four message types mavlink-gs decodes (HEARTBEAT, SYS_STATUS,
GLOBAL_POSITION_INT, ATTITUDE) to a UDP endpoint at ~10 Hz, with values that
move over time so the dashboard visibly updates.

This is NOT a real autopilot. It cannot answer commands (no COMMAND_ACK) or fly
a mission. Use it to prove the receive -> decode -> display pipeline works;
use PX4/ArduPilot SITL for realistic behavior.

Usage:
    .venv/bin/python tools/sim_telemetry.py                 # -> 127.0.0.1:14550
    .venv/bin/python tools/sim_telemetry.py 127.0.0.1:14550 # explicit target

Stop with Ctrl+C.
"""

import math
import sys
import time

from pymavlink import mavutil


def main() -> int:
    target = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:14550"

    # 'udpout' = act as a client that SENDS datagrams to `target`. mavlink-gs
    # binds that port and receives them. source_system=1 marks us as vehicle #1.
    link = mavutil.mavlink_connection(f"udpout:{target}", source_system=1)
    print(f"pumping fake telemetry -> udp://{target} at 10 Hz (Ctrl+C to stop)")

    start = time.time()
    while True:
        t = time.time() - start

        # HEARTBEAT: makes the link show CONNECTED and the vehicle show ARMED.
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,  # base_mode -> armed
            0,                                            # custom_mode
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

        # SYS_STATUS: power. Units are raw MAVLink: mV, cA, percent.
        voltage_mv = int(12000 + 400 * math.sin(t / 5.0))   # ~11.6-12.4 V
        current_ca = 850                                     # 8.50 A
        remaining = int(70 + 25 * math.sin(t / 12.0))       # 45-95 %
        link.mav.sys_status_send(
            0, 0, 0,            # sensors present/enabled/health (unused here)
            500,                # load, 0.1% units
            voltage_mv, current_ca, remaining,
            0, 0, 0, 0, 0, 0,   # comm drop / error counters
        )

        # GLOBAL_POSITION_INT: lat/lon in 1e7 deg, alt/rel_alt in mm. Drifts in
        # a small circle around a point in San Francisco.
        lat = int((37.7749 + 0.0002 * math.sin(t / 3.0)) * 1e7)
        lon = int((-122.4194 + 0.0002 * math.cos(t / 3.0)) * 1e7)
        link.mav.global_position_int_send(
            int(t * 1000),      # time_boot_ms
            lat, lon,
            100_000,            # alt AMSL, mm  -> 100.0 m
            25_000,             # relative alt, mm -> 25.0 m
            0, 0, 0,            # vx, vy, vz
            0,                  # heading
        )

        # ATTITUDE: radians. Gentle roll/pitch wobble, yaw sweeps a full circle.
        link.mav.attitude_send(
            int(t * 1000),
            0.3 * math.sin(t),          # roll
            0.2 * math.cos(t / 2.0),    # pitch
            (t * 0.2) % (2 * math.pi),  # yaw
            0, 0, 0,                    # body rates
        )

        time.sleep(0.1)  # 10 Hz
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nstopped")
