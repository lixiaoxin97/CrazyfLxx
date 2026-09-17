#!/usr/bin/env python3
"""
mock_vicon_sender.py

A lightweight fake Vicon/VRPN-like data source for CrazyfLxx development.

Purpose
-------
Use this script when the real Vicon system is not running. It generates
synthetic rigid-body data at a configurable rate and sends it over localhost
UDP as two JSON packets per frame:

1) position packet
   {
       "type": "position",
       "time": "...",
       "sensor": 0,
       "position": [x, y, z],
       "quaternion": [qx, qy, qz, qw]
   }

2) velocity packet
   {
       "type": "velocity",
       "time": "...",
       "sensor": 0,
       "velocity": [vx, vy, vz]
   }

The field semantics intentionally mirror the real callbacks observed from the
lab Vicon/VRPN setup.

Default state
-------------
position   = [0, 0, 1] m
attitude   = [roll, pitch, yaw] = [0, 0, 0] deg
velocity   = [0, 0, 0] m/s
rate       = 100 Hz

Modes
-----
static:
    Fixed pose, zero linear velocity.

yaw:
    Fixed position, constant yaw rotation.
    Useful for testing quaternion -> angular velocity code.

circle:
    Horizontal circular motion around the base position.
    Linear velocity is analytically consistent with the position.

Examples
--------
Static hover:
    python3 mock_vicon_sender.py

Constant yaw rotation at 45 deg/s:
    python3 mock_vicon_sender.py --mode yaw --yaw-rate-deg 45

Horizontal circle:
    python3 mock_vicon_sender.py --mode circle --radius 0.2 --period 4.0

Custom UDP destination:
    python3 mock_vicon_sender.py --host 127.0.0.1 --port 5005

Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from datetime import datetime


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    ZYX Euler angles [rad] -> quaternion [x, y, z, w].

    Rotation convention:
        R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]


def generate_state(args, elapsed):
    """
    Return:
        position [m]
        quaternion [x,y,z,w]
        linear velocity in world frame [m/s]
    """
    x0, y0, z0 = args.position

    roll = math.radians(args.roll_deg)
    pitch = math.radians(args.pitch_deg)
    yaw0 = math.radians(args.yaw_deg)

    if args.mode == "static":
        position = [x0, y0, z0]
        velocity = [0.0, 0.0, 0.0]
        yaw = yaw0

    elif args.mode == "yaw":
        yaw_rate = math.radians(args.yaw_rate_deg)
        yaw = yaw0 + yaw_rate * elapsed

        # Keep yaw numerically bounded. Quaternion itself would also remain valid
        # without this, but bounded angles make console output easier to inspect.
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        position = [x0, y0, z0]
        velocity = [0.0, 0.0, 0.0]

    elif args.mode == "circle":
        if args.period <= 0.0:
            raise ValueError("--period must be > 0")

        omega = 2.0 * math.pi / args.period
        theta = omega * elapsed

        position = [
            x0 + args.radius * math.cos(theta),
            y0 + args.radius * math.sin(theta),
            z0,
        ]

        velocity = [
            -args.radius * omega * math.sin(theta),
            +args.radius * omega * math.cos(theta),
            0.0,
        ]

        # Keep attitude fixed unless the user sets a non-zero base yaw.
        yaw = yaw0

    else:
        raise ValueError(f"Unknown mode: {args.mode}")

    quaternion = rpy_to_quaternion(roll, pitch, yaw)
    return position, quaternion, velocity


def fmt3(v, decimals=4):
    f = "{:+." + str(decimals) + "f}"
    return "[" + " ".join(f.format(float(x)) for x in v[:3]) + "]"


def fmt4(v, decimals=5):
    f = "{:+." + str(decimals) + "f}"
    return "[" + " ".join(f.format(float(x)) for x in v[:4]) + "]"


def main():
    parser = argparse.ArgumentParser(
        description="Fake Vicon state generator and UDP sender."
    )

    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="UDP destination host; default 127.0.0.1",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5005,
        help="UDP destination port; default 5005",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="synthetic Vicon update rate [Hz]; default 100",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=10.0,
        help="terminal print rate [Hz]; default 10",
    )
    parser.add_argument(
        "--sensor",
        type=int,
        default=0,
        help="sensor id; default 0",
    )

    parser.add_argument(
        "--mode",
        choices=("static", "yaw", "circle"),
        default="static",
        help="motion mode; default static",
    )

    parser.add_argument(
        "--position",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 1.0),
        help="base position [m]; default 0 0 1",
    )

    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)

    parser.add_argument(
        "--yaw-rate-deg",
        type=float,
        default=45.0,
        help="yaw rate for yaw mode [deg/s]; default 45",
    )

    parser.add_argument(
        "--radius",
        type=float,
        default=0.2,
        help="circle radius [m]; default 0.2",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=4.0,
        help="circle period [s]; default 4.0",
    )

    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    if args.print_rate <= 0.0:
        parser.error("--print-rate must be > 0")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.host, args.port)

    frame_period = 1.0 / args.rate
    print_period = 1.0 / args.print_rate

    start = time.monotonic()
    next_frame = start
    next_print = start

    print("Mock Vicon sender")
    print(f"destination : udp://{args.host}:{args.port}")
    print(f"sensor      : {args.sensor}")
    print(f"mode        : {args.mode}")
    print(f"rate        : {args.rate:.1f} Hz")
    print(f"base pos    : {tuple(args.position)} m")
    print("Ctrl+C to stop.\n")

    frame_count = 0

    try:
        while True:
            now = time.monotonic()

            if now < next_frame:
                time.sleep(min(next_frame - now, 0.001))
                continue

            elapsed = now - start
            position, quaternion, velocity = generate_state(args, elapsed)

            # Use exactly one timestamp for both packets, as observed from the
            # real Vicon/VRPN pose and velocity callbacks.
            stamp = datetime.now().isoformat(timespec="microseconds")

            position_packet = {
                "type": "position",
                "time": stamp,
                "sensor": args.sensor,
                "position": position,
                "quaternion": quaternion,
            }

            velocity_packet = {
                "type": "velocity",
                "time": stamp,
                "sensor": args.sensor,
                "velocity": velocity,
            }

            sock.sendto(
                json.dumps(position_packet, separators=(",", ":")).encode("utf-8"),
                destination,
            )
            sock.sendto(
                json.dumps(velocity_packet, separators=(",", ":")).encode("utf-8"),
                destination,
            )

            frame_count += 1

            if now >= next_print:
                print(
                    f"TX #{frame_count:7d} | "
                    f"p[m]={fmt3(position)} | "
                    f"q[x y z w]={fmt4(quaternion)} | "
                    f"v_world[m/s]={fmt3(velocity)}"
                )
                next_print += print_period

            # Advance by one scheduled frame rather than "now + period" to
            # reduce long-term drift.
            next_frame += frame_period

            # If the process was paused for a long time, resynchronize instead
            # of trying to send a huge burst of old frames.
            if now - next_frame > 0.5:
                next_frame = now + frame_period

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
