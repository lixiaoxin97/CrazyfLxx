#!/usr/bin/env python3
"""
crazyflie_axis_test.py

Small one-axis body-rate sign test for CrazyfLxx.

Examples
--------
Dry-run:
    python3 crazyflie_axis_test.py --axis roll

Live:
    python3 crazyflie_axis_test.py --axis roll --live
    python3 crazyflie_axis_test.py --axis pitch --live
    python3 crazyflie_axis_test.py --axis yaw --live

Negative direction:
    python3 crazyflie_axis_test.py --axis roll --sign -1 --live

Defaults
--------
body-rate magnitude : 10 deg/s
collective thrust   : 3.0 m/s^2
duration            : 0.4 s
command rate        : 50 Hz
URI                 : radio://0/100/2M

Only one body-rate axis is non-zero at a time.
"""

import argparse
import time

from crazyflie_interface import CrazyflieInterface


DEFAULT_URI = "radio://0/100/2M"
DEFAULT_MASS_KG = 0.0387


def make_rates(axis, rate_deg_s):
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    if axis == "roll":
        roll = rate_deg_s
    elif axis == "pitch":
        pitch = rate_deg_s
    elif axis == "yaw":
        yaw = rate_deg_s
    else:
        raise ValueError("unknown axis: %s" % axis)

    return roll, pitch, yaw


def main():
    parser = argparse.ArgumentParser(
        description="One-axis Crazyflie body-rate sign test."
    )

    parser.add_argument(
        "--axis",
        choices=("roll", "pitch", "yaw"),
        required=True,
        help="body-rate axis to test",
    )

    parser.add_argument(
        "--sign",
        type=int,
        choices=(-1, 1),
        default=1,
        help="command direction, default +1",
    )

    parser.add_argument(
        "--rate-deg-s",
        type=float,
        default=10.0,
        help="rate magnitude [deg/s], default 10",
    )

    parser.add_argument(
        "--thrust",
        type=float,
        default=3.0,
        help="collective thrust [m/s^2], default 3.0",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.4,
        help="test duration [s], default 0.4",
    )

    parser.add_argument(
        "--command-rate",
        type=float,
        default=50.0,
        help="send rate [Hz], default 50",
    )

    parser.add_argument(
        "--uri",
        default=DEFAULT_URI,
        help="Crazyflie radio URI",
    )

    parser.add_argument(
        "--live",
        action="store_true",
        help="actually send commands to the Crazyflie",
    )

    args = parser.parse_args()

    if args.rate_deg_s <= 0.0:
        parser.error("--rate-deg-s must be > 0")
    if args.thrust < 0.0:
        parser.error("--thrust must be >= 0")
    if args.duration <= 0.0:
        parser.error("--duration must be > 0")
    if args.command_rate <= 0.0:
        parser.error("--command-rate must be > 0")

    signed_rate = args.sign * args.rate_deg_s
    roll, pitch, yaw = make_rates(args.axis, signed_rate)

    cf = CrazyflieInterface(
        uri=args.uri,
        mass_kg=DEFAULT_MASS_KG,
        dry_run=not args.live,
    )

    mode = "LIVE" if args.live else "DRY RUN"

    print("Crazyflie axis test")
    print("-------------------")
    print("Mode       :", mode)
    print("URI        :", args.uri)
    print("Axis       :", args.axis)
    print("Command    : %.3f deg/s" % signed_rate)
    print(
        "Body rates : [%.3f, %.3f, %.3f] deg/s"
        % (roll, pitch, yaw)
    )
    print("Thrust     : %.3f m/s^2" % args.thrust)
    print("Duration   : %.3f s" % args.duration)
    print("Send rate  : %.1f Hz" % args.command_rate)
    print()

    period = 1.0 / args.command_rate

    try:
        cf.connect()

        # Zero-thrust packet first.
        cf.send_ctbr(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.05)

        if not args.live:
            preview = cf.send_ctbr(
                roll,
                pitch,
                yaw,
                args.thrust,
            )

            print("Dry-run packet:")
            print(
                "  rates used      = [%.3f, %.3f, %.3f] deg/s"
                % (
                    preview["roll_rate_deg_s"],
                    preview["pitch_rate_deg_s"],
                    preview["yaw_rate_deg_s"],
                )
            )
            print(
                "  thrust uint16   = %d"
                % preview["thrust_uint16"]
            )
            print(
                "  cflib args      = roll=%.3f pitch=%.3f "
                "yawrate=%.3f thrust=%d"
                % (
                    preview["commander_roll"],
                    preview["commander_pitch"],
                    preview["commander_yaw"],
                    preview["thrust_uint16"],
                )
            )
            print()
            print("Dry-run complete. No non-zero live command was sent.")
            return 0

        print("Starting axis test...")

        start = time.monotonic()
        next_send = start
        count = 0
        last_info = None

        while True:
            now = time.monotonic()

            if now - start >= args.duration:
                break

            if now >= next_send:
                last_info = cf.send_ctbr(
                    roll,
                    pitch,
                    yaw,
                    args.thrust,
                )
                count += 1
                next_send += period

            time.sleep(0.001)

        print("Sent %d packets." % count)

        if last_info is not None:
            print(
                "Last command: rates=[%.3f, %.3f, %.3f] deg/s, "
                "thrust=%d"
                % (
                    last_info["roll_rate_deg_s"],
                    last_info["pitch_rate_deg_s"],
                    last_info["yaw_rate_deg_s"],
                    last_info["thrust_uint16"],
                )
            )

        print("Axis test complete.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        print("STOP")
        try:
            cf.stop()
        finally:
            cf.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
