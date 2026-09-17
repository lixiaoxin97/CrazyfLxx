#!/usr/bin/env python3
"""
crazyflie_motor_test.py

Very small low-thrust motor test for CrazyfLxx.

Default:
    roll/pitch/yaw rate = 0 deg/s
    collective thrust   = 3.0 m/s^2
    duration            = 0.5 s
    command rate        = 50 Hz

Dry-run by default.
Use --live to actually send commands to the Crazyflie.
"""

import argparse
import time

from crazyflie_interface import CrazyflieInterface


DEFAULT_URI = "radio://0/100/2M"
DEFAULT_MASS_KG = 0.0387


def main():
    parser = argparse.ArgumentParser(
        description="Low-thrust Crazyflie motor test."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="actually send motor commands",
    )
    parser.add_argument(
        "--uri",
        default=DEFAULT_URI,
        help="Crazyflie radio URI",
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
        default=0.5,
        help="test duration [s], default 0.5",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="command rate [Hz], default 50",
    )

    args = parser.parse_args()

    if args.thrust < 0.0:
        parser.error("--thrust must be >= 0")
    if args.duration <= 0.0:
        parser.error("--duration must be > 0")
    if args.rate <= 0.0:
        parser.error("--rate must be > 0")

    cf = CrazyflieInterface(
        uri=args.uri,
        mass_kg=DEFAULT_MASS_KG,
        dry_run=not args.live,
    )

    mode = "LIVE" if args.live else "DRY RUN"
    print("Mode      :", mode)
    print("URI       :", args.uri)
    print("Thrust    : %.3f m/s^2" % args.thrust)
    print("Duration  : %.3f s" % args.duration)
    print("Rate      : %.1f Hz" % args.rate)
    print("Body rates: [0, 0, 0] deg/s")
    print()

    period = 1.0 / args.rate

    try:
        cf.connect()

        # Show the actual mapped thrust command before sending the sequence.
        preview = cf.send_ctbr(
            0.0,
            0.0,
            0.0,
            0.0,
        )

        # Stop after the zero-thrust preview/unlock command.
        cf.stop()

        if not args.live:
            print()
            print("Dry-run complete.")
            print("No non-zero thrust was sent.")
            print(
                "Run with --live to send %.3f m/s^2 for %.3f s."
                % (args.thrust, args.duration)
            )
            return 0

        print("Starting low-thrust motor test...")

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
                    0.0,
                    0.0,
                    0.0,
                    args.thrust,
                )
                count += 1
                next_send += period

            time.sleep(0.001)

        print("Sent %d low-thrust packets." % count)

        if last_info is not None:
            print(
                "Last thrust command: %d / 65535"
                % last_info["thrust_uint16"]
            )
            print(
                "Realizable thrust  : %.6f m/s^2"
                % last_info["collective_thrust_m_s2"]
            )

        print("Motor test complete.")

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
