#!/usr/bin/env python3
"""
CSV data logger for CrazyfLxx experiments.

The controller uses :class:`ExperimentDataLogger` when ``--log-file`` is
provided.  Each control tick records the Vicon state, the 12D FlightLxx
observation, the selected policy, the latest raw action, the converted CTBR
command, and safety-related flags.

The file also works as a small standalone Vicon/Mock recorder:

    python3 experiment_data_logger.py --source mock --duration 10
    python3 experiment_data_logger.py --source vrpn --output flight.csv

The standalone mode records state and observation columns; controller-only
columns are left empty.
"""

from __future__ import print_function

import argparse
import csv
import math
import os
import time
from datetime import datetime

import numpy as np

from vicon_to_nn_state import (
    MockUdpSource,
    VrpnSource,
    build_nn_observation,
    make_previous_pose,
    make_state,
    quat_to_euler_zyx_rad,
)


def _finite_or_blank(value):
    if value is None:
        return ""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    return value if math.isfinite(value) else ""


class ExperimentDataLogger(object):
    """Write one flat CSV row per control tick."""

    FIELDNAMES = [
        "timestamp_utc",
        "monotonic_s",
        "mode",
        "model",
        "pose_age_ms",
        "velocity_age_ms",
        "pose_outlier",
        "x_m",
        "y_m",
        "z_m",
        "qx",
        "qy",
        "qz",
        "qw",
        "yaw_rad",
        "pitch_rad",
        "roll_rad",
        "vx_m_s",
        "vy_m_s",
        "vz_m_s",
        "wx_rad_s",
        "wy_rad_s",
        "wz_rad_s",
    ]

    FIELDNAMES += ["obs_%d" % i for i in range(12)]
    FIELDNAMES += ["raw_action_%d" % i for i in range(4)]
    FIELDNAMES += [
        "roll_rate_deg_s",
        "pitch_rate_deg_s",
        "yaw_rate_deg_s",
        "requested_thrust_m_s2",
        "used_thrust_m_s2",
        "thrust_uint16",
        "rate_saturated",
        "thrust_saturated",
        "commander_roll",
        "commander_pitch",
        "commander_yaw",
    ]

    def __init__(self, path):
        path = os.path.abspath(os.path.expanduser(path))
        parent = os.path.dirname(path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)

        self.path = path
        self._file = open(path, "w", newline="")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=self.FIELDNAMES,
            extrasaction="ignore",
        )
        self._writer.writeheader()
        self._file.flush()

    @staticmethod
    def _ages_ms(state, now):
        pose_age = (
            1000.0 * (now - state["pose_rx_monotonic"])
            if state.get("pose_rx_monotonic") is not None
            else float("inf")
        )
        velocity_age = (
            1000.0 * (now - state["velocity_rx_monotonic"])
            if state.get("velocity_rx_monotonic") is not None
            else float("inf")
        )
        return pose_age, velocity_age

    @classmethod
    def _state_row(cls, state, now, physical_hover, nn_goal):
        row = {}
        pose = state.get("position")
        quaternion = state.get("quaternion")
        velocity = state.get("linear_velocity")
        omega = state.get("omega_body_est")

        if pose is not None:
            row.update({
                "x_m": _finite_or_blank(pose[0]),
                "y_m": _finite_or_blank(pose[1]),
                "z_m": _finite_or_blank(pose[2]),
            })

        if quaternion is not None:
            row.update({
                "qx": _finite_or_blank(quaternion[0]),
                "qy": _finite_or_blank(quaternion[1]),
                "qz": _finite_or_blank(quaternion[2]),
                "qw": _finite_or_blank(quaternion[3]),
            })
            ypr = quat_to_euler_zyx_rad(quaternion)
            if ypr is not None:
                row.update({
                    "yaw_rad": _finite_or_blank(ypr[0]),
                    "pitch_rad": _finite_or_blank(ypr[1]),
                    "roll_rad": _finite_or_blank(ypr[2]),
                })

        if velocity is not None:
            row.update({
                "vx_m_s": _finite_or_blank(velocity[0]),
                "vy_m_s": _finite_or_blank(velocity[1]),
                "vz_m_s": _finite_or_blank(velocity[2]),
            })

        if omega is not None:
            row.update({
                "wx_rad_s": _finite_or_blank(omega[0]),
                "wy_rad_s": _finite_or_blank(omega[1]),
                "wz_rad_s": _finite_or_blank(omega[2]),
            })

        pose_age, velocity_age = cls._ages_ms(state, now)
        row["pose_age_ms"] = _finite_or_blank(pose_age)
        row["velocity_age_ms"] = _finite_or_blank(velocity_age)
        row["pose_outlier"] = bool(state.get("pose_outlier", False))

        obs = build_nn_observation(
            position_vicon=pose,
            quaternion_vicon=quaternion,
            linear_velocity_world=velocity,
            angular_velocity_body=omega,
            physical_hover_position=physical_hover,
            nn_goal_position=nn_goal,
        )
        if obs is not None:
            for index, value in enumerate(np.asarray(obs).reshape(-1)):
                row["obs_%d" % index] = _finite_or_blank(value)

        return row

    def write_controller_sample(self, controller, now):
        """Record one sample from a running CrazyfLxxController."""
        row = self._state_row(
            controller.state,
            now,
            controller.hover,
            controller.nn_goal,
        )
        row.update({
            "timestamp_utc": datetime.utcnow().isoformat(timespec="microseconds")
            + "Z",
            "monotonic_s": _finite_or_blank(now),
            "mode": controller.mode,
            "model": getattr(controller, "model_name", controller.args.model),
        })

        if controller.last_nn_raw_action is not None:
            for index, value in enumerate(
                np.asarray(controller.last_nn_raw_action).reshape(-1)
            ):
                if index < 4:
                    row["raw_action_%d" % index] = _finite_or_blank(value)

        info = controller.last_nn_info
        if info is not None:
            mapping = {
                "roll_rate_deg_s": "roll_rate_deg_s",
                "pitch_rate_deg_s": "pitch_rate_deg_s",
                "yaw_rate_deg_s": "yaw_rate_deg_s",
                "requested_collective_thrust_m_s2": "requested_thrust_m_s2",
                "collective_thrust_m_s2": "used_thrust_m_s2",
                "thrust_uint16": "thrust_uint16",
                "rate_saturated": "rate_saturated",
                "thrust_saturated": "thrust_saturated",
                "commander_roll": "commander_roll",
                "commander_pitch": "commander_pitch",
                "commander_yaw": "commander_yaw",
            }
            for source_key, target_key in mapping.items():
                value = info.get(source_key)
                row[target_key] = (
                    _finite_or_blank(value)
                    if isinstance(value, (int, float, np.number))
                    else value
                )

        self._writer.writerow(row)
        self._file.flush()

    def write_state_sample(self, state, now, physical_hover, nn_goal):
        """Record a standalone state/observation sample."""
        row = self._state_row(state, now, physical_hover, nn_goal)
        row.update({
            "timestamp_utc": datetime.utcnow().isoformat(timespec="microseconds")
            + "Z",
            "monotonic_s": _finite_or_blank(now),
            "mode": "STATE_ONLY",
            "model": "",
        })
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def main():
    parser = argparse.ArgumentParser(
        description="Record Vicon/Mock state and FlightLxx observations to CSV."
    )
    parser.add_argument("--output", default="crazyflie_experiment.csv")
    parser.add_argument("--source", choices=("vrpn", "mock"), default="vrpn")
    parser.add_argument("--tracker", default="crazyflie")
    parser.add_argument("--server", default="192.168.10.1")
    parser.add_argument("--sensor", type=int, default=0)
    parser.add_argument("--mock-host", default="127.0.0.1")
    parser.add_argument("--mock-port", type=int, default=5005)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--physical-hover",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 1.0),
    )
    parser.add_argument(
        "--nn-goal",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 5.0),
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    if args.duration < 0.0:
        parser.error("--duration must be >= 0; zero means until Ctrl+C")

    state = make_state()
    previous_pose = make_previous_pose()
    if args.source == "vrpn":
        source = VrpnSource(
            state,
            previous_pose,
            tracker_name=args.tracker,
            server=args.server,
            sensor=args.sensor,
        )
    else:
        source = MockUdpSource(
            state,
            previous_pose,
            host=args.mock_host,
            port=args.mock_port,
            sensor=args.sensor,
        )

    logger = ExperimentDataLogger(args.output)
    period = 1.0 / args.rate
    next_sample = time.monotonic()
    end_time = (
        next_sample + args.duration
        if args.duration > 0.0
        else None
    )

    print("CSV output:", logger.path)
    print("Source    :", source.description())
    print("Ctrl+C to stop.")

    try:
        while end_time is None or time.monotonic() < end_time:
            source.poll()
            now = time.monotonic()
            if now >= next_sample:
                logger.write_state_sample(
                    state,
                    now,
                    np.asarray(args.physical_hover, dtype=np.float64),
                    np.asarray(args.nn_goal, dtype=np.float64),
                )
                next_sample += period
                if now - next_sample > 0.5:
                    next_sample = now + period
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        logger.close()
        close_fn = getattr(source, "close", None)
        if callable(close_fn):
            close_fn()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
