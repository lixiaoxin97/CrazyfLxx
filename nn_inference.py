#!/usr/bin/env python3
"""
nn_inference.py

CrazyfLxx Step 3:
    Vicon/mock state -> 12D observation -> FlightLxx PPO2 -> raw 4D action

Compatible with the CURRENT vicon_to_nn_state.py interface:
    make_state()
    make_previous_pose()
    VrpnSource(state, previous_pose, ...)
    MockUdpSource(state, previous_pose, ...)

Expected environment:
    Python 3.8
    TensorFlow 1.15

Inference pattern matches run_experiment.py:
    model = PPO2.load(model_path)
    action, _ = model.predict(obs, deterministic=True)
"""

import argparse
import os
import sys
import time

import numpy as np

from vicon_to_nn_state import (
    MockUdpSource,
    VrpnSource,
    build_nn_observation,
    make_previous_pose,
    make_state,
)


def add_flightlxx_to_python_path(explicit_path=None):
    """
    Make the FlightLxx repository importable.

    Priority:
      1) --flightlxx-path
      2) $FlightLxx_PATH
      3) directory containing this script
    """
    if explicit_path:
        root = os.path.abspath(os.path.expanduser(explicit_path))
    elif os.environ.get("FlightLxx_PATH"):
        root = os.path.abspath(
            os.path.expanduser(os.environ["FlightLxx_PATH"])
        )
    else:
        root = os.path.dirname(os.path.abspath(__file__))

    if root not in sys.path:
        sys.path.insert(0, root)

    return root


def load_policy(model_name, model_path=None):
    """
    Load PPO2 exactly like run_experiment.py.
    """
    from rl.lxx_baselines.ppo.ppo2 import PPO2

    if model_path is None:
        from Simulation_Experiments.configs import get_model_weight
        model_path = get_model_weight(model_name)
    else:
        model_path = os.path.abspath(os.path.expanduser(model_path))

    print("Loading model:")
    print("  name :", model_name)
    print("  path :", model_path)

    model = PPO2.load(model_path)
    return model, model_path


def data_age_ms(state, now):
    pose_age = (
        1000.0 * (now - state["pose_rx_monotonic"])
        if state["pose_rx_monotonic"] is not None
        else float("inf")
    )

    vel_age = (
        1000.0 * (now - state["velocity_rx_monotonic"])
        if state["velocity_rx_monotonic"] is not None
        else float("inf")
    )

    return pose_age, vel_age


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Vicon/mock state -> FlightLxx PPO2 -> raw 4D policy output."
        )
    )

    # Model
    parser.add_argument(
        "--model",
        choices=("CTBR", "CTBR+DR", "CTBR+ID"),
        default="CTBR",
        help="policy name; default CTBR",
    )
    parser.add_argument(
        "--model-path",
        default=None,
        help=(
            "explicit PPO2 model path. If omitted, use "
            "Simulation_Experiments.configs.get_model_weight()."
        ),
    )
    parser.add_argument(
        "--flightlxx-path",
        default=None,
        help=(
            "FlightLxx repository root. If omitted, use $FlightLxx_PATH "
            "or the directory containing this script."
        ),
    )

    # State source
    parser.add_argument(
        "--source",
        choices=("vrpn", "mock"),
        default="mock",
        help="state source; default mock",
    )

    # Real VRPN
    parser.add_argument("--tracker", default="snowyowl3")
    parser.add_argument("--server", default="192.168.10.1")

    # Mock UDP
    parser.add_argument("--mock-host", default="127.0.0.1")
    parser.add_argument("--mock-port", type=int, default=5005)

    # Common
    parser.add_argument("--sensor", type=int, default=0)

    # Observation conversion
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

    # Inference loop
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="policy inference/print rate [Hz]; default 50",
    )
    parser.add_argument(
        "--max-age-ms",
        type=float,
        default=100.0,
        help="skip inference if pose/velocity data is older than this",
    )

    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    if args.max_age_ms <= 0.0:
        parser.error("--max-age-ms must be > 0")

    flightlxx_root = add_flightlxx_to_python_path(
        args.flightlxx_path
    )

    try:
        model, model_path = load_policy(
            model_name=args.model,
            model_path=args.model_path,
        )
    except Exception as exc:
        print("ERROR: failed to load FlightLxx PPO2 model:")
        print(" ", exc)
        return 2

    # Current vicon_to_nn_state.py interface
    state = make_state()
    previous_pose = make_previous_pose()

    try:
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
    except Exception as exc:
        print("ERROR: failed to create state source:")
        print(" ", exc)
        return 2

    print()
    print("CrazyfLxx NN inference")
    print("-----------------------")
    print("FlightLxx root :", flightlxx_root)
    print("source         :", source.description())
    print("model          :", args.model)
    print("model path     :", model_path)
    print("inference rate :", "%.1f Hz" % args.rate)
    print()
    print("obs order:")
    print("[x, y, z, yaw, pitch, roll, vx, vy, vz, wx, wy, wz]")
    print()
    print("Output is RAW policy action only.")
    print("Ctrl+C to stop.")
    print()

    period = 1.0 / args.rate
    next_inference = time.monotonic()

    try:
        while True:
            source.poll()

            now = time.monotonic()

            if now < next_inference:
                time.sleep(0.001)
                continue

            next_inference += period
            if now - next_inference > 0.5:
                next_inference = now + period

            pose_age_ms, vel_age_ms = data_age_ms(state, now)

            if (
                pose_age_ms > args.max_age_ms
                or vel_age_ms > args.max_age_ms
            ):
                print(
                    "WAIT: state stale/not ready | "
                    "age pose/vel = %.1f/%.1f ms"
                    % (pose_age_ms, vel_age_ms)
                )
                continue

            obs_1d = build_nn_observation(
                position_vicon=state["position"],
                quaternion_vicon=state["quaternion"],
                linear_velocity_world=state["linear_velocity"],
                angular_velocity_body=state["omega_body_est"],
                physical_hover_position=args.physical_hover,
                nn_goal_position=args.nn_goal,
            )

            if obs_1d is None:
                print("WAIT: incomplete state")
                continue

            # Match run_experiment.py: obs shape must be (1, 12).
            obs = np.asarray(
                obs_1d,
                dtype=np.float32,
            ).reshape(1, 12)

            action, _ = model.predict(
                obs,
                deterministic=True,
            )

            action = np.asarray(
                action,
                dtype=np.float32,
            )

            if action.ndim == 2 and action.shape[0] == 1:
                action_4d = action[0]
            else:
                action_4d = action.reshape(-1)

            if action_4d.size != 4:
                raise RuntimeError(
                    "Expected 4 policy outputs, got shape %r"
                    % (action.shape,)
                )

            print(
                "obs = %s"
                % np.array2string(
                    obs[0],
                    precision=4,
                    separator=", ",
                    max_line_width=200,
                )
            )

            print(
                "act = %s"
                % np.array2string(
                    action_4d,
                    precision=6,
                    separator=", ",
                    max_line_width=120,
                )
            )

            print(
                "shape obs/action = %s / %s | "
                "age pose/vel = %.1f/%.1f ms"
                % (
                    obs.shape,
                    action.shape,
                    pose_age_ms,
                    vel_age_ms,
                )
            )
            print()

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    finally:
        # Current VrpnSource has no close(); MockUdpSource does.
        close_fn = getattr(source, "close", None)
        if callable(close_fn):
            close_fn()


if __name__ == "__main__":
    raise SystemExit(main())
