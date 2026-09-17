#!/usr/bin/env python3
"""
action_converter.py

Convert FlightLxx raw PPO action into the physical CTBR command used by the
FlightLxx environment.

Verified against the supplied FlightLxx source:

  libs/src/envs/rl_env.cpp
      quad_act_ = act.cwiseProduct(act_std_) + act_mean_;

      act_mean = [0, 0, 0, 9.81]
      act_std  = [2*pi, 2*pi, pi, 9.81]

      cmd_.omega             = quad_act_[0:3]
      cmd_.collective_thrust = quad_act_[3]

  libs/include/common/command.hpp
      omega              : body rates [rad/s]
      collective_thrust  : mass-normalized collective thrust [m/s^2]

  libs/src/objects/quadrotor.cpp
      total force [N] = mass [kg] * collective_thrust [m/s^2]

Therefore, for raw action a = [a0, a1, a2, a3]:

    wx = 2*pi * a0                         [rad/s]
    wy = 2*pi * a1                         [rad/s]
    wz = pi   * a2                         [rad/s]
    collective_thrust = 9.81 * (a3 + 1)   [m/s^2]

If the vehicle mass is known:

    total_thrust_N = mass_kg * collective_thrust

Important:
    - The PPO policy uses tanh actions, so normal raw actions are in [-1, 1].
    - This converter does NOT silently clip the raw action.
    - For the supplied simulation config, Crazyflie mass is 0.027 kg.
      Do not assume the real vehicle has exactly this mass; weigh the actual
      all-up Crazyflie before using the Newton conversion for hardware.

Examples
--------
Use the action observed at hover:

    python3 action_converter.py \
        --action -0.108741 0.015503 0.103868 0.001347

Also calculate total thrust for the FlightLxx simulated Crazyflie mass:

    python3 action_converter.py \
        --action -0.108741 0.015503 0.103868 0.001347 \
        --mass-kg 0.027
"""

import argparse
import math
import sys

import numpy as np


# Use the same numeric constants as the supplied FlightLxx C++ source.
FLIGHTLXX_PI = 3.1415926
GRAVITY = 9.81

ACT_MEAN = np.array(
    [0.0, 0.0, 0.0, GRAVITY],
    dtype=np.float64,
)

ACT_STD = np.array(
    [
        2.0 * FLIGHTLXX_PI,
        2.0 * FLIGHTLXX_PI,
        FLIGHTLXX_PI,
        GRAVITY,
    ],
    dtype=np.float64,
)

SIM_CRAZYFLIE_MASS_KG = 0.027


def raw_action_to_ctbr(raw_action):
    """
    Convert one FlightLxx raw action to physical CTBR units.

    Parameters
    ----------
    raw_action : array-like, shape (4,)
        PPO/tanh action:
            [a0, a1, a2, a3]

    Returns
    -------
    np.ndarray, shape (4,), dtype float64
        [wx, wy, wz, collective_thrust]

        wx, wy, wz:
            body-rate commands [rad/s]

        collective_thrust:
            mass-normalized collective thrust [m/s^2]
    """
    action = np.asarray(raw_action, dtype=np.float64).reshape(-1)

    if action.size != 4:
        raise ValueError(
            "Expected exactly 4 raw action values, got shape %r"
            % (np.asarray(raw_action).shape,)
        )

    if not np.all(np.isfinite(action)):
        raise ValueError("Raw action contains NaN or Inf")

    # Exact FlightLxx environment conversion:
    # quad_act = act * act_std + act_mean
    return action * ACT_STD + ACT_MEAN


def total_thrust_newtons(collective_thrust, mass_kg):
    """
    Convert mass-normalized collective thrust [m/s^2] to total force [N].
    """
    mass_kg = float(mass_kg)

    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("mass_kg must be finite and > 0")

    return mass_kg * float(collective_thrust)


def convert_action(raw_action, mass_kg=None):
    """
    Convenience wrapper returning named physical quantities.
    """
    raw = np.asarray(raw_action, dtype=np.float64).reshape(4)
    ctbr = raw_action_to_ctbr(raw)

    result = {
        "raw_action": raw,
        "omega_body_rad_s": ctbr[:3].copy(),
        "omega_body_deg_s": np.degrees(ctbr[:3]),
        "collective_thrust_m_s2": float(ctbr[3]),
    }

    if mass_kg is not None:
        result["mass_kg"] = float(mass_kg)
        result["total_thrust_N"] = total_thrust_newtons(
            ctbr[3],
            mass_kg,
        )

    return result


def format_vector(values, precision=6):
    return np.array2string(
        np.asarray(values),
        precision=precision,
        separator=", ",
        max_line_width=120,
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Convert FlightLxx raw 4D action to physical CTBR units."
        )
    )

    parser.add_argument(
        "--action",
        nargs=4,
        type=float,
        metavar=("A0", "A1", "A2", "A3"),
        required=True,
        help="raw PPO action [a0 a1 a2 a3]",
    )

    parser.add_argument(
        "--mass-kg",
        type=float,
        default=None,
        help=(
            "optional vehicle mass for converting mass-normalized thrust "
            "to total thrust in Newtons"
        ),
    )

    args = parser.parse_args()

    raw = np.asarray(args.action, dtype=np.float64)

    if np.any(raw < -1.0) or np.any(raw > 1.0):
        print(
            "WARNING: one or more raw action values are outside [-1, 1].",
            file=sys.stderr,
        )
        print(
            "FlightLxx normally supplies tanh-bounded actions in this range.",
            file=sys.stderr,
        )

    result = convert_action(
        raw_action=raw,
        mass_kg=args.mass_kg,
    )

    omega = result["omega_body_rad_s"]
    omega_deg = result["omega_body_deg_s"]
    thrust_acc = result["collective_thrust_m_s2"]

    print("FlightLxx action conversion")
    print("---------------------------")
    print("raw action              =", format_vector(raw))
    print()
    print("omega_body [rad/s]      =", format_vector(omega))
    print("omega_body [deg/s]      =", format_vector(omega_deg, 3))
    print(
        "collective thrust [m/s^2] = %.6f"
        % thrust_acc
    )

    if args.mass_kg is not None:
        print("mass [kg]               = %.6f" % args.mass_kg)
        print(
            "total thrust [N]         = %.6f"
            % result["total_thrust_N"]
        )

    print()
    print("Formula:")
    print("  wx = 2*pi*a0")
    print("  wy = 2*pi*a1")
    print("  wz = pi*a2")
    print("  collective_thrust = 9.81*(a3 + 1)")


if __name__ == "__main__":
    raise SystemExit(main())
