#!/usr/bin/env python3
"""
crazyflie_interface.py

CrazyfLxx hardware interface:
    physical CTBR command -> Crazyflie low-level rate controller

INPUT TO THIS MODULE
--------------------
The upstream action_converter is assumed to have ALREADY produced:

    roll_rate_deg_s          [deg/s]
    pitch_rate_deg_s         [deg/s]
    yaw_rate_deg_s           [deg/s]
    collective_thrust_m_s2   [m/s^2]

No rad/s -> deg/s conversion is performed here.

CONTROL PATH
------------
FlightLxx / action_converter
        |
        |  roll/pitch/yaw rate [deg/s]
        |  collective thrust   [m/s^2]
        v
crazyflie_interface.py
        |
        |  collective thrust -> total force -> uint16 thrust
        |  configure roll/pitch/yaw to RATE mode
        v
cflib Commander.send_setpoint(...)
        v
Crazyflie firmware rate PID + motor mixer

IMPORTANT FIRMWARE ASSUMPTION
-----------------------------
The thrust conversion in this file assumes a Crazyflie 2.x brushed platform
running firmware built with:

    CONFIG_ENABLE_THRUST_BAT_COMPENSATED=y
    CONFIG_CRAZYFLIE_LEGACY_PROPELLERS=y

For the legacy-propeller profile, current Bitcraze firmware defines:

    THRUST_MAX = 0.12 N per motor
    THRUST_MIN = 0.012817578393224994 N per motor

and internally interprets the motor thrust command approximately as:

    motor_target_thrust_N = thrust_uint16 / 65535 * THRUST_MAX

before battery-voltage compensation.

Vehicle mass for the current experiment:
    38.7 g = 0.0387 kg

At 1 g:
    total thrust ~= 0.379647 N
    per motor    ~= 0.094912 N
    uint16 thrust ~= 51834

Because the vehicle is relatively heavy for legacy props, the theoretical
maximum mass-normalized collective thrust using THRUST_MAX=0.12 N/motor is:

    4 * 0.12 / 0.0387 ~= 12.4031 m/s^2

Any higher FlightLxx collective-thrust request must saturate on this hardware.

SAFETY
------
- This file does not automatically take off.
- CLI mode is dry-run by default and does not import/connect to cflib.
- A real radio connection requires explicit --live.
- Even in --live CLI mode, this script only connects, configures RATE mode,
  sends a zero-thrust unlock packet, then stops and disconnects.
- Non-zero CTBR commands are intended to be sent later by the integrated
  control loop through CrazyflieInterface.send_ctbr().
"""

from __future__ import print_function

import argparse
import math
import os
import time


UINT16_MAX = 65535

DEFAULT_MASS_KG = 0.0387

# Current Bitcraze legacy-propeller profile.
LEGACY_THRUST_MAX_PER_MOTOR_N = 0.12
LEGACY_THRUST_MIN_PER_MOTOR_N = 0.012817578393224994

# FlightLxx policy physical body-rate ranges.
DEFAULT_MAX_ROLL_RATE_DEG_S = 360.0
DEFAULT_MAX_PITCH_RATE_DEG_S = 360.0
DEFAULT_MAX_YAW_RATE_DEG_S = 180.0


def clamp(value, low, high):
    return max(low, min(high, value))


def collective_thrust_to_uint16(
    collective_thrust_m_s2,
    mass_kg=DEFAULT_MASS_KG,
    thrust_max_per_motor_n=LEGACY_THRUST_MAX_PER_MOTOR_N,
    thrust_min_per_motor_n=LEGACY_THRUST_MIN_PER_MOTOR_N,
):
    """
    Convert FlightLxx mass-normalized collective thrust [m/s^2]
    to the Crazyflie uint16 thrust command.

    Derivation:
        F_total = m * a_T
        F_motor = F_total / 4

        u = 65535 * F_motor / THRUST_MAX

    The output is clipped to [0, 65535].

    For battery-compensated firmware, commands below the firmware's
    THRUST_MIN threshold are effectively zero. We mirror that behavior here.

    Returns a dictionary with both the requested and realizable quantities.
    """
    a_requested = float(collective_thrust_m_s2)
    mass_kg = float(mass_kg)
    thrust_max_per_motor_n = float(thrust_max_per_motor_n)
    thrust_min_per_motor_n = float(thrust_min_per_motor_n)

    if not math.isfinite(a_requested):
        raise ValueError("collective_thrust_m_s2 must be finite")
    if mass_kg <= 0.0 or not math.isfinite(mass_kg):
        raise ValueError("mass_kg must be finite and > 0")
    if thrust_max_per_motor_n <= 0.0:
        raise ValueError("thrust_max_per_motor_n must be > 0")
    if thrust_min_per_motor_n < 0.0:
        raise ValueError("thrust_min_per_motor_n must be >= 0")

    # Negative collective thrust is not physically available on the brushed CF.
    a_nonnegative = max(0.0, a_requested)

    total_thrust_requested_n = mass_kg * a_nonnegative
    per_motor_requested_n = total_thrust_requested_n / 4.0

    a_max_m_s2 = 4.0 * thrust_max_per_motor_n / mass_kg
    a_min_nonzero_m_s2 = 4.0 * thrust_min_per_motor_n / mass_kg

    per_motor_used_n = clamp(
        per_motor_requested_n,
        0.0,
        thrust_max_per_motor_n,
    )

    command_float = (
        UINT16_MAX * per_motor_used_n / thrust_max_per_motor_n
    )

    command_uint16 = int(round(command_float))
    command_uint16 = int(clamp(command_uint16, 0, UINT16_MAX))

    below_min = (
        command_uint16 > 0
        and per_motor_used_n < thrust_min_per_motor_n
    )

    # Match firmware behavior: thrust below THRUST_MIN becomes zero.
    if below_min:
        command_uint16 = 0
        per_motor_realizable_n = 0.0
    else:
        per_motor_realizable_n = (
            command_uint16 / float(UINT16_MAX)
            * thrust_max_per_motor_n
        )

    total_thrust_realizable_n = 4.0 * per_motor_realizable_n
    collective_thrust_realizable_m_s2 = (
        total_thrust_realizable_n / mass_kg
    )

    saturated = (
        a_requested < 0.0
        or a_nonnegative > a_max_m_s2
        or below_min
    )

    return {
        "requested_collective_thrust_m_s2": a_requested,
        "used_collective_thrust_m_s2": collective_thrust_realizable_m_s2,
        "requested_total_thrust_N": total_thrust_requested_n,
        "used_total_thrust_N": total_thrust_realizable_n,
        "requested_per_motor_thrust_N": per_motor_requested_n,
        "used_per_motor_thrust_N": per_motor_realizable_n,
        "thrust_uint16": command_uint16,
        "max_collective_thrust_m_s2": a_max_m_s2,
        "min_nonzero_collective_thrust_m_s2": a_min_nonzero_m_s2,
        "saturated": saturated,
        "below_min": below_min,
    }


def prepare_rate_command(
    roll_rate_deg_s,
    pitch_rate_deg_s,
    yaw_rate_deg_s,
    roll_sign=1.0,
    pitch_sign=1.0,
    yaw_sign=1.0,
    max_roll_rate_deg_s=DEFAULT_MAX_ROLL_RATE_DEG_S,
    max_pitch_rate_deg_s=DEFAULT_MAX_PITCH_RATE_DEG_S,
    max_yaw_rate_deg_s=DEFAULT_MAX_YAW_RATE_DEG_S,
):
    """
    Prepare desired Crazyflie body-rate commands.

    The input/output of this helper use the intuitive Crazyflie body-rate signs.
    Historical cflib/firmware sign compensation is handled separately when the
    packet is sent.

    Returns:
        dict with clipped body rates in deg/s.
    """
    r = float(roll_rate_deg_s) * float(roll_sign)
    p = float(pitch_rate_deg_s) * float(pitch_sign)
    y = float(yaw_rate_deg_s) * float(yaw_sign)

    for name, value in (
        ("roll_rate_deg_s", r),
        ("pitch_rate_deg_s", p),
        ("yaw_rate_deg_s", y),
    ):
        if not math.isfinite(value):
            raise ValueError("%s must be finite" % name)

    r_used = clamp(r, -max_roll_rate_deg_s, max_roll_rate_deg_s)
    p_used = clamp(p, -max_pitch_rate_deg_s, max_pitch_rate_deg_s)
    y_used = clamp(y, -max_yaw_rate_deg_s, max_yaw_rate_deg_s)

    return {
        "roll_rate_deg_s": r_used,
        "pitch_rate_deg_s": p_used,
        "yaw_rate_deg_s": y_used,
        "rate_saturated": (
            r_used != r or p_used != p or y_used != y
        ),
    }


def body_rates_to_legacy_send_setpoint_args(
    roll_rate_deg_s,
    pitch_rate_deg_s,
    yaw_rate_deg_s,
):
    """
    Convert desired firmware body rates to cflib send_setpoint() arguments.

    Current cflib send_setpoint() packs:
        roll, -pitch, yawrate

    The legacy RPYT firmware RATE decoder then uses:
        roll_rate  = packet.roll
        pitch_rate = packet.pitch
        yaw_rate   = -packet.yaw

    Therefore to realize desired:
        [roll_rate, pitch_rate, yaw_rate]

    call cflib with:
        [roll_rate, -pitch_rate, -yaw_rate]
    """
    return (
        float(roll_rate_deg_s),
        -float(pitch_rate_deg_s),
        -float(yaw_rate_deg_s),
    )


class CrazyflieInterface(object):
    """
    Thin Crazyflie CTBR hardware interface.

    The class is safe to instantiate without cflib when dry_run=True.
    """

    def __init__(
        self,
        uri=None,
        mass_kg=DEFAULT_MASS_KG,
        dry_run=True,
        cache_dir="./cache",
        roll_sign=1.0,
        pitch_sign=-1.0,
        yaw_sign=1.0,
    ):
        self.uri = (
            uri
            or os.environ.get("CRAZYFLIE_URI")
            or "radio://0/100/2M"
        )

        self.mass_kg = float(mass_kg)
        self.dry_run = bool(dry_run)
        self.cache_dir = cache_dir

        # These signs are intentionally configurable for the final empirical
        # axis/sign validation against the physical vehicle.
        self.roll_sign = float(roll_sign)
        self.pitch_sign = float(pitch_sign)
        self.yaw_sign = float(yaw_sign)

        self._connected = False
        self._scf = None
        self._cf = None
        self._last_command = None

    @property
    def connected(self):
        return self._connected

    @property
    def max_collective_thrust_m_s2(self):
        return (
            4.0 * LEGACY_THRUST_MAX_PER_MOTOR_N / self.mass_kg
        )

    @property
    def hover_thrust_uint16(self):
        result = collective_thrust_to_uint16(
            9.81,
            mass_kg=self.mass_kg,
        )
        return result["thrust_uint16"]

    def connect(self):
        """
        Connect and configure low-level roll/pitch/yaw RATE mode.

        In live mode this waits until the parameter values have been downloaded,
        disables position/altitude assist modes, selects rate control for R/P/Y,
        verifies the parameters, and sends one zero-thrust packet to unlock the
        legacy RPYT commander.
        """
        if self._connected:
            return

        if self.dry_run:
            self._connected = True
            print("[DRY RUN] Crazyflie connection simulated:", self.uri)
            return

        # Delay imports so dry-run works on machines without cflib installed.
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()

        cf = Crazyflie(rw_cache=self.cache_dir)
        scf = SyncCrazyflie(self.uri, cf=cf)

        print("Connecting to Crazyflie:", self.uri)
        scf.open_link()
        scf.wait_for_params()

        self._scf = scf
        self._cf = scf.cf

        # Ensure the legacy RPYT packet means exactly:
        # roll rate / pitch rate / yaw rate / raw collective thrust.
        desired_params = {
            "flightmode.althold": "0",
            "flightmode.poshold": "0",
            "flightmode.posSet": "0",
            "flightmode.stabModeRoll": "0",
            "flightmode.stabModePitch": "0",
            "flightmode.stabModeYaw": "0",
        }

        for name, value in desired_params.items():
            self._cf.param.set_value(name, value)

        # Parameter writes are asynchronous. Give the round trips time to
        # update the local parameter cache, then verify every critical mode.
        time.sleep(0.3)

        for name, expected in desired_params.items():
            actual = self._cf.param.get_value(name)
            try:
                actual_num = int(float(actual))
            except (TypeError, ValueError):
                raise RuntimeError(
                    "Could not verify Crazyflie parameter %s=%r"
                    % (name, actual)
                )

            if actual_num != int(expected):
                raise RuntimeError(
                    "Crazyflie parameter verification failed: "
                    "%s=%r, expected %s"
                    % (name, actual, expected)
                )

        # The legacy commander has a motor lock. A zero-thrust setpoint unlocks it.
        self._cf.commander.send_setpoint(
            0.0,
            0.0,
            0.0,
            0,
        )
        time.sleep(0.05)

        self._connected = True

        print("Crazyflie connected.")
        print("R/P/Y mode: RATE / RATE / RATE")
        print("Mass: %.4f kg" % self.mass_kg)
        print(
            "Legacy-prop max collective thrust: %.4f m/s^2"
            % self.max_collective_thrust_m_s2
        )
        print(
            "Predicted hover thrust command: %d"
            % self.hover_thrust_uint16
        )

    def send_ctbr(
        self,
        roll_rate_deg_s,
        pitch_rate_deg_s,
        yaw_rate_deg_s,
        collective_thrust_m_s2,
    ):
        """
        Send one CTBR command.

        Inputs are already in physical units:
            roll_rate_deg_s        deg/s
            pitch_rate_deg_s       deg/s
            yaw_rate_deg_s         deg/s
            collective_thrust_m_s2 m/s^2

        This method should later be called continuously by the main control loop,
        nominally at 50 Hz.
        """
        if not self._connected:
            raise RuntimeError(
                "CrazyflieInterface is not connected. Call connect() first."
            )

        rates = prepare_rate_command(
            roll_rate_deg_s,
            pitch_rate_deg_s,
            yaw_rate_deg_s,
            roll_sign=self.roll_sign,
            pitch_sign=self.pitch_sign,
            yaw_sign=self.yaw_sign,
        )

        thrust = collective_thrust_to_uint16(
            collective_thrust_m_s2,
            mass_kg=self.mass_kg,
        )

        commander_roll, commander_pitch, commander_yaw = (
            body_rates_to_legacy_send_setpoint_args(
                rates["roll_rate_deg_s"],
                rates["pitch_rate_deg_s"],
                rates["yaw_rate_deg_s"],
            )
        )

        info = {
            "roll_rate_deg_s": rates["roll_rate_deg_s"],
            "pitch_rate_deg_s": rates["pitch_rate_deg_s"],
            "yaw_rate_deg_s": rates["yaw_rate_deg_s"],
            "rate_saturated": rates["rate_saturated"],
            "collective_thrust_m_s2": (
                thrust["used_collective_thrust_m_s2"]
            ),
            "requested_collective_thrust_m_s2": (
                thrust["requested_collective_thrust_m_s2"]
            ),
            "thrust_uint16": thrust["thrust_uint16"],
            "thrust_saturated": thrust["saturated"],
            "commander_roll": commander_roll,
            "commander_pitch": commander_pitch,
            "commander_yaw": commander_yaw,
        }

        self._last_command = info

        if self.dry_run:
            return info

        self._cf.commander.send_setpoint(
            commander_roll,
            commander_pitch,
            commander_yaw,
            thrust["thrust_uint16"],
        )

        return info

    def stop(self):
        """Immediately send the Crazyflie STOP setpoint."""
        if not self._connected:
            return

        if self.dry_run:
            print("[DRY RUN] STOP")
            return

        # Send STOP more than once for a little robustness on a radio link.
        for _ in range(3):
            self._cf.commander.send_stop_setpoint()
            time.sleep(0.02)

    def close(self):
        """Stop motors and close the radio link."""
        if not self._connected:
            return

        try:
            self.stop()
        finally:
            if not self.dry_run and self._scf is not None:
                self._scf.close_link()

            self._connected = False
            self._scf = None
            self._cf = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def print_thrust_summary(mass_kg):
    hover = collective_thrust_to_uint16(
        9.81,
        mass_kg=mass_kg,
    )

    max_accel = hover["max_collective_thrust_m_s2"]

    print("Crazyflie thrust model summary")
    print("------------------------------")
    print("mass                    : %.4f kg" % mass_kg)
    print(
        "legacy thrust max/motor : %.6f N"
        % LEGACY_THRUST_MAX_PER_MOTOR_N
    )
    print(
        "legacy thrust min/motor : %.6f N"
        % LEGACY_THRUST_MIN_PER_MOTOR_N
    )
    print("hover total thrust      : %.6f N" % (mass_kg * 9.81))
    print(
        "hover thrust command    : %d / 65535"
        % hover["thrust_uint16"]
    )
    print(
        "max collective thrust   : %.6f m/s^2"
        % max_accel
    )
    print(
        "max / g                 : %.3f g"
        % (max_accel / 9.81)
    )
    print()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "CrazyfLxx Crazyflie CTBR interface. "
            "Dry-run by default."
        )
    )

    parser.add_argument(
        "--uri",
        default=None,
        help=(
            "Crazyflie URI. Default: $CRAZYFLIE_URI or "
            "radio://0/80/2M/E7E7E7E7E7"
        ),
    )

    parser.add_argument(
        "--mass-g",
        type=float,
        default=38.7,
        help="all-up vehicle mass [g]; default 38.7",
    )

    parser.add_argument(
        "--live",
        action="store_true",
        help=(
            "actually connect to Crazyflie. "
            "No non-zero thrust is sent by this CLI."
        ),
    )

    parser.add_argument(
        "--show",
        nargs=4,
        type=float,
        metavar=("ROLL_DPS", "PITCH_DPS", "YAW_DPS", "THRUST_M_S2"),
        default=(0.0, 0.0, 0.0, 9.81),
        help=(
            "show how one CTBR command would be converted; "
            "default: 0 0 0 9.81"
        ),
    )

    args = parser.parse_args()

    mass_kg = args.mass_g / 1000.0
    print_thrust_summary(mass_kg)

    interface = CrazyflieInterface(
        uri=args.uri,
        mass_kg=mass_kg,
        dry_run=not args.live,
    )

    if args.live:
        # Safety: CLI live mode only checks connection/configuration.
        # It does NOT pass --show as a real motor command.
        try:
            interface.connect()
            print(
                "Live connection/configuration test passed. "
                "No non-zero thrust was sent."
            )
        finally:
            interface.close()
        return 0

    # Dry-run path: safe to exercise the full conversion.
    interface.connect()

    roll, pitch, yaw, thrust = args.show
    info = interface.send_ctbr(
        roll,
        pitch,
        yaw,
        thrust,
    )

    print("Dry-run CTBR conversion")
    print("-----------------------")
    print(
        "body rates requested    : "
        "[%.3f, %.3f, %.3f] deg/s"
        % (roll, pitch, yaw)
    )
    print(
        "body rates used         : "
        "[%.3f, %.3f, %.3f] deg/s"
        % (
            info["roll_rate_deg_s"],
            info["pitch_rate_deg_s"],
            info["yaw_rate_deg_s"],
        )
    )
    print(
        "collective thrust req.  : %.6f m/s^2"
        % info["requested_collective_thrust_m_s2"]
    )
    print(
        "collective thrust used  : %.6f m/s^2"
        % info["collective_thrust_m_s2"]
    )
    print(
        "thrust uint16           : %d"
        % info["thrust_uint16"]
    )
    print(
        "cflib send_setpoint args: "
        "roll=%.3f pitch=%.3f yawrate=%.3f thrust=%d"
        % (
            info["commander_roll"],
            info["commander_pitch"],
            info["commander_yaw"],
            info["thrust_uint16"],
        )
    )
    print(
        "rate/thrust saturated   : %s / %s"
        % (
            info["rate_saturated"],
            info["thrust_saturated"],
        )
    )

    interface.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
