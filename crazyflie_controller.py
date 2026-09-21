#!/usr/bin/env python3
"""
crazyflie_controller.py

CrazyfLxx closed-loop experiment controller.

Control flow
------------
1. Crazyflie is placed on the ground.
2. "arm"
      -> supervisor arming request
      -> low collective thrust so the motors spin on the ground
3. "takeoff"
      -> onboard Crazyflie PID position controller
      -> smooth position trajectory to (0, 0, 1), yaw = 0
4. "nn"
      -> FlightLxx PPO policy
      -> raw action -> action_converter -> CTBR
      -> Crazyflie body-rate PID + collective thrust
5. "position"
      -> disable NN control
      -> switch directly back to onboard position control
      -> smooth return to (0, 0, 1), yaw = 0
6. "land"
      -> onboard position control
      -> smooth descent to the recorded ground height
      -> zero thrust -> STOP -> disarm

The Vicon pose is continuously forwarded to the Crazyflie EKF.

Expected repository modules
---------------------------
    vicon_to_nn_state.py
    nn_inference.py
    action_converter.py
    crazyflie_interface.py

The current crazyflie_interface.py is reused for the verified CTBR mapping.
This controller uses its connected cflib Crazyflie object for:
    - supervisor arm/disarm
    - Vicon extpose/extpos forwarding
    - generic position setpoints

Python:
    3.8 compatible

IMPORTANT
---------
This is experimental flight-control software.

Default mode is DRY RUN. Add --live to enable the radio/motors.

Before the first real takeoff:
    - verify Vicon axes and Crazyflie body orientation
    - keep a clear flight volume
    - keep a physical emergency plan
    - start with the normal position-controller takeoff/landing path
      before enabling the NN in flight
"""

from __future__ import print_function

import argparse
import math
import os
import select
import sys
import time

import numpy as np

from action_converter import convert_action
from crazyflie_interface import CrazyflieInterface
from vicon_to_nn_state import (
    MockUdpSource,
    VrpnSource,
    build_nn_observation,
    make_previous_pose,
    make_state,
    quat_normalize,
    quat_to_euler_zyx_rad,
)


STATE_DISARMED = "DISARMED"
STATE_ARMED_IDLE = "ARMED_IDLE"
STATE_TAKEOFF = "TAKEOFF"
STATE_POSITION_HOLD = "POSITION_HOLD"
STATE_RETURN_POSITION = "RETURN_POSITION"
STATE_NN = "NN_CONTROL"
STATE_LANDING = "LANDING"
STATE_STOPPED = "STOPPED"


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
    Load FlightLxx PPO2 in the same way as run_experiment.py.
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


def clamp(value, low, high):
    return max(low, min(high, value))


def smoothstep01(x):
    x = clamp(float(x), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def wrap_deg(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def interpolate_yaw_deg(yaw0, yaw1, alpha):
    delta = wrap_deg(float(yaw1) - float(yaw0))
    return wrap_deg(float(yaw0) + float(alpha) * delta)


def position_tuple(state):
    if state["position"] is None:
        return None
    return tuple(float(v) for v in state["position"])


def velocity_tuple(state):
    if state["linear_velocity"] is None:
        return None
    return tuple(float(v) for v in state["linear_velocity"])


def attitude_ypr_rad(state):
    if state["quaternion"] is None:
        return None
    return quat_to_euler_zyx_rad(state["quaternion"])


def vector_norm(values):
    return math.sqrt(sum(float(v) * float(v) for v in values))


def action_to_4d(action):
    action = np.asarray(action, dtype=np.float32)
    if action.ndim == 2 and action.shape[0] == 1:
        action = action[0]
    else:
        action = action.reshape(-1)

    if action.size != 4:
        raise RuntimeError(
            "Expected 4 policy outputs, got shape %r"
            % (np.asarray(action).shape,)
        )

    if not np.all(np.isfinite(action)):
        raise RuntimeError("NN action contains NaN or Inf")

    return action


class CrazyflieFlightLink(object):
    """
    Position/Vicon/supervisor adapter around the already-tested
    CrazyflieInterface.

    CTBR still goes through CrazyflieInterface.send_ctbr().
    """

    def __init__(self, hardware):
        self.hardware = hardware

    @property
    def dry_run(self):
        return self.hardware.dry_run

    @property
    def cf(self):
        if self.dry_run:
            return None
        if not self.hardware.connected or self.hardware._cf is None:
            raise RuntimeError("Crazyflie is not connected")
        return self.hardware._cf

    def set_param_verified(self, name, value, settle_s=0.08):
        if self.dry_run:
            print("[DRY RUN] param %s = %s" % (name, value))
            return

        self.cf.param.set_value(name, str(value))
        time.sleep(float(settle_s))

        actual = self.cf.param.get_value(name)
        try:
            expected_num = float(value)
            actual_num = float(actual)
            if abs(actual_num - expected_num) > 1e-6:
                raise RuntimeError(
                    "Parameter verification failed: %s=%r, expected %r"
                    % (name, actual, value)
                )
        except (TypeError, ValueError):
            if str(actual) != str(value):
                raise RuntimeError(
                    "Parameter verification failed: %s=%r, expected %r"
                    % (name, actual, value)
                )

    def configure_position_stack(self):
        """
        Force:
            EKF estimator = 2
            PID controller = 1

        Generic position setpoints then activate the onboard cascaded
        position/velocity/attitude/rate loops.
        """
        self.set_param_verified("stabilizer.estimator", 2)
        self.set_param_verified("stabilizer.controller", 1)

    def set_kalman_initial_state(self, position, yaw_rad):
        x, y, z = map(float, position)
        self.set_param_verified("kalman.initialX", x)
        self.set_param_verified("kalman.initialY", y)
        self.set_param_verified("kalman.initialZ", z)
        self.set_param_verified("kalman.initialYaw", float(yaw_rad))

    def reset_kalman(self):
        if self.dry_run:
            print("[DRY RUN] Kalman reset")
            return

        self.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        self.cf.param.set_value("kalman.resetEstimation", "0")

    def send_external_measurement(self, state, fusion_mode):
        p = state["position"]
        q = state["quaternion"]

        if p is None:
            return False

        if self.dry_run:
            return True

        if fusion_mode == "position":
            self.cf.extpos.send_extpos(
                float(p[0]),
                float(p[1]),
                float(p[2]),
            )
            return True

        if q is None:
            return False

        qn = quat_normalize(q)
        if qn is None:
            return False

        self.cf.extpos.send_extpose(
            float(p[0]),
            float(p[1]),
            float(p[2]),
            float(qn[0]),
            float(qn[1]),
            float(qn[2]),
            float(qn[3]),
        )
        return True

    def send_position_setpoint(self, x, y, z, yaw_deg):
        if self.dry_run:
            return

        self.cf.commander.send_position_setpoint(
            float(x),
            float(y),
            float(z),
            float(yaw_deg),
        )

    def arm(self):
        if self.dry_run:
            print("[DRY RUN] ARM")
            return True

        cf = self.cf
        supervisor = getattr(cf, "supervisor", None)

        if supervisor is not None and hasattr(
            supervisor, "send_arming_request"
        ):
            supervisor.send_arming_request(True)
        else:
            # Backward-compatible cflib fallback.
            cf.platform.send_arming_request(True)

        time.sleep(0.20)
        return self.is_armed(default=True)

    def disarm(self):
        if self.dry_run:
            print("[DRY RUN] DISARM")
            return

        cf = self.cf
        supervisor = getattr(cf, "supervisor", None)

        if supervisor is not None and hasattr(
            supervisor, "send_arming_request"
        ):
            supervisor.send_arming_request(False)
        else:
            cf.platform.send_arming_request(False)

    def is_armed(self, default=None):
        if self.dry_run:
            return default

        supervisor = getattr(self.cf, "supervisor", None)
        if supervisor is None:
            return default

        try:
            if hasattr(supervisor, "is_armed"):
                return bool(supervisor.is_armed)
        except Exception:
            pass

        return default

    def supervisor_states(self):
        if self.dry_run:
            return []

        supervisor = getattr(self.cf, "supervisor", None)
        if supervisor is None:
            return []

        try:
            if hasattr(supervisor, "read_state_list"):
                return list(supervisor.read_state_list())
        except Exception:
            pass

        return []

    def emergency_stop(self):
        if self.dry_run:
            print("[DRY RUN] EMERGENCY STOP")
            return

        supervisor = getattr(self.cf, "supervisor", None)

        if supervisor is not None and hasattr(
            supervisor, "send_emergency_stop"
        ):
            supervisor.send_emergency_stop()
        else:
            self.cf.commander.send_stop_setpoint()


class CrazyfLxxController(object):
    def __init__(
        self,
        args,
        model,
        model_path,
        source,
        state,
        hardware,
        flight_link,
    ):
        self.args = args
        self.model = model
        self.model_path = model_path
        self.source = source
        self.state = state
        self.hardware = hardware
        self.link = flight_link

        self.mode = STATE_DISARMED
        self.running = True

        self.hover = np.asarray(args.hover, dtype=np.float64)
        self.nn_goal = np.asarray(args.nn_goal, dtype=np.float64)

        self.ground_position = None
        self.ground_yaw_deg = 0.0

        self.transition_start_time = None
        self.transition_duration = None
        self.transition_start_position = None
        self.transition_target_position = None
        self.transition_start_yaw_deg = 0.0
        self.transition_target_yaw_deg = 0.0

        self.nn_start_time = None
        self.nn_saturation_start = None
        self.last_nn_raw_action = None
        self.last_nn_info = None

        self.last_status_print = 0.0
        self.last_supervisor_check = 0.0

        self.vicon_soft_fault_active = False

    # ------------------------------------------------------------------
    # Vicon / EKF setup
    # ------------------------------------------------------------------

    def poll_vicon(self):
        self.source.poll()

    def state_ready(self):
        return (
            self.state["position"] is not None
            and self.state["quaternion"] is not None
            and self.state["linear_velocity"] is not None
            and self.state["omega_body_est"] is not None
        )

    def pose_fresh(self, now=None):
        if now is None:
            now = time.monotonic()

        if self.state["pose_rx_monotonic"] is None:
            return False

        age_ms = 1000.0 * (
            now - self.state["pose_rx_monotonic"]
        )
        return age_ms <= self.args.vicon_hard_timeout_ms

    def full_state_fresh(self, now=None):
        if now is None:
            now = time.monotonic()

        pose_age_ms, vel_age_ms = data_age_ms(
            self.state,
            now,
        )

        return (
            pose_age_ms <= self.args.vicon_soft_timeout_ms
            and vel_age_ms <= self.args.vicon_soft_timeout_ms
        )

    def send_vicon_to_cf(self, now):
        if self.state["pose_rx_monotonic"] is None:
            return

        pose_age_ms = 1000.0 * (
            now - self.state["pose_rx_monotonic"]
        )

        # Never keep re-sending an old mocap sample to the EKF.
        # Once the pose is older than the soft timeout, let the onboard
        # estimator coast on its IMU until Vicon recovers.
        if pose_age_ms > self.args.vicon_soft_timeout_ms:
            return

        self.link.send_external_measurement(
            self.state,
            self.args.vicon_fusion,
        )

    def wait_for_initial_vicon(self):
        print()
        print("Waiting for complete Vicon state...")

        while self.running:
            self.poll_vicon()
            now = time.monotonic()

            if self.state_ready() and self.full_state_fresh(now):
                p = position_tuple(self.state)
                ypr = attitude_ypr_rad(self.state)
                yaw_deg = math.degrees(ypr[0])

                self.ground_position = np.asarray(
                    p,
                    dtype=np.float64,
                )
                self.ground_yaw_deg = yaw_deg

                print(
                    "Vicon ready: p=[%.3f, %.3f, %.3f] m, "
                    "yaw=%.2f deg"
                    % (
                        p[0],
                        p[1],
                        p[2],
                        yaw_deg,
                    )
                )
                return

            time.sleep(0.002)

    def initialize_estimator(self):
        ypr = attitude_ypr_rad(self.state)
        if ypr is None:
            raise RuntimeError("Vicon attitude unavailable")

        print()
        print("Configuring Crazyflie position stack...")
        print("  estimator : EKF (2)")
        print("  controller: PID (1)")
        print("  Vicon fuse: %s" % self.args.vicon_fusion)

        self.link.configure_position_stack()

        # Prime the estimator with Vicon measurements before reset.
        prime_end = time.monotonic() + self.args.ekf_prime_s
        while time.monotonic() < prime_end:
            self.poll_vicon()
            now = time.monotonic()
            self.send_vicon_to_cf(now)
            time.sleep(0.005)

        p = position_tuple(self.state)
        ypr = attitude_ypr_rad(self.state)
        yaw_rad = float(ypr[0])

        self.link.set_kalman_initial_state(
            p,
            yaw_rad,
        )
        self.link.reset_kalman()

        print(
            "Settling EKF for %.2f s while streaming Vicon..."
            % self.args.ekf_settle_s
        )

        settle_end = time.monotonic() + self.args.ekf_settle_s
        while time.monotonic() < settle_end:
            self.poll_vicon()
            now = time.monotonic()

            if not self.pose_fresh(now):
                raise RuntimeError(
                    "Vicon became stale during EKF initialization"
                )

            self.send_vicon_to_cf(now)
            time.sleep(0.005)

        # Record ground reference after estimator setup.
        p = position_tuple(self.state)
        ypr = attitude_ypr_rad(self.state)
        self.ground_position = np.asarray(p, dtype=np.float64)
        self.ground_yaw_deg = math.degrees(ypr[0])

        print(
            "Ground reference: [%.3f, %.3f, %.3f] m"
            % tuple(self.ground_position)
        )

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def set_mode(self, mode):
        if mode != self.mode:
            print()
            print("MODE: %s -> %s" % (self.mode, mode))
            self.mode = mode

    def command_arm(self):
        if self.mode != STATE_DISARMED:
            print("ARM rejected: current mode is %s" % self.mode)
            return

        if not self.full_state_fresh():
            print("ARM rejected: Vicon state is not fresh")
            return

        p = np.asarray(position_tuple(self.state))
        xy_error = vector_norm(p[:2] - self.hover[:2])

        if xy_error > self.args.max_ground_xy_error:
            print(
                "ARM rejected: ground XY is %.3f m from hover XY; "
                "limit %.3f m"
                % (xy_error, self.args.max_ground_xy_error)
            )
            return

        self.ground_position = p.copy()

        armed = self.link.arm()

        if armed is False:
            print("ARM request was not accepted by supervisor")
            return

        # The already-tested low thrust makes brushed motors visibly spin.
        self.set_mode(STATE_ARMED_IDLE)
        print(
            "Armed. Ground idle collective thrust: %.2f m/s^2"
            % self.args.idle_thrust
        )
        print("Next command: takeoff")

    def begin_position_transition(
        self,
        mode,
        target_position,
        target_yaw_deg,
        duration,
    ):
        p = position_tuple(self.state)
        ypr = attitude_ypr_rad(self.state)

        if p is None or ypr is None:
            raise RuntimeError(
                "Cannot start position transition without Vicon pose"
            )

        self.transition_start_time = time.monotonic()
        self.transition_duration = max(0.05, float(duration))
        self.transition_start_position = np.asarray(
            p,
            dtype=np.float64,
        )
        self.transition_target_position = np.asarray(
            target_position,
            dtype=np.float64,
        )
        self.transition_start_yaw_deg = math.degrees(ypr[0])
        self.transition_target_yaw_deg = float(target_yaw_deg)

        self.set_mode(mode)

    def command_takeoff(self):
        if self.mode != STATE_ARMED_IDLE:
            print(
                "TAKEOFF rejected: arm first; current mode is %s"
                % self.mode
            )
            return

        if not self.full_state_fresh():
            print("TAKEOFF rejected: Vicon state is not fresh")
            return

        self.begin_position_transition(
            STATE_TAKEOFF,
            self.hover,
            self.args.hover_yaw_deg,
            self.args.takeoff_duration,
        )

        print(
            "Position takeoff -> [%.2f, %.2f, %.2f] m, yaw %.1f deg"
            % (
                self.hover[0],
                self.hover[1],
                self.hover[2],
                self.args.hover_yaw_deg,
            )
        )

    def nn_entry_is_safe(self):
        p = np.asarray(position_tuple(self.state), dtype=np.float64)
        v = np.asarray(velocity_tuple(self.state), dtype=np.float64)
        ypr = attitude_ypr_rad(self.state)

        if ypr is None:
            return False, "attitude unavailable"

        yaw, pitch, roll = ypr

        pos_error = vector_norm(p - self.hover)
        speed = vector_norm(v)
        yaw_error_deg = abs(
            wrap_deg(
                math.degrees(yaw)
                - self.args.hover_yaw_deg
            )
        )

        if pos_error > self.args.nn_entry_position_tolerance:
            return (
                False,
                "position error %.3f m > %.3f m"
                % (
                    pos_error,
                    self.args.nn_entry_position_tolerance,
                ),
            )

        if speed > self.args.nn_entry_speed_tolerance:
            return (
                False,
                "speed %.3f m/s > %.3f m/s"
                % (
                    speed,
                    self.args.nn_entry_speed_tolerance,
                ),
            )

        if abs(math.degrees(roll)) > self.args.nn_entry_tilt_deg:
            return False, "roll is not near level"

        if abs(math.degrees(pitch)) > self.args.nn_entry_tilt_deg:
            return False, "pitch is not near level"

        if yaw_error_deg > self.args.nn_entry_yaw_tolerance_deg:
            return False, "yaw is not near the FlightLxx target"

        return True, "OK"

    def command_nn(self):
        if self.mode != STATE_POSITION_HOLD:
            print(
                "NN rejected: first stabilize in POSITION_HOLD; "
                "current mode is %s"
                % self.mode
            )
            return

        if not self.full_state_fresh():
            print("NN rejected: Vicon state is not fresh")
            return

        safe, reason = self.nn_entry_is_safe()
        if not safe:
            print("NN rejected:", reason)
            return

        self.nn_start_time = time.monotonic()
        self.nn_saturation_start = None
        self.last_nn_raw_action = None
        self.last_nn_info = None

        self.set_mode(STATE_NN)
        print("NN model active:", self.args.model)
        print("Use 'position' to disable NN and return to hover.")

    def command_position(self, automatic_reason=None):
        if self.mode not in (
            STATE_NN,
            STATE_POSITION_HOLD,
            STATE_RETURN_POSITION,
        ):
            print(
                "POSITION rejected: current mode is %s"
                % self.mode
            )
            return

        if self.mode == STATE_POSITION_HOLD:
            print("Already in POSITION_HOLD")
            return

        self.begin_position_transition(
            STATE_RETURN_POSITION,
            self.hover,
            self.args.hover_yaw_deg,
            self.args.return_duration,
        )

        if automatic_reason:
            print(
                "NN disabled automatically: %s"
                % automatic_reason
            )
        else:
            print(
                "NN disabled. Returning to position hover "
                "[%.2f, %.2f, %.2f]."
                % tuple(self.hover)
            )

    def command_land(self):
        if self.mode != STATE_POSITION_HOLD:
            print(
                "LAND rejected: first switch to POSITION_HOLD; "
                "current mode is %s"
                % self.mode
            )
            return

        target = np.array(
            [
                self.hover[0],
                self.hover[1],
                self.ground_position[2]
                - self.args.landing_below_ground_m,
            ],
            dtype=np.float64,
        )

        self.begin_position_transition(
            STATE_LANDING,
            target,
            self.args.hover_yaw_deg,
            self.args.landing_duration,
        )

        print(
            "Landing toward ground z=%.3f m"
            % self.ground_position[2]
        )

    def normal_stop_and_disarm(self, reason):
        print()
        print("STOP:", reason)

        try:
            self.hardware.stop()
        finally:
            try:
                self.link.disarm()
            finally:
                self.set_mode(STATE_DISARMED)

    def emergency_stop(self, reason):
        print()
        print("EMERGENCY STOP:", reason)

        try:
            self.link.emergency_stop()
        finally:
            try:
                self.hardware.stop()
            finally:
                try:
                    self.link.disarm()
                finally:
                    self.set_mode(STATE_STOPPED)

    # ------------------------------------------------------------------
    # Control outputs
    # ------------------------------------------------------------------

    def send_idle(self):
        self.hardware.send_ctbr(
            0.0,
            0.0,
            0.0,
            self.args.idle_thrust,
        )

    def send_position_transition(self, now):
        elapsed = now - self.transition_start_time
        phase = elapsed / self.transition_duration
        alpha = smoothstep01(phase)

        p_ref = (
            self.transition_start_position
            + alpha
            * (
                self.transition_target_position
                - self.transition_start_position
            )
        )

        yaw_ref = interpolate_yaw_deg(
            self.transition_start_yaw_deg,
            self.transition_target_yaw_deg,
            alpha,
        )

        self.link.send_position_setpoint(
            p_ref[0],
            p_ref[1],
            p_ref[2],
            yaw_ref,
        )

        if phase >= 1.0:
            if self.mode == STATE_TAKEOFF:
                self.set_mode(STATE_POSITION_HOLD)
                print(
                    "Takeoff trajectory finished. "
                    "Holding [%.2f, %.2f, %.2f]."
                    % tuple(self.hover)
                )

            elif self.mode == STATE_RETURN_POSITION:
                self.set_mode(STATE_POSITION_HOLD)
                print(
                    "Return trajectory finished. POSITION_HOLD."
                )

    def send_position_hold(self):
        self.link.send_position_setpoint(
            self.hover[0],
            self.hover[1],
            self.hover[2],
            self.args.hover_yaw_deg,
        )

    def send_nn(self, now):
        obs_1d = build_nn_observation(
            position_vicon=self.state["position"],
            quaternion_vicon=self.state["quaternion"],
            linear_velocity_world=self.state["linear_velocity"],
            angular_velocity_body=self.state["omega_body_est"],
            physical_hover_position=self.hover,
            nn_goal_position=self.nn_goal,
        )

        if obs_1d is None:
            self.command_position(
                automatic_reason="incomplete NN observation"
            )
            return

        obs = np.asarray(
            obs_1d,
            dtype=np.float32,
        ).reshape(1, 12)

        raw_action, _ = self.model.predict(
            obs,
            deterministic=True,
        )

        raw_action = action_to_4d(raw_action)
        converted = convert_action(
            raw_action,
            mass_kg=self.args.mass_kg,
        )

        rates = np.asarray(
            converted["omega_body_deg_s"],
            dtype=np.float64,
        )
        thrust = float(
            converted["collective_thrust_m_s2"]
        )

        # Short bumpless-ish entry: ramp rates from zero and collective
        # thrust from 1 g to the policy command.
        if self.args.nn_ramp_s > 0.0:
            alpha = clamp(
                (now - self.nn_start_time)
                / self.args.nn_ramp_s,
                0.0,
                1.0,
            )
        else:
            alpha = 1.0

        rates = alpha * rates
        thrust = 9.81 + alpha * (thrust - 9.81)

        info = self.hardware.send_ctbr(
            rates[0],
            rates[1],
            rates[2],
            thrust,
        )

        self.last_nn_raw_action = raw_action.copy()
        self.last_nn_info = info

        # Persistent physical thrust saturation is a sign that the learned
        # controller is asking more than this 42.9 g legacy-prop vehicle
        # can produce. Fall back to position control rather than stay pinned.
        if info["thrust_saturated"]:
            if self.nn_saturation_start is None:
                self.nn_saturation_start = now
            elif (
                now - self.nn_saturation_start
                >= self.args.nn_saturation_timeout_s
            ):
                self.command_position(
                    automatic_reason=(
                        "persistent thrust saturation"
                    )
                )
        else:
            self.nn_saturation_start = None

    def update_landing(self, now):
        self.send_position_transition(now)

        elapsed = now - self.transition_start_time
        p = position_tuple(self.state)
        v = velocity_tuple(self.state)

        if p is None or v is None:
            return

        near_ground = (
            p[2]
            <= self.ground_position[2]
            + self.args.touchdown_height_tolerance
        )

        low_vertical_speed = (
            abs(v[2])
            <= self.args.touchdown_vz_tolerance
        )

        if (
            elapsed >= self.args.landing_duration
            and near_ground
            and low_vertical_speed
        ):
            self.normal_stop_and_disarm(
                "touchdown detected"
            )
            return

        if (
            elapsed
            >= self.args.landing_duration
            + self.args.landing_extra_timeout_s
        ):
            # Do not cut the motors if Vicon still says that the vehicle
            # has not reached the ground. Abort the landing and recover
            # to the normal hover target instead.
            if near_ground:
                self.normal_stop_and_disarm(
                    "landing timeout reached at ground"
                )
            else:
                print(
                    "Landing timeout but vehicle is not near ground; "
                    "aborting landing and returning to hover."
                )
                self.begin_position_transition(
                    STATE_RETURN_POSITION,
                    self.hover,
                    self.args.hover_yaw_deg,
                    self.args.return_duration,
                )

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def airborne_mode(self):
        return self.mode in (
            STATE_TAKEOFF,
            STATE_POSITION_HOLD,
            STATE_RETURN_POSITION,
            STATE_NN,
            STATE_LANDING,
        )

    def check_vicon_watchdog(self, now):
        pose_age_ms, vel_age_ms = data_age_ms(
            self.state,
            now,
        )

        if self.mode in (
            STATE_DISARMED,
            STATE_STOPPED,
        ):
            return

        # Hard pose loss: position control is no longer trustworthy.
        if pose_age_ms > self.args.vicon_hard_timeout_ms:
            self.emergency_stop(
                "Vicon pose age %.1f ms > hard limit %.1f ms"
                % (
                    pose_age_ms,
                    self.args.vicon_hard_timeout_ms,
                )
            )
            return

        # NN specifically requires fresh pose + velocity + angular rate.
        if self.mode == STATE_NN:
            if (
                pose_age_ms > self.args.vicon_soft_timeout_ms
                or vel_age_ms > self.args.vicon_soft_timeout_ms
                or self.state["omega_body_est"] is None
            ):
                self.command_position(
                    automatic_reason=(
                        "Vicon state stale "
                        "(pose %.1f ms, velocity %.1f ms)"
                        % (pose_age_ms, vel_age_ms)
                    )
                )

    def check_geofence(self):
        if not self.airborne_mode():
            return

        p = position_tuple(self.state)
        ypr = attitude_ypr_rad(self.state)

        if p is None or ypr is None:
            return

        x, y, z = p
        yaw, pitch, roll = ypr

        # Hard bounds: stop the experiment.
        if (
            abs(x) > self.args.hard_xy_limit
            or abs(y) > self.args.hard_xy_limit
            or z > self.args.hard_z_max
            or z < self.args.hard_z_min
            or abs(math.degrees(roll)) > self.args.hard_tilt_deg
            or abs(math.degrees(pitch)) > self.args.hard_tilt_deg
        ):
            self.emergency_stop(
                "hard flight-envelope violation"
            )
            return

        # Softer NN-only boundary: give control back to the onboard
        # position controller.
        if self.mode == STATE_NN:
            if (
                abs(x) > self.args.nn_xy_limit
                or abs(y) > self.args.nn_xy_limit
                or z > self.args.nn_z_max
                or z < self.args.nn_z_min
                or abs(math.degrees(roll))
                > self.args.nn_tilt_limit_deg
                or abs(math.degrees(pitch))
                > self.args.nn_tilt_limit_deg
            ):
                self.command_position(
                    automatic_reason=(
                        "NN safety envelope exceeded"
                    )
                )

    def check_supervisor(self, now):
        if self.mode in (
            STATE_DISARMED,
            STATE_STOPPED,
        ):
            return

        if now - self.last_supervisor_check < 1.0:
            return

        self.last_supervisor_check = now
        armed = self.link.is_armed(default=None)

        if armed is False:
            print(
                "Supervisor reports DISARMED; returning to DISARMED state."
            )
            self.set_mode(STATE_DISARMED)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def print_help(self):
        print()
        print("Commands")
        print("--------")
        print("  arm / a       arm + low-thrust motor idle")
        print("  takeoff / t   position-control takeoff to hover")
        print("  nn / n        enable selected FlightLxx policy")
        print("  position / p  disable NN and return to hover")
        print("  land / l      position-control landing + disarm")
        print("  stop          immediate normal STOP + disarm")
        print("  kill / k      supervisor emergency stop")
        print("  status / s    print current state")
        print("  help / h      show commands")
        print("  quit / q      exit only when disarmed/stopped")
        print()
        print(
            "Selected NN model: %s"
            % self.args.model
        )
        print()

    def read_command(self):
        try:
            readable, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.0,
            )
        except (ValueError, OSError):
            return None

        if not readable:
            return None

        line = sys.stdin.readline()

        if line == "":
            return None

        return line.strip().lower()

    def handle_command(self, command):
        if not command:
            return

        if command in ("arm", "a"):
            self.command_arm()

        elif command in ("takeoff", "t"):
            self.command_takeoff()

        elif command in ("nn", "n"):
            self.command_nn()

        elif command in ("position", "p", "pos"):
            self.command_position()

        elif command in ("land", "l"):
            self.command_land()

        elif command == "stop":
            self.normal_stop_and_disarm(
                "manual STOP command"
            )

        elif command in ("kill", "k"):
            self.emergency_stop(
                "manual emergency stop"
            )

        elif command in ("status", "s"):
            self.print_status(
                time.monotonic(),
                force=True,
            )

        elif command in ("help", "h", "?"):
            self.print_help()

        elif command in ("quit", "q", "exit"):
            if self.mode in (
                STATE_DISARMED,
                STATE_STOPPED,
            ):
                self.running = False
            else:
                print(
                    "QUIT rejected while armed/flying. "
                    "Use land, stop, or kill first."
                )

        else:
            print("Unknown command:", command)
            print("Type 'help'.")

    def print_status(self, now, force=False):
        if (
            not force
            and now - self.last_status_print
            < self.args.status_period_s
        ):
            return

        self.last_status_print = now

        p = position_tuple(self.state)
        v = velocity_tuple(self.state)
        pose_age_ms, vel_age_ms = data_age_ms(
            self.state,
            now,
        )

        if p is None:
            p_text = "n/a"
        else:
            p_text = "[%+.3f %+.3f %+.3f]" % p

        if v is None:
            v_text = "n/a"
        else:
            v_text = "[%+.3f %+.3f %+.3f]" % v

        text = (
            "[%s] p=%s v=%s age=%.1f/%.1f ms"
            % (
                self.mode,
                p_text,
                v_text,
                pose_age_ms,
                vel_age_ms,
            )
        )

        if (
            self.mode == STATE_NN
            and self.last_nn_raw_action is not None
            and self.last_nn_info is not None
        ):
            action_text = np.array2string(
                self.last_nn_raw_action,
                precision=3,
                separator=",",
                max_line_width=80,
            )

            text += (
                " act=%s CTBR=[%+.1f %+.1f %+.1f %.2f]"
                % (
                    action_text,
                    self.last_nn_info[
                        "roll_rate_deg_s"
                    ],
                    self.last_nn_info[
                        "pitch_rate_deg_s"
                    ],
                    self.last_nn_info[
                        "yaw_rate_deg_s"
                    ],
                    self.last_nn_info[
                        "collective_thrust_m_s2"
                    ],
                )
            )

            if self.last_nn_info["thrust_saturated"]:
                text += " THRUST_SAT"

        print(text)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        self.print_help()

        control_period = 1.0 / self.args.control_rate
        vicon_send_period = 1.0 / self.args.vicon_send_rate

        next_control = time.monotonic()
        next_vicon_send = time.monotonic()

        try:
            while self.running:
                self.poll_vicon()
                now = time.monotonic()

                if now >= next_vicon_send:
                    self.send_vicon_to_cf(now)
                    next_vicon_send += vicon_send_period

                    if now - next_vicon_send > 0.5:
                        next_vicon_send = (
                            now + vicon_send_period
                        )

                command = self.read_command()
                if command is not None:
                    self.handle_command(command)

                self.check_vicon_watchdog(now)

                if self.mode == STATE_STOPPED:
                    self.print_status(now)
                    time.sleep(0.002)
                    continue

                self.check_geofence()
                self.check_supervisor(now)

                if now >= next_control:
                    next_control += control_period

                    if now - next_control > 0.5:
                        next_control = (
                            now + control_period
                        )

                    if self.mode == STATE_ARMED_IDLE:
                        self.send_idle()

                    elif self.mode in (
                        STATE_TAKEOFF,
                        STATE_RETURN_POSITION,
                    ):
                        self.send_position_transition(now)

                    elif self.mode == STATE_POSITION_HOLD:
                        self.send_position_hold()

                    elif self.mode == STATE_NN:
                        self.send_nn(now)

                    elif self.mode == STATE_LANDING:
                        self.update_landing(now)

                self.print_status(now)
                time.sleep(0.001)

        except KeyboardInterrupt:
            print("\nCtrl+C received.")

            if self.mode not in (
                STATE_DISARMED,
                STATE_STOPPED,
            ):
                self.normal_stop_and_disarm(
                    "Ctrl+C"
                )

        except Exception as exc:
            print()
            print("FATAL:", repr(exc))

            if self.mode not in (
                STATE_DISARMED,
                STATE_STOPPED,
            ):
                try:
                    self.emergency_stop(
                        "controller exception"
                    )
                except Exception as stop_exc:
                    print(
                        "Emergency stop also raised:",
                        repr(stop_exc),
                    )

            raise


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "CrazyfLxx interactive closed-loop controller."
        )
    )

    # FlightLxx
    parser.add_argument(
        "--model",
        choices=("CTBR", "CTBR+DR", "CTBR+ID"),
        default="CTBR",
        help="FlightLxx policy; default CTBR",
    )
    parser.add_argument(
        "--model-path",
        default=None,
    )
    parser.add_argument(
        "--flightlxx-path",
        default=None,
    )

    # Vicon
    parser.add_argument(
        "--source",
        choices=("vrpn", "mock"),
        default="vrpn",
    )
    parser.add_argument(
        "--tracker",
        default="crazyflie",
    )
    parser.add_argument(
        "--server",
        default="192.168.10.1",
    )
    parser.add_argument(
        "--sensor",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--mock-host",
        default="127.0.0.1",
    )
    parser.add_argument(
        "--mock-port",
        type=int,
        default=5005,
    )
    parser.add_argument(
        "--vicon-fusion",
        choices=("pose", "position"),
        default="pose",
        help=(
            "send full Vicon pose or only position to the "
            "Crazyflie EKF; default pose"
        ),
    )

    # Crazyflie
    parser.add_argument(
        "--uri",
        default="radio://0/100/2M",
    )
    parser.add_argument(
        "--mass-g",
        type=float,
        default=42.9,
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="enable real radio/motor output",
    )

    # References
    parser.add_argument(
        "--hover",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 1.0),
    )
    parser.add_argument(
        "--hover-yaw-deg",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--nn-goal",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 5.0),
    )

    # Rates / transitions
    parser.add_argument(
        "--control-rate",
        type=float,
        default=50.0,
    )
    parser.add_argument(
        "--vicon-send-rate",
        type=float,
        default=100.0,
    )
    parser.add_argument(
        "--idle-thrust",
        type=float,
        default=3.0,
        help=(
            "ground motor-spin collective thrust [m/s^2]; "
            "default 3.0"
        ),
    )
    parser.add_argument(
        "--takeoff-duration",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--return-duration",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--landing-duration",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--landing-extra-timeout-s",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--landing-below-ground-m",
        type=float,
        default=0.04,
    )
    parser.add_argument(
        "--touchdown-height-tolerance",
        type=float,
        default=0.06,
    )
    parser.add_argument(
        "--touchdown-vz-tolerance",
        type=float,
        default=0.20,
    )
    parser.add_argument(
        "--nn-ramp-s",
        type=float,
        default=0.25,
    )

    # Estimator
    parser.add_argument(
        "--ekf-prime-s",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        "--ekf-settle-s",
        type=float,
        default=1.5,
    )

    # Entry checks
    parser.add_argument(
        "--max-ground-xy-error",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--nn-entry-position-tolerance",
        type=float,
        default=0.15,
    )
    parser.add_argument(
        "--nn-entry-speed-tolerance",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--nn-entry-tilt-deg",
        type=float,
        default=15.0,
    )
    parser.add_argument(
        "--nn-entry-yaw-tolerance-deg",
        type=float,
        default=20.0,
    )

    # Vicon safety
    parser.add_argument(
        "--vicon-soft-timeout-ms",
        type=float,
        default=100.0,
        help="NN -> position fallback threshold",
    )
    parser.add_argument(
        "--vicon-hard-timeout-ms",
        type=float,
        default=500.0,
        help="emergency stop threshold",
    )

    # NN flight envelope
    parser.add_argument(
        "--nn-xy-limit",
        type=float,
        default=0.80,
    )
    parser.add_argument(
        "--nn-z-min",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--nn-z-max",
        type=float,
        default=1.80,
    )
    parser.add_argument(
        "--nn-tilt-limit-deg",
        type=float,
        default=45.0,
    )
    parser.add_argument(
        "--nn-saturation-timeout-s",
        type=float,
        default=0.50,
    )

    # Hard flight envelope
    parser.add_argument(
        "--hard-xy-limit",
        type=float,
        default=1.50,
    )
    parser.add_argument(
        "--hard-z-min",
        type=float,
        default=-0.20,
    )
    parser.add_argument(
        "--hard-z-max",
        type=float,
        default=2.50,
    )
    parser.add_argument(
        "--hard-tilt-deg",
        type=float,
        default=70.0,
    )

    parser.add_argument(
        "--status-period-s",
        type=float,
        default=0.5,
    )

    return parser


def validate_args(parser, args):
    positive = (
        "mass_g",
        "control_rate",
        "vicon_send_rate",
        "takeoff_duration",
        "return_duration",
        "landing_duration",
        "vicon_soft_timeout_ms",
        "vicon_hard_timeout_ms",
        "nn_entry_position_tolerance",
        "nn_entry_speed_tolerance",
        "nn_entry_tilt_deg",
        "hard_xy_limit",
        "hard_z_max",
        "hard_tilt_deg",
    )

    for name in positive:
        if getattr(args, name) <= 0.0:
            parser.error("--%s must be > 0" % name.replace("_", "-"))

    if args.idle_thrust < 0.0:
        parser.error("--idle-thrust must be >= 0")

    if (
        args.vicon_hard_timeout_ms
        <= args.vicon_soft_timeout_ms
    ):
        parser.error(
            "--vicon-hard-timeout-ms must be greater than "
            "--vicon-soft-timeout-ms"
        )

    if args.hover[2] <= args.hard_z_min:
        parser.error(
            "hover altitude must be above hard z minimum"
        )


def make_source(args, state, previous_pose):
    if args.source == "vrpn":
        return VrpnSource(
            state,
            previous_pose,
            tracker_name=args.tracker,
            server=args.server,
            sensor=args.sensor,
        )

    return MockUdpSource(
        state,
        previous_pose,
        host=args.mock_host,
        port=args.mock_port,
        sensor=args.sensor,
    )


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    validate_args(parser, args)

    args.mass_kg = args.mass_g / 1000.0

    print("CrazyfLxx closed-loop controller")
    print("===============================")
    print(
        "mode          : %s"
        % ("LIVE" if args.live else "DRY RUN")
    )
    print("Crazyflie URI : %s" % args.uri)
    print("mass          : %.4f kg" % args.mass_kg)
    print(
        "hover target  : [%.2f, %.2f, %.2f] m"
        % tuple(args.hover)
    )
    print("hover yaw     : %.1f deg" % args.hover_yaw_deg)
    print("NN model      : %s" % args.model)
    print(
        "Vicon fusion  : %s"
        % args.vicon_fusion
    )
    print()

    # Model is loaded before opening the radio so a TensorFlow/model error
    # cannot occur after the vehicle is connected.
    flightlxx_root = add_flightlxx_to_python_path(
        args.flightlxx_path
    )

    model, model_path = load_policy(
        model_name=args.model,
        model_path=args.model_path,
    )

    print("FlightLxx root:", flightlxx_root)
    print("Model path    :", model_path)

    state = make_state()
    previous_pose = make_previous_pose()
    source = make_source(
        args,
        state,
        previous_pose,
    )

    print("Vicon source  :", source.description())

    hardware = CrazyflieInterface(
        uri=args.uri,
        mass_kg=args.mass_kg,
        dry_run=not args.live,
        roll_sign=1.0,
        pitch_sign=-1.0,
        yaw_sign=1.0,
    )

    link = CrazyflieFlightLink(hardware)

    controller = CrazyfLxxController(
        args=args,
        model=model,
        model_path=model_path,
        source=source,
        state=state,
        hardware=hardware,
        flight_link=link,
    )

    try:
        hardware.connect()

        controller.wait_for_initial_vicon()
        controller.initialize_estimator()

        print()
        print("Preflight setup complete.")
        print(
            "Vehicle should still be on the ground. "
            "Type 'arm' when ready."
        )

        controller.run()

    finally:
        # Never leave the radio commander active when the program exits.
        try:
            if hardware.connected:
                hardware.stop()
                try:
                    link.disarm()
                except Exception as exc:
                    print(
                        "Warning: disarm during shutdown failed:",
                        exc,
                    )
        finally:
            try:
                hardware.close()
            finally:
                close_fn = getattr(source, "close", None)
                if callable(close_fn):
                    close_fn()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
