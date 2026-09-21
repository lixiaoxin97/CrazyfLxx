#!/usr/bin/env python3
"""
vicon_to_nn_state.py

CrazyfLxx Step 2:
    Vicon-like state -> FlightLxx 12D neural-network observation

Two interchangeable input sources are supported:

1) Real Vicon/VRPN:
       python3 vicon_to_nn_state.py --source vrpn

2) Local mock sender:
       python3 mock_vicon_sender.py
       python3 vicon_to_nn_state.py --source mock

Both sources feed exactly the same state-processing path.

FlightLxx observation order:
    [x, y, z, yaw, pitch, roll, vx, vy, vz, wx, wy, wz]

Coordinate convention used internally by CrazyfLxx:
    +X = forward
    +Y = left
    +Z = up

Current Vicon world/rigid-body axes:
    +X = right
    +Y = forward
    +Z = up

Real VRPN input is converted once at the VrpnSource boundary:
    x_control =  y_vicon
    y_control = -x_vicon
    z_control =  z_vicon

Mock input is assumed to already use the internal/control convention.
"""

import argparse
import json
import math
import socket
import sys
import time
from datetime import datetime

import numpy as np


# ----------------------------------------------------------------------
# Quaternion helpers -- convention: [x, y, z, w]
# ----------------------------------------------------------------------

def quat_normalize(q):
    x, y, z, w = map(float, q)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return None
    return (x/n, y/n, z/n, w/n)


def quat_dot(a, b):
    return sum(float(x) * float(y) for x, y in zip(a, b))


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )



# ----------------------------------------------------------------------
# Vicon frame -> control / FlightLxx frame
#
# Vicon:
#   +X = right
#   +Y = forward
#   +Z = up
#
# Control / FlightLxx / Crazyflie:
#   +X = forward
#   +Y = left
#   +Z = up
#
# Coordinate mapping:
#   x =  y_vicon
#   y = -x_vicon
#   z =  z_vicon
#
# This is a proper rotation: Rz(-90 deg).
# ----------------------------------------------------------------------

_SQRT_HALF = math.sqrt(0.5)

Q_VICON_TO_CONTROL = (
    0.0,
    0.0,
    -_SQRT_HALF,
    _SQRT_HALF,
)


def vicon_vector_to_control(v):
    """Transform a vector from the current Vicon frame to control frame."""
    x, y, z = map(float, v)
    return (
        y,
        -x,
        z,
    )


def vicon_quaternion_to_control(q):
    """
    Transform a Vicon rigid-body orientation to the control frame.

    The current Vicon rigid-body local axes are treated consistently with the
    current Vicon world axes (X=right, Y=forward, Z=up), while the control/body
    axes are X=forward, Y=left, Z=up.

    Re-expressing both world and body coordinates gives:

        R_control = C * R_vicon * C^T

    where C = Rz(-90 deg).
    """
    q = quat_normalize(q)
    if q is None:
        return None

    qc = Q_VICON_TO_CONTROL
    q_control = quat_multiply(
        quat_multiply(qc, q),
        quat_conjugate(qc),
    )
    return quat_normalize(q_control)

def quat_to_euler_zyx_rad(q):
    """
    Quaternion [x,y,z,w] -> [yaw,pitch,roll] [rad].
    ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    q = quat_normalize(q)
    if q is None:
        return None

    x, y, z, w = q

    roll = math.atan2(
        2.0 * (w*x + y*z),
        1.0 - 2.0 * (x*x + y*y),
    )

    s = 2.0 * (w*y - z*x)
    s = max(-1.0, min(1.0, s))
    pitch = math.asin(s)

    yaw = math.atan2(
        2.0 * (w*z + x*y),
        1.0 - 2.0 * (y*y + z*z),
    )

    return (yaw, pitch, roll)


def timestamp_dt_seconds(t_new, t_old):
    if t_new is None or t_old is None:
        return None

    try:
        d = t_new - t_old
        if hasattr(d, "total_seconds"):
            return float(d.total_seconds())
        return float(d)
    except Exception:
        return None


def timestamp_difference_seconds(t1, t2):
    dt = timestamp_dt_seconds(t1, t2)
    return abs(dt) if dt is not None else None


def body_angular_velocity(q_prev, q_curr, dt):
    """
    Body-frame angular velocity estimate [rad/s] from two orientations.
    """
    if dt is None or dt <= 1e-6:
        return None

    q0 = quat_normalize(q_prev)
    q1 = quat_normalize(q_curr)
    if q0 is None or q1 is None:
        return None

    # q and -q represent the same attitude.
    if quat_dot(q0, q1) < 0.0:
        q1 = tuple(-v for v in q1)

    q_delta = quat_multiply(quat_conjugate(q0), q1)
    q_delta = quat_normalize(q_delta)
    if q_delta is None:
        return None

    x, y, z, w = q_delta

    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    w = max(-1.0, min(1.0, w))
    s = math.sqrt(x*x + y*y + z*z)

    if s < 1e-10:
        return (2.0*x/dt, 2.0*y/dt, 2.0*z/dt)

    angle = 2.0 * math.atan2(s, w)
    axis = (x/s, y/s, z/s)

    return (
        axis[0] * angle / dt,
        axis[1] * angle / dt,
        axis[2] * angle / dt,
    )


# ----------------------------------------------------------------------
# Shared state update -- identical for real VRPN and mock UDP
# ----------------------------------------------------------------------

def make_state():
    return {
        "position": None,
        "quaternion": None,
        "linear_velocity": None,
        "omega_body_est": None,
        "pose_timestamp": None,
        "velocity_timestamp": None,
        "pose_rx_monotonic": None,
        "velocity_rx_monotonic": None,
        "pose_dt": None,
    }


def make_previous_pose():
    return {
        "quaternion": None,
        "timestamp": None,
    }


def update_pose(state, previous_pose, position, quaternion, timestamp):
    p = tuple(float(x) for x in position)
    q = tuple(float(x) for x in quaternion)

    if (
        previous_pose["quaternion"] is not None
        and previous_pose["timestamp"] is not None
    ):
        dt = timestamp_dt_seconds(timestamp, previous_pose["timestamp"])

        if dt is not None and 1e-5 < dt < 0.2:
            omega = body_angular_velocity(
                previous_pose["quaternion"],
                q,
                dt,
            )
            if omega is not None:
                state["omega_body_est"] = omega
                state["pose_dt"] = dt
            else:
                state["omega_body_est"] = None
                state["pose_dt"] = None
        else:
            state["omega_body_est"] = None
            state["pose_dt"] = None

    previous_pose["quaternion"] = q
    previous_pose["timestamp"] = timestamp

    state["position"] = p
    state["quaternion"] = q
    state["pose_timestamp"] = timestamp
    state["pose_rx_monotonic"] = time.monotonic()


def update_velocity(state, velocity, timestamp):
    state["linear_velocity"] = tuple(float(x) for x in velocity)
    state["velocity_timestamp"] = timestamp
    state["velocity_rx_monotonic"] = time.monotonic()


# ----------------------------------------------------------------------
# FlightLxx 12D observation
# ----------------------------------------------------------------------

def build_nn_observation(
    position_vicon,
    quaternion_vicon,
    linear_velocity_world,
    angular_velocity_body,
    physical_hover_position=(0.0, 0.0, 1.0),
    nn_goal_position=(0.0, 0.0, 5.0),
):
    if any(
        value is None
        for value in (
            position_vicon,
            quaternion_vicon,
            linear_velocity_world,
            angular_velocity_body,
        )
    ):
        return None

    euler_zyx = quat_to_euler_zyx_rad(quaternion_vicon)
    if euler_zyx is None:
        return None

    p = np.asarray(position_vicon, dtype=np.float64)
    p_ref_physical = np.asarray(physical_hover_position, dtype=np.float64)
    p_ref_nn = np.asarray(nn_goal_position, dtype=np.float64)

    p_nn = p - p_ref_physical + p_ref_nn

    yaw, pitch, roll = euler_zyx
    vx, vy, vz = map(float, linear_velocity_world)
    wx, wy, wz = map(float, angular_velocity_body)

    obs = np.array(
        [
            p_nn[0], p_nn[1], p_nn[2],
            yaw, pitch, roll,
            vx, vy, vz,
            wx, wy, wz,
        ],
        dtype=np.float32,
    )

    return obs if np.all(np.isfinite(obs)) else None


def fmt_obs(obs):
    if obs is None:
        return "[waiting for complete state]"

    return np.array2string(
        obs,
        precision=5,
        separator=", ",
        max_line_width=200,
    )


# ----------------------------------------------------------------------
# Real VRPN source
# ----------------------------------------------------------------------

class VrpnSource:
    def __init__(self, state, previous_pose, tracker_name, server, sensor):
        try:
            import vrpn
        except ImportError as exc:
            raise RuntimeError(
                "cannot import 'vrpn'; VRPN source is unavailable"
            ) from exc

        self.state = state
        self.previous_pose = previous_pose
        self.sensor = sensor

        self.address = f"{tracker_name}@{server}"
        self.tracker = vrpn.receiver.Tracker(self.address)

        self.tracker.register_change_handler(
            None, self._on_position, "position"
        )
        self.tracker.register_change_handler(
            None, self._on_velocity, "velocity"
        )

    def _on_position(self, userdata, data):
        if int(data.get("sensor", 0)) != self.sensor:
            return

        p_raw = data.get("position")
        q_raw = data.get("quaternion")
        stamp = data.get("time")

        if p_raw is None or q_raw is None or stamp is None:
            return

        p = vicon_vector_to_control(p_raw)
        q = vicon_quaternion_to_control(q_raw)

        if q is None:
            return

        update_pose(
            self.state,
            self.previous_pose,
            p,
            q,
            stamp,
        )

    def _on_velocity(self, userdata, data):
        if int(data.get("sensor", 0)) != self.sensor:
            return

        v_raw = data.get("velocity")
        stamp = data.get("time")

        if v_raw is None or stamp is None:
            return

        v = vicon_vector_to_control(v_raw)
        update_velocity(self.state, v, stamp)

        # Deliberately ignore "future quaternion"/"future delta".

    def poll(self):
        self.tracker.mainloop()

    def description(self):
        return f"VRPN {self.address}"


# ----------------------------------------------------------------------
# Local mock UDP source
# ----------------------------------------------------------------------

def parse_mock_timestamp(value):
    if isinstance(value, datetime):
        return value
    if isinstance(value, str):
        return datetime.fromisoformat(value)
    raise ValueError(f"unsupported mock timestamp: {value!r}")


class MockUdpSource:
    def __init__(self, state, previous_pose, host, port, sensor):
        self.state = state
        self.previous_pose = previous_pose
        self.sensor = sensor
        self.host = host
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

    def _handle_packet(self, packet):
        if int(packet.get("sensor", 0)) != self.sensor:
            return

        packet_type = packet.get("type")
        stamp = parse_mock_timestamp(packet.get("time"))

        if packet_type == "position":
            p = packet.get("position")
            q = packet.get("quaternion")
            if p is not None and q is not None:
                update_pose(
                    self.state,
                    self.previous_pose,
                    p,
                    q,
                    stamp,
                )

        elif packet_type == "velocity":
            v = packet.get("velocity")
            if v is not None:
                update_velocity(self.state, v, stamp)

    def poll(self):
        while True:
            try:
                raw, _addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                break

            try:
                packet = json.loads(raw.decode("utf-8"))
                self._handle_packet(packet)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                print(
                    f"WARNING: ignored invalid mock packet: {exc}",
                    file=sys.stderr,
                )

    def description(self):
        return f"mock UDP {self.host}:{self.port}"

    def close(self):
        self.sock.close()


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Real/mock Vicon state -> FlightLxx 12D observation."
    )

    parser.add_argument(
        "--source",
        choices=("vrpn", "mock"),
        default="vrpn",
        help="input source; default vrpn",
    )

    parser.add_argument(
        "--tracker",
        default="crazyflie",
        help="VRPN tracker name",
    )
    parser.add_argument(
        "--server",
        default="192.168.10.1",
        help="VRPN server",
    )

    parser.add_argument(
        "--mock-host",
        default="127.0.0.1",
        help="mock UDP bind host",
    )
    parser.add_argument(
        "--mock-port",
        type=int,
        default=5005,
        help="mock UDP port",
    )

    parser.add_argument(
        "--sensor",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="terminal print rate [Hz]",
    )
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

    state = make_state()
    previous_pose = make_previous_pose()

    try:
        if args.source == "vrpn":
            source = VrpnSource(
                state,
                previous_pose,
                args.tracker,
                args.server,
                args.sensor,
            )
        else:
            source = MockUdpSource(
                state,
                previous_pose,
                args.mock_host,
                args.mock_port,
                args.sensor,
            )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"source              : {source.description()}")
    print(f"sensor              : {args.sensor}")
    print(f"physical hover [m]  : {tuple(args.physical_hover)}")
    print(f"FlightLxx goal [m]  : {tuple(args.nn_goal)}")
    print()
    print("NN observation:")
    print("[x, y, z, yaw, pitch, roll, vx, vy, vz, wx, wy, wz]")
    print("Ctrl+C to stop.\n")

    period = 1.0 / max(args.rate, 0.1)
    next_print = time.monotonic()

    try:
        while True:
            source.poll()
            now = time.monotonic()

            if now >= next_print:
                next_print = now + period

                pose_age_ms = (
                    1000.0 * (now - state["pose_rx_monotonic"])
                    if state["pose_rx_monotonic"] is not None
                    else float("nan")
                )

                vel_age_ms = (
                    1000.0 * (now - state["velocity_rx_monotonic"])
                    if state["velocity_rx_monotonic"] is not None
                    else float("nan")
                )

                sync_dt = timestamp_difference_seconds(
                    state["pose_timestamp"],
                    state["velocity_timestamp"],
                )
                sync_ms = (
                    1000.0 * sync_dt
                    if sync_dt is not None
                    else float("nan")
                )

                obs = build_nn_observation(
                    state["position"],
                    state["quaternion"],
                    state["linear_velocity"],
                    state["omega_body_est"],
                    args.physical_hover,
                    args.nn_goal,
                )

                print(f"nn_obs = {fmt_obs(obs)}")
                print(
                    f"shape={None if obs is None else obs.shape}, "
                    f"dtype={None if obs is None else obs.dtype}, "
                    f"pose/vel sync={sync_ms:.3f} ms, "
                    f"age={pose_age_ms:.1f}/{vel_age_ms:.1f} ms"
                )
                print()

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    finally:
        if isinstance(source, MockUdpSource):
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
