#!/usr/bin/env python3
"""
vicon_state.py

Minimal Vicon/VRPN rigid-body state reader.

Confirmed from the user's VRPN binding:
  position callback:
    position   -> (x, y, z) [m]
    quaternion -> (qx, qy, qz, qw)

  velocity callback:
    velocity   -> (vx, vy, vz) [m/s]

The binding also exposes "future quaternion" / "future delta", but the observed
"future delta" is not a valid time interval. Therefore angular velocity is
estimated robustly from consecutive Vicon orientation quaternions and their
Vicon timestamps.

Output (after Vicon -> control-frame conversion):
  position       [m]
  roll/pitch/yaw [deg]
  linear velocity[m/s]
  body angular velocity estimate [rad/s] -- from quaternion finite difference

Internal/control frame:
  +X = forward
  +Y = left
  +Z = up

Current raw Vicon frame:
  +X = right
  +Y = forward
  +Z = up

Example:
    python3 vicon_state.py --tracker crazyflie --server 192.168.10.1

Use Ctrl+C to stop.
"""

import argparse
import math
import sys
import time


# ---------------- Quaternion helpers: [x, y, z, w] ----------------

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
    """Hamilton product for quaternions stored as [x, y, z, w]."""
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

def quat_to_rpy_deg(q):
    """Quaternion [x,y,z,w] -> roll, pitch, yaw using ZYX convention."""
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

    k = 180.0 / math.pi
    return (roll*k, pitch*k, yaw*k)


def timestamp_dt_seconds(t_new, t_old):
    """Works with datetime timestamps (as observed) and numeric timestamps."""
    if t_new is None or t_old is None:
        return None

    try:
        delta = t_new - t_old
        if hasattr(delta, "total_seconds"):
            return float(delta.total_seconds())
        return float(delta)
    except Exception:
        return None


def body_angular_velocity(q_prev, q_curr, dt):
    """
    Estimate body-frame angular velocity [rad/s].

    Assumption:
      the VRPN orientation quaternion represents rigid-body orientation in the
      Vicon world frame (the usual pose convention).

    For R = world_from_body:
        Delta_R_body = R_prev^T * R_curr
    which in quaternion form is:
        q_delta = inverse(q_prev) * q_curr

    For sufficiently small frame intervals, rotvec(q_delta) / dt is the
    body-frame angular velocity to first order.
    """
    if dt is None or dt <= 1e-6:
        return None

    q0 = quat_normalize(q_prev)
    q1 = quat_normalize(q_curr)
    if q0 is None or q1 is None:
        return None

    # q and -q encode the same orientation. Keep quaternion sign continuous.
    if quat_dot(q0, q1) < 0.0:
        q1 = tuple(-v for v in q1)

    q_delta = quat_multiply(quat_conjugate(q0), q1)
    q_delta = quat_normalize(q_delta)
    if q_delta is None:
        return None

    x, y, z, w = q_delta

    # Use the shortest equivalent rotation.
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    w = max(-1.0, min(1.0, w))
    s = math.sqrt(x*x + y*y + z*z)

    if s < 1e-10:
        # Small-angle approximation: rotvec ~= 2 * vector_part
        return (2.0*x/dt, 2.0*y/dt, 2.0*z/dt)

    angle = 2.0 * math.atan2(s, w)
    axis = (x/s, y/s, z/s)

    return (
        axis[0] * angle / dt,
        axis[1] * angle / dt,
        axis[2] * angle / dt,
    )


def fmt3(v, decimals=4):
    if v is None:
        return "[   ---    ---    --- ]"
    f = "{:+." + str(decimals) + "f}"
    return "[" + " ".join(f.format(float(x)) for x in v) + "]"


# ---------------- Main ----------------

def main():
    parser = argparse.ArgumentParser(
        description="Read Vicon rigid-body state through VRPN."
    )
    parser.add_argument("--tracker", default="crazyflie",
                        help="Vicon rigid-body / VRPN tracker name")
    parser.add_argument("--server", default="192.168.10.1",
                        help="VRPN server hostname/IP, optionally host:port")
    parser.add_argument("--sensor", type=int, default=0,
                        help="VRPN sensor id; default 0")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="terminal print rate [Hz]; default 20")
    args = parser.parse_args()

    try:
        import vrpn
    except ImportError:
        print("ERROR: cannot import 'vrpn'.", file=sys.stderr)
        print('Check with: python3 -c "import vrpn; print(vrpn)"',
              file=sys.stderr)
        return 2

    address = f"{args.tracker}@{args.server}"

    state = {
        "position": None,
        "quaternion": None,
        "rpy_deg": None,
        "linear_velocity": None,
        "omega_body_est": None,

        "pose_timestamp": None,
        "velocity_timestamp": None,

        "pose_rx_monotonic": None,
        "velocity_rx_monotonic": None,

        "pose_dt": None,
    }

    previous = {
        "quaternion": None,
        "timestamp": None,
    }

    def on_position(userdata, data):
        if int(data.get("sensor", 0)) != args.sensor:
            return

        p_raw = data.get("position")
        q_raw = data.get("quaternion")
        stamp = data.get("time")

        if p_raw is None or q_raw is None:
            return

        p = vicon_vector_to_control(p_raw)
        q = vicon_quaternion_to_control(q_raw)
        if q is None:
            return

        # Estimate angular velocity from consecutive converted Vicon callbacks,
        # not from the slower terminal printing loop.
        if previous["quaternion"] is not None and previous["timestamp"] is not None:
            dt = timestamp_dt_seconds(stamp, previous["timestamp"])

            # Reject obviously invalid gaps for the instantaneous estimate.
            # If timing is invalid, clear the previous estimate instead of
            # leaving stale angular-rate data visible as if it were current.
            if dt is not None and 1e-5 < dt < 0.2:
                omega = body_angular_velocity(
                    previous["quaternion"], q, dt
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

        previous["quaternion"] = q
        previous["timestamp"] = stamp

        state["position"] = p
        state["quaternion"] = q
        state["rpy_deg"] = quat_to_rpy_deg(q)
        state["pose_timestamp"] = stamp
        state["pose_rx_monotonic"] = time.monotonic()

    def on_velocity(userdata, data):
        if int(data.get("sensor", 0)) != args.sensor:
            return

        v_raw = data.get("velocity")
        if v_raw is not None:
            state["linear_velocity"] = vicon_vector_to_control(v_raw)

        state["velocity_timestamp"] = data.get("time")
        state["velocity_rx_monotonic"] = time.monotonic()

        # Intentionally ignore:
        #   data["future quaternion"]
        #   data["future delta"]
        # because the observed future-delta value is not decoded correctly
        # by this Python VRPN binding/build.

    tracker = vrpn.receiver.Tracker(address)
    tracker.register_change_handler(None, on_position, "position")
    tracker.register_change_handler(None, on_velocity, "velocity")

    print(f"VRPN tracker : {address}")
    print(f"sensor       : {args.sensor}")
    print("frame        : +X forward, +Y left, +Z up")
    print("angular rate : quaternion finite difference (body-frame estimate)")
    print("Ctrl+C to stop\n")

    period = 1.0 / max(args.rate, 0.1)
    next_print = time.monotonic()

    try:
        while True:
            tracker.mainloop()

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
                pose_dt_ms = (
                    1000.0 * state["pose_dt"]
                    if state["pose_dt"] is not None
                    else float("nan")
                )

                print(
                    f"p[m]       = {fmt3(state['position'])} | "
                    f"rpy[deg] = {fmt3(state['rpy_deg'], 2)}"
                )
                print(
                    f"v_world[m/s] = {fmt3(state['linear_velocity'])} | "
                    f"omega_body_est[rad/s] = {fmt3(state['omega_body_est'])}"
                )
                print(
                    f"Vicon pose dt = {pose_dt_ms:7.3f} ms | "
                    f"age pose/vel = {pose_age_ms:6.1f}/{vel_age_ms:6.1f} ms | "
                    f"timestamp = {state['pose_timestamp']}"
                )
                print()

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
