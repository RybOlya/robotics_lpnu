import math
from typing import Tuple

def twist_to_wheel_speeds(
    v: float,
    w: float,
    wheel_radius: float,
    wheel_separation: float
) -> Tuple[float, float]:
    """
    Convert desired body twist (v, w) to wheel angular velocities (rad/s)
    for a differential drive robot.

    v: linear velocity (m/s)
    w: angular velocity (rad/s)
    wheel_radius: wheel radius (m)
    wheel_separation: distance between wheels (m)

    Returns: (omega_left, omega_right) wheel angular velocities (rad/s)
    """
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be > 0")
    if wheel_separation <= 0:
        raise ValueError("wheel_separation must be > 0")

    v_l = v - (w * wheel_separation / 2.0)
    v_r = v + (w * wheel_separation / 2.0)

    omega_l = v_l / wheel_radius
    omega_r = v_r / wheel_radius
    return omega_l, omega_r

def curve_radius(v: float, w: float) -> float:
    """Return instantaneous curvature radius (m). If w==0 -> inf."""
    if abs(w) < 1e-9:
        return math.inf
    return v / w
