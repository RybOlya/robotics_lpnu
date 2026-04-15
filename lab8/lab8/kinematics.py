#!/usr/bin/env python3

import math

from .limits import JOINT_LIMITS, validate_joint_goal


# Simple SO-101 teaching model for FK/IK in the serial bridge.
# Units are meters/radians, frame is base-centric.
BASE_HEIGHT = 0.10
L1 = 0.12
L2 = 0.135
L3 = 0.08


def euler_to_quat(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def fk_pose(joints):
    q0, q1, q2, q3, q4, _ = joints
    pitch1 = q1
    pitch2 = q1 + q2
    pitch3 = q1 + q2 + q3
    r = (
        L1 * math.cos(pitch1)
        + L2 * math.cos(pitch2)
        + L3 * math.cos(pitch3)
    )
    z = (
        BASE_HEIGHT
        + L1 * math.sin(pitch1)
        + L2 * math.sin(pitch2)
        + L3 * math.sin(pitch3)
    )
    x = r * math.cos(q0)
    y = r * math.sin(q0)
    roll = q4
    pitch = pitch3
    yaw = q0
    qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
    return {
        "x": x,
        "y": y,
        "z": z,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
    }


def ik_position(x: float, y: float, z: float, current_gripper: float = 0.5):
    q0 = math.atan2(y, x)
    r = math.sqrt(x * x + y * y)

    # Keep wrist modestly level for basic pick/place teaching.
    desired_pitch = 0.0
    rw = r - L3 * math.cos(desired_pitch)
    zw = z - BASE_HEIGHT - L3 * math.sin(desired_pitch)

    d = (rw * rw + zw * zw - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    d = max(-1.0, min(1.0, d))
    q2 = math.atan2(math.sqrt(max(0.0, 1.0 - d * d)), d)
    q1 = math.atan2(zw, rw) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    q3 = desired_pitch - q1 - q2
    q4 = 0.0

    gripper = max(JOINT_LIMITS["gripper"][0], min(JOINT_LIMITS["gripper"][1], current_gripper))
    goal = [q0, q1, q2, q3, q4, gripper]
    ok, reason = validate_joint_goal(goal)
    if not ok:
        return False, reason, None
    return True, "", goal

