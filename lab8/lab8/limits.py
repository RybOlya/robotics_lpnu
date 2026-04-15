JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.74533, 1.5708),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.79253, 2.79253),
    "gripper": (0.0, 1.0),
}

SAFE_PRESETS = {
    "home": [0.0, -1.2, 1.2, 0.2, 0.0, 0.8],
    "ready": [0.0, -1.0, 1.1, 0.5, 0.0, 0.8],
    "open_gripper": [0.0,-1.8,1.5,0.7,-0.0,1.0],
    "close_gripper": [0.0,-1.8,1.5,0.7,-0.0,0.1],
}


def validate_joint_goal(goal):
    if len(goal) != len(JOINT_ORDER):
        return False, f"Expected {len(JOINT_ORDER)} values, got {len(goal)}"
    for idx, joint_name in enumerate(JOINT_ORDER):
        lower, upper = JOINT_LIMITS[joint_name]
        value = float(goal[idx])
        if value < lower or value > upper:
            return (
                False,
                f"{joint_name}={value:.4f} outside [{lower:.4f}, {upper:.4f}]",
            )
    return True, ""

