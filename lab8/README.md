# Laboratory 8 — SO-101 Joint/EE Control

## Overview

`lab8` is a ROS 2 serial bridge for the real SO-101 arm. It provides:
- live joint reading (`/joint_states`)
- RViz robot visualization (URDF + TF)
- safe bounded joint goals
- FK output (`/ee_pose`) and EE marker (`/ee_marker`)
- basic IK from `/ee_goal` to joint commands
- motion sequence execution from YAML (pick/place style)

## SO-101 References
- [Lerobot instructions](https://huggingface.co/docs/lerobot/so101)
- [SO101 ROS2 DeepWiki](https://deepwiki.com/msf4-0/so101_ros2)
- [so101_control.py example](https://github.com/msf4-0/so101_ros2/blob/69b9ef6884002cfe0098415d9ad2f58707bf96f3/so101_ros2/so101_control.py)

## Build

```bash
cd /home/orybe/uni/robotics_lpnu
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lab8 --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## Run

```bash
ros2 launch lab8 so101_real.launch.py \
  port:=/dev/ttyACM0 \
  baud_rate:=1000000
```

Launch starts:
- `joint_state_reader`
- `robot_state_publisher` (`so101_description/urdf/so101_new_calib.urdf`)
- `rviz2`

## Student Workflow

1. **Read joints first (required).**
   - Run:
   ```bash
   ros2 topic echo /joint_states
   ```
   - Move one joint manually up/down and observe value/sign changes.
   - Identify each joint's near-zero point before sending commands.

2. **Then create pick/place motion.**
   - Edit `lab8/config/pick_place_basic.yaml`
   - Add new joint targets
   - Run motion and iterate safely

## Joint Goals

Safe preset:
```bash
ros2 run lab8 send_goal -- --preset ready
```

Custom full goal:
```bash
ros2 run lab8 send_goal -- --joints 0.2 -1.2 1.2 0.4 0.0 0.7
```

Single-joint override from safe base:
```bash
ros2 run lab8 send_goal -- --preset ready --shoulder_pan 0.3 --gripper 0.2
```

Safety behavior:
- out-of-limit goals are rejected
- recommended first motions: `shoulder_pan` and `gripper`

## Motion Program

Run basic pick/place sequence:
```bash
ros2 run lab8 run_motion -- --file "$(ros2 pkg prefix lab8)/share/lab8/config/pick_place_basic.yaml"
```

## EE Goals and FK

Send EE goal:
```bash
ros2 run lab8 send_ee_goal -- --x 0.26 --y 0.00 --z 0.08
```

Check FK EE pose:
```bash
ros2 topic echo /ee_pose --once
```

Working beginner workspace:
- `x`: `0.15 .. 0.28`
- `y`: `-0.10 .. 0.10`
- `z`: `0.05 .. 0.22`

## Troubleshooting

- **Robot not found on serial**
  - Check port: `ls /dev/ttyACM*`
  - Verify launch `port:=...`
  - Check permissions (`dialout` group)

- **RViz shows empty scene**
  - Ensure launch is running (includes `robot_state_publisher`)
  - Check `/joint_states`: `ros2 topic echo /joint_states --once`
  - Verify RViz fixed frame is `base` or `base_link`

- **EE goal published but no movement**
  - Pose may be unreachable or rejected by limits
  - Try a reachable pose like `x=0.26 y=0.00 z=0.08`
  - Watch `joint_state_reader` logs for reject messages

- **Gripper opens/closes wrong range**
  - Tune `gripper.range_min/max` in calibration YAML
  - Restart launch after edits


