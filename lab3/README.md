# Lab 3: Moving Mobile Robots in Simulation

## Learning Goals

- Differential drive kinematics and velocity commands
- Odometry feedback for path following
- RViz2 trajectory visualization

**Further reading:** 

[Calculating Wheel Odometry for a Differential Drive Robot](https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/).

[Wheel Odometry Model for Differential Drive Robotics](https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299)

---

## Setup
**Copy your `robot.sdf` from previous labs to the `worlds` folder, then:**

```bash
cd /opt/ws
colcon build --packages-select lab3
source install/setup.bash
```
---

## Launch Options

**Full bringup** (Gazebo + path publisher + RViz2):
```bash
ros2 launch lab3 bringup.launch.py
```
**Gazebo only** (manual testing):
```bash
ros2 launch lab3 gazebo.launch.py
```
---

## Base Code

**Square** (`square_path.py`): Odometry-based, move forward + turn 90°, repeat 4 times.

**Circle** (`circle_path.py`): Timed motion, constant linear + angular velocity for one full circle.

**Velocity publisher** (`velocity_publisher`): Publish constant (v, w), prints wheel speeds from `diff_drive_math`.

**Odom path publisher** (`odom_path_publisher`): Subscribes to odometry, publishes `/path` for RViz2.

---

## Parameters (tune for your robot)

| Parameter | Default | Description |
|-----------|---------|-------------|
| side_length | 2.0 | Square side length (m) |
| linear_speed | 0.4 | Forward speed (m/s) |
| angular_speed | 0.8 | Turn rate (rad/s) |

```bash
ros2 run lab3 square_path --ros-args -p side_length:=2.5
```

---

## Tasks

### Task 1: Run square and circle

```bash
ros2 run lab3 square_path
ros2 run lab3 circle_path
```

### Task 2: Implement figure-8

You have `square_path.py` and `circle_path.py`. Implement `figure_8_path.py`:
- Figure-8 = two circles: first left (w>0), then right (w<0)
- Use the same timed motion as `circle_path`

### Task 3: RViz2 visualization

Launch bringup, then run a path. RViz2 shows the trajectory on `/path`.

**Paste a screenshot of the trajectory in your report.**

---

## Deliverables

1. Implemented `figure_8_path.py`
2. Screenshot of trajectory from RViz2
3. Best parameters for square path
4. Brief answers: What is differential drive? Why might the square drift?

---

## Troubleshooting

**RViz shows no path:** Fixed Frame must be `odom`. Add Path display, topic `/path`.

## Code Structure

```
lab3/
├── lab3/
│   ├── diff_drive_math.py
│   ├── velocity_publisher.py
│   ├── odom_path_publisher.py
│   ├── square_path.py
│   ├── circle_path.py
│   └── figure_8_path.py        # Student task
├── launch/
│   ├── gazebo.launch.py
│   └── bringup.launch.py
├── rviz/
│   └── trajectory.rviz
└── worlds/
    └── robot.sdf
```
