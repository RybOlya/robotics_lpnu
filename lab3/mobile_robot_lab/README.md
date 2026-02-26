# Moving Mobile Robots in Simulation (ROS2 + Gazebo) — 1-hour Laboratory

This repository contains a ready-to-run **ROS2 Humble + Gazebo** lab (Docker-based) where students control a **differential-drive mobile robot** using `/cmd_vel`, execute **square** and **triangle** trajectories, visualize motion in **RViz2**, and analyze how velocity commands map to wheel speeds and motion curvature.

✅ Runs on:
- **Linux** (recommended)
- **Windows via WSL2** (recommended with Windows 11 + WSLg)

---

## What you will do (Learning Objectives)

By the end of the lab, you will be able to:

1. Launch a mobile robot simulation in **Gazebo**
2. Inspect ROS2 topics: `/cmd_vel`, `/odom`, `/path`
3. Publish velocity commands (`geometry_msgs/Twist`) to move the robot
4. Drive straight and in arcs by changing `(linear.x, angular.z)`
5. Execute **square** and **triangle** paths using timed motion scripts
6. Visualize trajectory in RViz2 using `nav_msgs/Path`
7. Compute/interpret **differential drive wheel speeds** implied by `(v, ω)`
8. Produce a **PDF report** including screenshots + plots of robot position `(x, y)` extracted from ROS topics

---

# Repository Structure (important)

```
mobile_robot_lab/
├── README.md
├── .gitignore
├── docker/
│   ├── Dockerfile
│   └── .dockerignore
├── scripts/
│   ├── run_docker_linux.sh
│   └── run_docker_wsl.sh
└── ros2_ws/
    └── src/
        └── mobile_robot_lab/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/
            │   └── mobile_robot_lab
            ├── launch/
            │   ├── gazebo_launch.py
            │   ├── rviz_launch.py
            │   └── bringup.launch.py
            ├── rviz/
            │   └── trajectory.rviz
            └── mobile_robot_lab/
                ├── __init__.py
                ├── diff_drive_math.py
                ├── velocity_publisher.py
                ├── square_path.py
                ├── triangle_path.py
                └── odom_path_publisher.py
```

---

# 0) Prerequisites

## Linux (recommended)
- Docker installed and working
- Desktop GUI (X11) available

Ubuntu quick install:
```bash
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER
newgrp docker
```

## Windows using WSL2 (recommended: Windows 11 + WSLg)
- WSL2 + Ubuntu installed
- Docker Desktop installed (Windows)
- Docker Desktop → Settings → Resources → WSL integration enabled for your Ubuntu distro

> Windows 11 + WSLg supports GUI apps automatically.  
> On Windows 10 you may need a separate X server (not covered here).

---

# 1) Build Docker image

From the repository root:

```bash
docker build -t ros2_mobile_robot_lab -f docker/Dockerfile .
```

---

# 2) Run Docker container (GUI enabled)

## 2A) Linux
Allow local docker GUI access once per session:
```bash
xhost +local:docker
```

Run container:
```bash
chmod +x scripts/run_docker_linux.sh
./scripts/run_docker_linux.sh
```

## 2B) Windows + WSL2 (inside WSL Ubuntu terminal)
Run container:
```bash
chmod +x scripts/run_docker_wsl.sh
./scripts/run_docker_wsl.sh
```

You should now be inside the container shell, with `/ros2_ws` mounted.

---

# 3) Build the ROS2 workspace (inside container)

```bash
colcon build
source install/setup.bash
```

---

# 4) Start Gazebo + RViz + Path publisher

## Recommended: one-command bringup
```bash
source /ros2_ws/install/setup.bash
ros2 launch mobile_robot_lab bringup.launch.py
```

This starts:
- Gazebo simulation (TurtleBot3 Burger)
- `odom_path_publisher` (subscribes `/odom`, publishes `/path`)
- RViz2 (configured to display `/path`)

---

# 5) Verify topics (inside container)

Open a **second terminal** in the container (or use another shell tab) ie:
```bash
docker exec -it ros2_mobile_robot_lab_container bash
```

then:

```bash
source /ros2_ws/install/setup.bash
ros2 topic list
```

Confirm you can see:
- `/cmd_vel`
- `/odom`
- `/path`

Optional quick checks:
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

Stop `echo` with `Ctrl+C`.

---

# 6) Lab Tasks (Step-by-step)

## Task A — Drive with constant velocity publisher
Run:
```bash
source /ros2_ws/install/setup.bash
ros2 run mobile_robot_lab velocity_publisher
```

The robot should move forward (default parameters).

### A1) Make an arc (change angular velocity)
```bash
ros2 run mobile_robot_lab velocity_publisher --ros-args -p linear_x:=0.15 -p angular_z:=0.50
```

Observe:
- Robot follows a curved trajectory in Gazebo
- RViz path bends accordingly
- Terminal prints the **implied left/right wheel angular velocities** and curve radius

Stop with `Ctrl+C`.

---

## Task B — Square path
Run:
```bash
source /ros2_ws/install/setup.bash
ros2 run mobile_robot_lab square_path
```

Optional: change parameters:
```bash
ros2 run mobile_robot_lab square_path --ros-args -p side_length:=1.5 -p linear_speed:=0.22 -p turn_speed:=0.70
```

Stop (if needed) with `Ctrl+C` (then run `ros2 topic pub` stop is not necessary; scripts stop at the end).

---

## Task C — Triangle path
Run:
```bash
source /ros2_ws/install/setup.bash
ros2 run mobile_robot_lab triangle_path
```

Optional parameter changes:
```bash
ros2 run mobile_robot_lab triangle_path --ros-args -p side_length:=2.0 -p linear_speed:=0.20 -p turn_speed:=0.60
```

---

# 7) Trajectory visualization in RViz2 (required screenshots)

In RViz2:
1. Ensure **Fixed Frame** is set to `odom`
2. Confirm the **Path** display is subscribed to `/path`
3. You should see a growing trajectory line

📸 Take a screenshot of:
- RViz showing the completed square path
- RViz showing the completed triangle path

(You may use your OS screenshot tool.)

---

# 8) REQUIRED: Extract position (x, y) from ROS topics and plot it

Your report must include **plots of position (x, y)** extracted from ROS topics (not only RViz).

We will use the `/path` topic (published by `odom_path_publisher`).  
It contains the accumulated robot positions and can be captured as a single message.

---

## 8A) Install plotting tools (inside container, one-time)
Some base images do not include matplotlib/YAML. Run:

```bash
apt-get update
apt-get install -y python3-matplotlib python3-yaml
```

---

## 8B) Capture `/path` into a file (YAML-like output)

After running a trajectory (square or triangle), capture the current path:

### For square (example)
```bash
source /ros2_ws/install/setup.bash
ros2 topic echo /path --once > path_square.txt
```

### For triangle (example)
```bash
ros2 topic echo /path --once > path_triangle.txt
```

> Tip: If your `/path` already contains older runs, restart the bringup (Gazebo + path publisher) to reset the path:
> - Stop bringup with `Ctrl+C`
> - Relaunch: `ros2 launch mobile_robot_lab bringup.launch.py`

---

## 8C) Generate an XY plot from the captured topic output

### Create square plot image
```bash
python3 - << 'PY'
import yaml
import matplotlib.pyplot as plt

fname = "path_square.txt"

with open(fname, "r", encoding="utf-8") as f:
    raw = f.read()

# ros2 topic echo may include '---' separators; keep the first document
doc = raw.split('---')[0].strip()
data = yaml.safe_load(doc)

xs, ys = [], []
for ps in data.get("poses", []):
    p = ps["pose"]["position"]
    xs.append(p["x"])
    ys.append(p["y"])

plt.figure()
plt.plot(xs, ys)
plt.title("Trajectory (x vs y) from /path topic — SQUARE")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.axis("equal")
plt.grid(True)
plt.savefig("trajectory_square.png", dpi=200)
print("Saved trajectory_square.png")
PY
```

### Create triangle plot image
```bash
python3 - << 'PY'
import yaml
import matplotlib.pyplot as plt

fname = "path_triangle.txt"

with open(fname, "r", encoding="utf-8") as f:
    raw = f.read()

doc = raw.split('---')[0].strip()
data = yaml.safe_load(doc)

xs, ys = [], []
for ps in data.get("poses", []):
    p = ps["pose"]["position"]
    xs.append(p["x"])
    ys.append(p["y"])

plt.figure()
plt.plot(xs, ys)
plt.title("Trajectory (x vs y) from /path topic — TRIANGLE")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.axis("equal")
plt.grid(True)
plt.savefig("trajectory_triangle.png", dpi=200)
print("Saved trajectory_triangle.png")
PY
```

📸 Your PDF report must include screenshots (or inserted images) of:
- `trajectory_square.png`
- `trajectory_triangle.png`

---

# 9) Differential drive analysis (required discussion)

Differential drive kinematics:
- Linear velocity: `v = (v_r + v_l) / 2`
- Angular velocity: `ω = (v_r - v_l) / L`

Where:
- `v_r`, `v_l` are the right/left wheel linear velocities
- `L` is wheel separation

In this lab, the nodes print **implied wheel angular speeds** (rad/s) computed from `(v, ω)`.

### Answer these questions in your report:
1. What motion do you expect when `angular_z = 0` and `linear_x > 0`?
2. If `linear_x` is fixed, what happens to curvature when `angular_z` increases?
3. Why might the robot not return exactly to the start after a square?
4. Compare two runs:
   - Run 1: `linear_x=0.15`, `angular_z=0.20`
   - Run 2: `linear_x=0.15`, `angular_z=0.60`  
   How do the implied wheel speeds differ, and how does the trajectory differ?

---

# 10) REQUIRED Deliverable: PDF Report (submission)

Your laboratory submission must include a **PDF report**.

## Report must contain (minimum):
1. **Student info**: name, date, environment (Linux or WSL)
2. **Setup confirmation screenshots**:
   - Gazebo running with robot visible
   - RViz displaying `/path`
3. **Trajectory results**:
   - Screenshot of RViz showing square trajectory
   - Screenshot of RViz showing triangle trajectory
4. **Plots extracted from topics (REQUIRED)**:
   - `trajectory_square.png` (x vs y from `/path`)
   - `trajectory_triangle.png` (x vs y from `/path`)
5. **Short analysis answers** (Section 9 questions)
6. **Parameter table** listing the values you used for:
   - `linear_x`, `angular_z` (or `linear_speed`, `turn_speed`)
   - `side_length`

## How to create the PDF
Use any method you prefer, for example:
- Write in Word/LibreOffice → Export PDF
- Write in Markdown and convert with `pandoc` (if installed)
- Use Overleaf / LaTeX

---

# Troubleshooting

## Gazebo/RViz window does not open
- Linux: ensure you ran:
  ```bash
  xhost +local:docker
  ```
- WSL: make sure you use Windows 11 + WSLg and that GUI apps work inside WSL

## Container fails with `/dev/dri` error
Edit `scripts/run_docker_*.sh` and remove:
```
--device=/dev/dri:/dev/dri
```

## Robot does not move
- Verify commands publish:
  ```bash
  ros2 topic echo /cmd_vel
  ```
- Ensure Gazebo is running and robot spawned

## RViz shows no path
- Confirm `/path` exists:
  ```bash
  ros2 topic echo /path --once
  ```
- RViz Fixed Frame: `odom`

---

# Typical 1-hour timing (recommended)

- 0–10 min: Docker build/run + workspace build
- 10–20 min: bringup + inspect topics
- 20–35 min: velocity publisher experiments (straight + arcs)
- 35–50 min: square + triangle paths
- 50–60 min: capture `/path`, generate XY plots, gather screenshots, start report

---
End of lab.
