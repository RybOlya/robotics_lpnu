# Robotics Introduction Labs

Hands-on labs for learning ROS2 and robotics simulation with Gazebo.

---

## Prerequisites

- **Ubuntu 24.04** (native, WSL2, dual boot, or VirtualBox)
- **Git** and **GitHub account** - [GitHub Hello World Guide](https://docs.github.com/en/get-started/start-your-journey/hello-world)
- **30 GB free disk space**
- **8 GB RAM** (16 GB recommended)

**Don't have Ubuntu 24.04?** See the [Installation Guide](docs/INSTALLATION_GUIDE.md) for setup instructions.

---
### 1. Clone Repository

```bash
cd ~
git clone https://github.com/RybOlya/robotics_lpnu.git
cd robotics_lpnu
```

### 2. Build Docker Image

```bash
./scripts/cmd build-docker
```

This takes 10-15 minutes on first run.

### 3. Run Container

```bash
./scripts/cmd run
```
### 4. Install and open VScode

```bash
code .
```


#### Open Additional Terminals

```bash
# In a new terminal window
./scripts/cmd bash
```
---

## Labs

| Lab | Topic | Description |
|-----|-------|-------------|
| **[Lab 1](lab1/README.md)** | Introduction to ROS2 and Simulation | Create 4-wheel mobile robot with sensors |
| **[Lab 2](lab2/README.md)** | Adding Objects to Simulation | Create and position objects in Gazebo |
| **[Lab 3](lab3/README.md)** | Moving Mobile Robots in Simulation | Differential drive kinematics, geometric paths. You also need to run the TurtleBot3 setup - will be used in the next lab. |
| **[Lab 4](lab4/README.md)** | Dead Reckoning | Integrate velocity commands, compare with Gazebo odometry. Runs with lab3 circle trajectory. |
| **[Lab 5](lab5/README.md)** | Obstacle Avoidance | Potential fields, Bug, DWA, or RL. TurtleBot3 in maze/forest. |
| **[Lab 6](lab6/README.md)** | Motion Planning (Nav2) | Nav2 with static map: global/local planners, TurtleBot3 in Gazebo. |

### Development Workflow

**The typical development cycle:**

1. **Edit files** on your host machine (outside container) using your favorite IDE
   - Files in `/home/YOUR_USER/robotics_lpnu/` are automatically visible inside container at `/opt/ws/src/code/`

2. **Enter container** (if not already inside):
   ```bash
   ./scripts/cmd bash
   ```

3. **Build your code** inside container:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. **Run and test** inside container:
   ```bash
   gz sim worlds/robot.sdf        # For Lab 1
   ros2 launch lab2 ...            # For Lab 2
   ```

5. **Repeat** steps 1-4 as needed

**Important**: Changes to files are instant (no rebuild needed for world files, Python changes need rebuild).

### Useful Workspace Commands

```bash
# Build only specific packages
colcon build --packages-select lab1

# Build with symbolic links (faster for Python)
colcon build --symlink-install

# In case of building at wrong level for example - clean workspace (remove build artifacts)
rm -rf ./build ./install ./log
```

**Note:** After cleaning, you must rebuild with `colcon build` before running any ROS2 commands.

---

## Repository Structure

```
robotics_lpnu/
├── lab1/                          # Lab 1: Building a Robot in Gazebo
│   ├── worlds/robot.sdf          # Robot world file
│   └── README.md                  # Lab 1 instructions
├── lab2/                          # Lab 2: ROS2 Integration
│   ├── lab2/                      # Python nodes (you create)
│   ├── launch/                    # Launch files (you create)
│   ├── config/robot.rviz         # RViz configuration
│   └── README.md                  # Lab 2 instructions
├── docs/                          
│   └── INSTALLATION_GUIDE.md      # OS setup guide
├── docker/                        
│   ├── Dockerfile                 # ROS2 + Gazebo image
│   └── entrypoint.bash           
└── scripts/                       
    └── cmd                        # Docker helper script
```

---

## Troubleshooting

### Docker permission denied
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### GUI not working
```bash
# Linux: Allow X11 connections
xhost +local:docker

# WSL2: Update WSLg
wsl --update
```

### Container won't start
```bash
# Remove inactive containers
docker system prune -a
./scripts/cmd build-docker
```

### ROS2 commands not found / Package issues
```bash
# Try re-sourcing the environment
source /opt/ros/jazzy/setup.bash
source /opt/ws/install/setup.bash

# Or restart the container
exit
./scripts/cmd run
```

---

## Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [SDFormat Specification](http://sdformat.org/)
- [Docker for Robotics](https://articulatedrobotics.xyz/category/docker-for-robotics)
- [Installation Guide](docs/INSTALLATION_GUIDE.md) - OS setup
