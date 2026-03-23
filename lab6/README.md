# Lab 6: Motion Planning for Mobile Robots (Nav2)

**Objective:** Get familiar with the **Nav2 stack** (map server, localization, planner, controller, costmaps, behavior tree) by running it in simulation and **reading the official Nav2 docs** linked under **Further reading** below (start with the main site, then planner server and controller server when you tune YAML). Send goals in `room_nav2`, and **edit** `nav2_params.yaml` until the robot moves smoothly, the **local costmap** follows obstacles without too much blur or “drift,” and the robot **reaches the goal** in a sensible way. The first settings in the repo are **on purpose a bit bad** (for example **fast** motion and **slow** costmap/controller updates). Your job is to change rates, speed limits, inflation, and goal tolerances until things work better.

---

## Learning goals

- Start Nav2 with a saved map and AMCL on the TurtleBot3 in Gazebo.
- Set the robot pose and a goal in RViz.
- Know the difference between **global** and **local** planning, and connect YAML settings to what you see in RViz and Gazebo.
- **Change** Nav2 settings (costmaps, controller, goal checker) to get better motion, clearer obstacles on the local map, and better stops at the goal.
- *(Optional)* Connect Dijkstra-style vs A*-style ideas to NavFn, or try different **global** / **local** planner **plugins**.

**Further reading**

- [Nav2 documentation](https://docs.nav2.org/)
- [Nav2 Planner Server](https://docs.nav2.org/configuration/packages/configuring-planner-server.html)
- [Nav2 Controller Server](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)

---


## Setup

Rebuild the Docker image the first time (so Nav2 packages are installed):

```bash
./scripts/cmd build-docker
```

Inside the container:

```bash
cd /opt/ws
colcon build --packages-select lab6
source install/setup.bash
```

---

## Launch

```bash
ros2 launch lab6 nav2_room_bringup.launch.py
```

This starts:

- Gazebo with **`room_nav2`** (8×8 m room, perimeter walls, and ten fixed box obstacles).
- TurtleBot3 at (0, 0).
- Nav2: **map_server**, **AMCL**, planner, controller, behavior tree, etc.

---

## Tasks

**Main track:** run the stack (**task 1**) and **tune parameters** (**task 2**) so driving feels OK: improve the robot drive performance; the **local costmap** updates **clearly** (especially when the robot turns), and the robot **reaches the goal** without too much trouble. **Tasks 3 and 4 are optional** (try other **planner plugins**).

**Global** vs **local** (short):

- **Global planner** (`planner_server`): Uses the **saved map** on disk and draws the **global path** in RViz. It does **not** build that path from the laser step by step.

- **Local controller** (`controller_server` / **FollowPath**): Runs **while the robot moves**, follows the global path, uses the **local costmap** (laser, small rolling map) for nearby obstacles, and sends **`/cmd_vel`**.

So: **global** = path on the **saved map**; **local** = **use sensors** around the robot while driving.

### 1. First navigation run

1. Do **Setup** and **Launch** so the package builds and Nav2 starts.
2. In RViz, set **Fixed Frame** to `map` if needed so the map, path, and robot line up.
3. Use **2D Pose Estimate** so the robot in RViz matches Gazebo. Wait if needed; **2D Pose Estimate** again if the map and room do not match.
4. When the pose looks stable, use **Nav2 Goal** for a target. If nothing happens, wait, fix **2D Pose Estimate**, try again. Read the terminal for errors.
5. Watch the global path, local plan, and costmaps.

### 2. Parameter experiments (main lab work)

The first version of **`lab6/config/nav2_params.yaml`** is **meant to be hard** (for example **fast** robot motion and **slow** costmap/controller updates). At first you may see: local map **blur** or **lag** when turning, **messy** stops at the goal, or **wobble** near obstacles. **Change settings** until it improves.

**Try to get:**

- **Local** costmap that **sticks** to obstacles without sliding around too much when the robot rotates.
- **Motion** that feels under control (you may need to **lower** `max_vel_x` / turn speed if the sim is unstable).
- A **goal stop** that is good enough for you (position + angle), knowing sim is noisy.

Use the list below (and other keys in the same file). Change **one setting at a time**, watch RViz and Gazebo, and write what you see. Use your notes for the **deliverable table**.

**Inflation** (in `nav2_params.yaml`, under `global_costmap` / `local_costmap` → `inflation_layer`)

- **`inflation_radius`** — How wide the “cushion” around walls is. **Bigger** → the path stays **farther from walls**; the route may get **longer**; the robot may pass **narrow places** more safely.
- **`cost_scaling_factor`** — How fast the cost drops as you move away from a wall. **Bigger** → the cushion looks **stronger** in RViz; the planner **hates** squeezing next to walls **more**.

**Costmap `resolution`** (same file, `global_costmap` / `local_costmap`)

- **Smaller** numbers (e.g. `0.05` → `0.03` m) → **smaller cells**, finer grid in RViz, more CPU work.
- **Bigger** numbers (e.g. `0.1` m) → **bigger cells**, blockier grid; path may look more **jagged**; thin gaps can **vanish** on the grid.
- The map file uses **`0.05` m** (`lab6/maps/room_nav2.yaml`). If changing **global** resolution causes problems, try only **`local_costmap`** first, or keep **global** at **`0.05`**.

**Costmap rates** (`global_costmap` / `local_costmap` → top-level `ros__parameters`)

- **`update_frequency`** — How often the costmap **recomputes** (Hz). **Higher** → grid **tracks** the laser and the robot **faster** (often **less** smearing when turning); **lower** → less CPU, but obstacles can look **laggy** or **drift**.
- **`publish_frequency`** — How often the costmap is **sent to RViz** (Hz). **Higher** → **smoother** display; **lower** → can look **choppy** even if the map is fine internally.

**Controller server** (`controller_server` → `ros__parameters`)

- **`controller_frequency`** — How often the local controller **runs** (Hz). **Higher** → **snappier** `/cmd_vel` updates (can help next to a fast **local** costmap); **lower** → simpler load, motion can feel **softer** or **late**.

**FollowPath** (`controller_server` → `FollowPath`; default in this lab is **DWB** — other plugins use different names)

- **`max_speed_xy`** — Top speed in the plane (m/s). On a **diff-drive** robot, **`max_vel_x`** is the main forward limit. **Higher** → **faster** motion if nothing else blocks it.
- **`max_vel_theta`** — Max **turn rate** (rad/s). **Higher** → **faster** spins (can make laser/costmap look **worse** when rotating); **lower** → **slower** turns, often **calmer** near walls.
- **`acc_lim_theta`** — Max **angular acceleration** (rad/s²). **Higher** → reaches top spin speed **faster**; **lower** → **gentler** starts/stops of rotation.
- **`decel_lim_theta`** — Max **angular deceleration** (negative rad/s² in YAML). **Higher magnitude** → **sharper** braking of rotation; **lower** → **smoother** stop.

**Goal checker** (`controller_server` → `goal_checker`)

- **`xy_goal_tolerance`** — How close (meters) the robot must sit on the goal point. **Bigger** → “good enough” stop **sooner**; **smaller** → takes **longer** to finish (sim noise can make this hard).
- **`yaw_goal_tolerance`** — How close the final angle must be (radians). **Bigger** → may stop **more sideways**; **smaller** → more **turning in place** at the end.

You can also change **other** keys in `nav2_params.yaml` (speed limits, lookahead, planner options, etc.). The list above is just a guide. **Write down** what you changed so your report is clear.

After you edit YAML, rebuild and launch again:

```bash
cd /opt/ws
colcon build --packages-select lab6
source install/setup.bash
ros2 launch lab6 nav2_room_bringup.launch.py
```

### 3. (Optional) Global planner comparison (NavFn vs Smac 2D)

Do this **after** task 2 works **well enough**. Then you can swap only the **global** planner to compare algorithms.

Change only **`planner_server`** (keep the default local controller unless you also do task 4). You should see changes in the **path line** in RViz (shape, corners, which corridor). For NavFn, `use_astar` ties to Dijkstra-like vs A* ideas; **how the robot drives** still depends a lot on the local controller.

Edit `lab6/config/nav2_params.yaml` under `planner_server`:

- **Default:** `nav2_navfn_planner::NavfnPlanner` with `use_astar: false` (Dijkstra-like) or try `use_astar: true`.
- **Alternative:** Comment out the `GridBased` / NavFn block and use **Smac Planner 2D** (see the comment block above `planner_server` in the same file). You may need extra Smac-specific keys—check the [Nav2 Smac planner docs](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html).

Rebuild, then launch and observe:

```bash
ros2 launch lab6 nav2_room_bringup.launch.py
```

### 4. (Optional) Local controller comparison (DWB vs Regulated Pure Pursuit)

Change only **`FollowPath`** in `controller_server`. If you did task 3, you can keep that global planner; if not, use the default global planner. You should see changes in **how the robot moves** (smoothness, wobble near obstacles, turns, stop at goal)—often clearer than **path-only** changes.

The default **FollowPath** plugin is **DWB** (`dwb_core::DWBLocalPlanner`). Replace it with **Regulated Pure Pursuit**:

- Set `controller_plugins: ["FollowPath"]` and under `FollowPath` use  
  `plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"`  
  with parameters from the [controller tuning guide](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html) (start from Nav2 examples if available).

Keep `enable_stamped_cmd_vel: true` on the controller so `/cmd_vel` works with TurtleBot3 and Gazebo in this course.

---

## Deliverables

**Required**

1. Short report: screenshots or text for navigation in `room_nav2` **before and after** tuning (or two runs that differ), and show what got better (local map, motion, goal) if you can.
2. One table with **at least three** parameter edits from **task 2** and what you saw (costmap, speed, goal, etc.).
3. A short text: **what you changed** to fix local map “drift” / blur and to get **better goal stops**, and how that fits **global vs local** planning in simple words.

**Optional** (only if you did tasks 3 and 4)

4. **Two global planner setups** (e.g. NavFn `use_astar: false` vs `true`, or NavFn vs Smac 2D)—what was different?
5. **Two local controller setups** (e.g. DWB vs Regulated Pure Pursuit)—what was different?
6. A few sentences on Dijkstra / A* or planner choice, if it fits your optional work.
