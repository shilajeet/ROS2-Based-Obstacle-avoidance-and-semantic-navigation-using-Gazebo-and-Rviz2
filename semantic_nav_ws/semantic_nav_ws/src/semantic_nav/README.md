# Semantic Navigation & Obstacle Avoidance
### ROS2 Humble · Gazebo · TurtleBot3 Simulation

> **Interview-ready robotics project** — runs entirely in simulation (no hardware needed).

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────┐
│                        ROS2 System                              │
│                                                                 │
│  ┌──────────────┐   NavigateToPose   ┌──────────────────────┐  │
│  │   Semantic   │ ─────────────────► │   Nav2 Stack          │  │
│  │  Navigator   │ ◄─ feedback/result │  (AMCL + NavFn + DWB) │  │
│  │   (C++ node) │                   └──────────────────────┘  │
│  │              │                         │           ▲        │
│  │  Waypoint DB │   obstacle alert        │           │        │
│  │  entrance    │ ◄──────────────────┐    │ /cmd_vel  │ /odom  │
│  │  office_desk │                   │    ▼           │        │
│  │  storage_rm  │            ┌───────────────┐       │        │
│  │  charging_dk │            │   Obstacle    │       │        │
│  └──────────────┘            │   Monitor     │   ┌───────┐    │
│         │                    │  (C++ node)   │   │Gazebo │    │
│         │ /semantic_waypoints│               │   │  +    │    │
│         ▼                    │ /scan analysis│   │  TB3  │    │
│  ┌────────────┐              └───────┬───────┘   └───────┘    │
│  │   RViz2    │                      │ /scan                   │
│  │  Semantic  │ ◄────────────────────┘                        │
│  │  Overlay   │                                                 │
│  └────────────┘                                                 │
└────────────────────────────────────────────────────────────────┘
```

### Key Nodes

| Node | File | Purpose |
|------|------|---------|
| `semantic_navigator` | `src/semantic_navigator.cpp` | Maintains semantic waypoint DB, drives Nav2 action client, handles command/obstacle topics |
| `obstacle_monitor` | `src/obstacle_monitor.cpp` | Analyses `/scan` for proximity warnings and forward path blockage |

---

## Prerequisites — Windows + WSL2

### 1. Enable WSL2 + Ubuntu 22.04
```powershell
# In PowerShell (Administrator)
wsl --install -d Ubuntu-22.04
wsl --set-default-version 2
```

### 2. Install ROS2 Humble
```bash
# In WSL2 Ubuntu terminal
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 3. Install TurtleBot3 + Nav2
```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-turtlebot3-gazebo

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger"    >> ~/.bashrc
source ~/.bashrc
```

### 4. Install a map for turtlebot3_world
```bash
# The map YAML/PGM is usually at:
ls /opt/ros/humble/share/turtlebot3_navigation2/map/
# Copy or symlink into your package's maps/ directory:
cp /opt/ros/humble/share/turtlebot3_navigation2/map/map.pgm \
   ~/semantic_nav_ws/src/semantic_nav/maps/turtlebot3_world.pgm
cp /opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml \
   ~/semantic_nav_ws/src/semantic_nav/maps/turtlebot3_world.yaml
```

---

## Build & Run

```bash
cd ~/semantic_nav_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash

# Launch everything (Gazebo + Nav2 + RViz + our nodes)
ros2 launch semantic_nav semantic_nav_demo.launch.py

# Optional — loop the mission forever
ros2 launch semantic_nav semantic_nav_demo.launch.py loop_mission:=true
```

---

## Runtime Commands

Send commands to the robot from a **second terminal**:

```bash
source ~/semantic_nav_ws/install/setup.bash

# Jump directly to a named location
ros2 topic pub /semantic_nav/command std_msgs/msg/String \
  "data: 'go_to:charging_dock'" --once

# Pause mission
ros2 topic pub /semantic_nav/command std_msgs/msg/String \
  "data: 'pause'" --once

# Resume mission
ros2 topic pub /semantic_nav/command std_msgs/msg/String \
  "data: 'resume'" --once

# Monitor navigation status
ros2 topic echo /semantic_nav/status

# Monitor obstacle alerts
ros2 topic echo /obstacle_monitor/alert

# Live-tune obstacle warning threshold (no restart needed!)
ros2 param set /obstacle_monitor warning_distance_m 0.4
```

---

## What You'll See in RViz

| Visual Element | Meaning |
|----------------|---------|
| 🟢 Green spheres | Semantic waypoints on the map |
| White text labels | Waypoint names (entrance, office_desk …) |
| Green path line | Nav2 global plan |
| Red/yellow arrow | Closest obstacle direction + distance |
| Costmap overlay | Local & global costmaps updating in real time |

---

## Project Structure

```
semantic_nav/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── semantic_navigator.cpp    ← Main C++ nav node
│   └── obstacle_monitor.cpp     ← LaserScan analysis node
├── launch/
│   └── semantic_nav_demo.launch.py
├── config/
│   └── nav2_params.yaml          ← Full Nav2 parameter file
├── rviz/
│   └── semantic_nav.rviz         ← Pre-configured RViz layout
├── maps/                         ← Add turtlebot3_world.yaml/.pgm here
└── worlds/                       ← Custom Gazebo worlds (optional)
```

---

## Interview Talking Points 🎯

### "What is semantic navigation?"
> Traditional navigation uses raw coordinates. Semantic navigation adds **human-readable labels** to locations — "go to kitchen" instead of "go to (1.5, 2.3)". The robot maintains a dictionary mapping labels → map-frame poses. This makes it easier to build task-level planners on top.

### "How does obstacle avoidance work here?"
> **Two-layer approach:**
> 1. **Nav2 reactive layer** — DWB controller evaluates hundreds of velocity trajectories every 50 ms and picks the one that minimises a cost function including obstacle proximity.
> 2. **Semantic monitor layer** — My `obstacle_monitor` node independently analyses the LaserScan, detects critical proximity and forward path blockage, then signals the navigator to cancel and re-send the goal, forcing Nav2 to compute a fresh global path.

### "Why ROS2 action for navigation?"
> Actions are the right abstraction for **long-running tasks with feedback** — navigation can take seconds or minutes. The action client gets per-cycle feedback (distance remaining), can cancel mid-flight, and gets a typed result on completion or failure. Services would block; topics lack the result/cancel semantics.

### "How would you extend this to real hardware?"
> Only the launch file changes — swap `turtlebot3_gazebo` for the real robot's `turtlebot3_bringup`. The nav stack, obstacle monitor, and semantic navigator are hardware-agnostic. The AMCL node would need an initial pose estimate (or you'd use a better localiser like SLAM Toolbox in live mapping mode).

### "What's the costmap doing?"
> The **global costmap** inflates known obstacles on the static map, guiding the global planner (NavFn/A*) to stay away from walls. The **local costmap** is a rolling window that adds *dynamic* obstacles seen by the live laser scan, allowing the DWB controller to dodge people or moved furniture that weren't in the original map.

---

## Extending the Project

- **Add more semantic locations** — edit the `register_waypoint()` calls in `semantic_navigator.cpp`.
- **Custom Gazebo world** — add a `.world` file to `worlds/`, update the launch file.
- **3D obstacles** — replace LaserScan with a depth camera + PointCloud2 in `obstacle_monitor.cpp`.
- **NLP commands** — add a Python node that calls an LLM/intent parser and publishes to `/semantic_nav/command`.
- **Task planning** — layer a BehaviorTree.CPP or PDDL planner on top of the semantic waypoint graph.
