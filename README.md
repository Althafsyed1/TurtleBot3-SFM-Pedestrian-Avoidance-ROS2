

# TurtleBot3 Socially Aware Navigation Using Social Force Model in ROS2

**Author:** Mohammad Althaf Syed | CWID: 20034265  
Autonomous Mobile Robotic Systems  
**Institution:** Stevens Institute of Technology  

---

## Overview

This project implements a custom **Social Force Model (SFM)** local planner for the TurtleBot3 Burger robot navigating a simulated indoor cafe environment with four moving pedestrians using ROS2 Jazzy. The SFM planner is based on Helbing and Molnar (1995) and replaces the standard DWB local controller as a standalone ROS2 Python node without modifying the Nav2 stack.

The proposed planner was compared against the standard Dynamic Window Approach (DWB) baseline across five navigation runs using four performance metrics.

**Key Result: 75.6% reduction in pedestrian encounters compared to DWB baseline.**

---

## Workflow

1. Build the cafe map using SLAM in a static environment and save it
2. Run DWB baseline navigation with active pedestrians and record bag files
3. Run custom SFM planner navigation with active pedestrians and record bag files
4. Compute performance metrics from recorded bag files

## Demo Video

https://github.com/Althafsyed1/TurtleBot3-SFM-Pedestrian-Avoidance-ROS2/blob/main/dyn_bas_rec2.mov

---

## Results Summary

| Run | Planner | Path Distance (m) | Travel Time (s) | Encounters |
|-----|---------|-------------------|-----------------|------------|
| DWB Run 2 | DWB Baseline | 6.972 | 55.9 | 8 |
| DWB Run 3 | DWB Baseline | 11.745 | 94.1 | 14 |
| DWB Run 4 | DWB Baseline | 9.287 | 73.2 | 15 |
| SFM Run 1 | Custom SFM | 8.702 | 47.7 | 3 |
| SFM Run 2 | Custom SFM | 6.987 | 33.2 | 3 |

| Planner | Avg Path Distance (m) | Avg Travel Time (s) | Avg Encounters |
|---------|-----------------------|---------------------|----------------|
| DWB Baseline | 9.335 | 74.4 | 12.3 |
| Custom SFM | 7.845 | 40.45 | 3.0 |

---

## Files in This Repository

| File | Description |
|------|-------------|
| `sfm_planner.py` | Custom SFM local planner node. Subscribes to `/amcl_pose`, `/scan`, and `/goal_pose`. Publishes velocity commands to `/cmd_vel` at 10 Hz |
| `compute_final_metrics.py` | Reads a recorded MCAP bag file from an SFM run and computes path distance, travel time, and human encounter count |
| `compute_dwb_run2.py` | Computes metrics from DWB baseline Run 2 bag file |
| `compute_dwb_run3.py` | Computes metrics from DWB baseline Run 3 bag file |
| `compute_dwb_run4.py` | Computes metrics from DWB baseline Run 4 bag file |
| `compute_dwb_dynamic.py` | Computes metrics from dynamic DWB runs |
| `sfm_planner_backup.py` | Backup of the SFM planner node |
| `nav2_burger_backup.yaml` | Working Nav2 configuration with ARM64 parameter adjustments |
| `maps/cafe.yaml` | Saved cafe map metadata |
| `maps/cafe.pgm` | Saved cafe occupancy grid image |

---

## Dependencies

ROS2 Jazzy and the following packages:

```
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge
ros-jazzy-ros-gz-image
ros-jazzy-slam-toolbox
ros-jazzy-nav2-bringup
ros-jazzy-turtlebot3-gazebo
ros-jazzy-turtlebot3-navigation2
ros-jazzy-rviz2
```

Install them with:

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2 ros-jazzy-rviz2
```

---

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2
git clone https://github.com/Althafsyed1/TurtleBot3-SFM-Pedestrian-Avoidance-ROS2
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Copy the SFM planner and metrics scripts into the workspace:

```bash
cp ~/ros2_ws/src/TurtleBot3-SFM-Pedestrian-Avoidance-ROS2/sfm_planner.py ~/ros2_ws/
cp ~/ros2_ws/src/TurtleBot3-SFM-Pedestrian-Avoidance-ROS2/compute_final_metrics.py ~/ros2_ws/
cp ~/ros2_ws/src/TurtleBot3-SFM-Pedestrian-Avoidance-ROS2/maps/* ~/ros2_ws/maps/
```

Add to your `.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Run

### 1) Mapping (static scene, no pedestrians)

```bash
ros2 launch cpe631_ros2 cafe.launch.py mapping:=true
```

Use teleop to drive and build the map:

```bash
ros2 launch cpe631_ros2 teleop.launch.py model:=burger
```

### 2) Save the Map

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/cafe
```

This creates:
- `maps/cafe.yaml`
- `maps/cafe.pgm`

### 3) DWB Baseline Navigation (with pedestrians)

```bash
ros2 launch cpe631_ros2 cafe.launch.py navigation:=true map_file:=$HOME/ros2_ws/maps/cafe.yaml enable_peds:=true
```

In RViz:
1. Click **2D Pose Estimate** to set the robot starting position
2. Wait for AMCL particles to converge
3. Click **Nav2 Goal** to send a navigation goal

Record the run:

```bash
ros2 bag record -o ~/ros2_ws/dwb_run /odom /scan /cmd_vel
```

### 4) SFM Planner Navigation (with pedestrians)

Terminal 1:
```bash
ros2 launch cpe631_ros2 cafe.launch.py navigation:=true map_file:=$HOME/ros2_ws/maps/cafe.yaml enable_peds:=true
```

Terminal 2 (start SFM planner):
```bash
python3 ~/ros2_ws/sfm_planner.py
```

You should see: `SFM Planner node started`

In RViz:
1. Click **2D Pose Estimate** to set the robot starting position
2. Wait for AMCL particles to converge
3. Click **Nav2 Goal** to send a navigation goal
4. The robot will navigate while performing directional pedestrian avoidance

Record the run:

```bash
ros2 bag record -o ~/ros2_ws/sfm_run /odom /scan /cmd_vel
```

### 5) Compute Performance Metrics

Open `compute_final_metrics.py` and update the bag file path at the top to match your recorded file:

```python
uri='/home/YOUR_USERNAME/ros2_ws/sfm_run/sfm_run_0.mcap'
```

Then run:

```bash
python3 ~/ros2_ws/compute_final_metrics.py
```

Output:
- Path distance in meters
- Travel time in seconds  
- Human encounter count
- Social behavior description

---

## Launch Arguments

`cafe.launch.py` supports:

| Argument | Values | Description |
|----------|--------|-------------|
| `mapping` | true/false | Enable SLAM mapping mode |
| `navigation` | true/false | Enable Nav2 navigation |
| `enable_peds` | true/false | Enable moving pedestrians |
| `map_file` | path | Map YAML file for navigation |
| `model` | burger/waffle | TurtleBot3 model |

---

## SFM Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `k_goal` | 1.5 | Attractive force gain toward goal |
| `k_rep` | 8.0 | Repulsive force gain away from obstacles |
| `sigma` | 0.8 | Repulsion decay distance in meters |
| `obstacle_range` | 2.5 m | Maximum distance for obstacle influence |
| `danger_zone` | 0.5 m | Distance threshold for directional avoidance |
| `goal_threshold` | 0.3 m | Distance to consider goal reached |

---

## Topics Used

| Topic | Type | Description |
|-------|------|-------------|
| `/amcl_pose` | Subscribe | Robot pose from AMCL localization |
| `/scan` | Subscribe | 360 degree LiDAR scan data |
| `/goal_pose` | Subscribe | Navigation goal from RViz |
| `/cmd_vel` | Publish | Velocity commands to robot |
| `/odom` | Recorded | Odometry for path distance computation |

---

## ARM64 Note

This project was developed on Apple Silicon Mac M4 running Ubuntu 24.04.3 ARM64 via UTM virtualization. A clock synchronization issue between Gazebo and the system clock causes the Nav2 collision monitor to repeatedly halt the robot during DWB runs. The following parameters in `nav2_burger.yaml` reduce this issue:

```yaml
transform_tolerance: 2.0
source_timeout: 60.0
do_beamskip: true
max_particles: 5000
min_particles: 1000
```

A working backup of this configuration is included as `nav2_burger_backup.yaml`.

---

## Nav2 Configuration

**Default navigation** uses `cafe.launch.py` with `param/nav2_burger.yaml`:
- Global planner: NavfnPlanner (A* based)
- Local controller: DWB (Dynamic Window Approach)

**Custom SFM navigation** uses the same launch file but replaces the DWB local controller with the standalone `sfm_planner.py` node which publishes directly to `/cmd_vel`.

---

## Reference

D. Helbing and P. Molnar, "Social force model for pedestrian dynamics," *Physical Review E*, vol. 51, no. 5, pp. 4282-4286, 1995.

---

## Course Repository

The cafe simulation environment is provided by:  
https://github.com/SIT-Robotics-and-Automation-Laboratory/CPE631-Navigation-ROS2
