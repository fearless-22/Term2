# Project Gap Analysis and Completion Route

This document maps the current workspace to the courseware requirements in
`课件PPT`, especially lessons 8, 9, 11 and 12.

## 1. What the Project Already Has

### Gazebo and RViz basics, lessons 1-4

- `custom_bot.urdf` defines a differential-drive robot with lidar, camera,
  wheel joints and Gazebo plugins.
- `track.world` and the root `README.md` describe the early Gazebo + RViz2
  workflow.
- `RobotStateMonitor.py` subscribes to `/odom` and `/battery_state`, reads TF,
  and prints a JSON-style robot state.

### SLAM mapping, lessons 5-7

- `ros2_slam_ws/src/custom_slam/launch/slam_launch.py` launches
  `slam_toolbox` for mapping.
- `ros2_slam_ws/src/custom_slam/config/slam_params.yaml` provides the basic
  SLAM parameters.
- `output/` and `ros2_slam_ws/src/custom_slam/maps/` already contain a saved
  `secondmap.yaml` and `secondmap.pgm`.

### Embodied-SimLite Web simulation prototype

- `embodied-sim-lite/ProductV1.0.py` provides the FastAPI + WebSocket +
  Three.js digital twin.
- The Web scene already includes lidar ray casting, a 40 x 40 test field,
  dynamic obstacles, collision handling and telemetry.
- `embodied-sim-lite/sim_ros2_bridgeV1.0.py` bridges the Web simulation to ROS2:
  it subscribes `/cmd_vel` and `/cmd_vel_nav`, publishes `/odom` and `/scan`,
  and broadcasts `odom -> base_footprint -> base_link -> laser_frame`.
  It also publishes `/scan_nav` for dynamic obstacle avoidance and red RViz
  marker rays on `/scan_rays` and `/scan_nav_rays`.

## 2. What Was Missing Before This Completion Step

### Navigation2 integration, lessons 8-9

- `nav2_params.yaml` existed, but there was no source launch file that started
  Nav2 from the `custom_slam` package.
- The Web simulation does not publish `/clock`, while the old Nav2 parameters
  used `use_sim_time: True`. That can make Nav2/SLAM time handling inconsistent
  in the Web route.
- No repeatable command existed for launching SLAM + Nav2 + project nodes as one
  system.

### Integrated autonomous exploration, lesson 11

The courseware asks for this module chain:

`/scan -> SLAM Node -> /map -> Exploration Node -> /goal_pose -> Nav2 -> /cmd_vel`

Before this step, the following required source modules were absent:

- `state_manager.py`
- `exploration_node.py`
- `map_evaluator.py`

The required FSM states and recovery behavior were also absent:

- `IDLE`
- `EXPLORATION`
- `NAVIGATION`
- `MAP_EVALUATION`
- `ERROR`
- recovery by retrying and clearing costmaps

### Final delivery and demo, lesson 12

- No 3-minute unedited demo plan existed.
- No clear final packaging topology existed for report, video and source code.
- No terminal-visible FSM log stream existed for the required recovery segment.

## 3. What Has Been Added

### Web-route launch files

- `launch/slam_web_launch.py`
  - Starts `slam_toolbox` with Web-friendly system-time parameters.
- `launch/nav2_web_launch.py`
  - Starts Nav2 from `nav2_bringup`.
  - Starts `map_evaluator.py`, `exploration_node.py`, and `state_manager.py`.
- `launch/system_launch.py`
  - Starts Web SLAM first, then starts Nav2 and project nodes after a short
    delay.

### Web-route configuration

- `config/slam_web_params.yaml`
  - Uses `use_sim_time: false`.
  - Tunes `slam_toolbox` for `/scan`, `/odom`, `base_link` and a 0.05 m map.
- `config/nav2_web_params.yaml`
  - Based on the existing Nav2 config.
  - Changes all `use_sim_time` values to false for the Web bridge.
  - Uses `/scan_nav` for the local costmap so moving obstacles are temporary
    avoidance targets instead of permanent SLAM map obstacles.
- `config/exploration_params.yaml`
  - Controls frontier size, goal interval, information gain radius and scoring.
- `config/system_params.yaml`
  - Controls the FSM heartbeat, goal timeout, recovery cooldown and map pass
    thresholds.

### Courseware-required Python nodes

- `scripts/exploration_node.py`
  - Subscribes `/map` and `/odom`.
  - Detects frontier cells where Free(0) borders Unknown(-1).
  - Clusters and filters isolated frontiers.
  - Selects the next goal with `U = alpha * I - beta * C`.
  - Publishes `/exploration/goal_pose` and `/exploration/status`.
- `scripts/map_evaluator.py`
  - Subscribes `/map`.
  - Publishes `/map_evaluation` as JSON with known ratio, known area, frontier
    count and pass/fail status.
- `scripts/state_manager.py`
  - Implements the top-level FSM.
  - Sends frontier goals to Nav2 using `NavigateToPose`.
  - Clears local/global costmaps with `ClearEntireCostmap` during recovery.
  - Publishes `/system_state`.
  - Accepts `/system_command` for repeatable demo control:
    `START`, `STOP`, `INJECT_ERROR`, `FINISH`.

## 4. Run Sequence for the Final Web Demo

Terminal 1, Web simulation:

```bash
cd /home/lxj/design_project/Term2/embodied-sim-lite
python3 ProductV1.0.py
```

Browser:

```text
http://localhost:8000
```

Terminal 2, ROS2 bridge:

```bash
cd /home/lxj/design_project/Term2/embodied-sim-lite
source /opt/ros/humble/setup.bash
python3 sim_ros2_bridgeV1.0.py
```

Terminal 3, integrated SLAM + Nav2 + exploration system:

```bash
cd /home/lxj/design_project/Term2/ros2_slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch custom_slam system_launch.py
```

Terminal 4, RViz2:

```bash
source /opt/ros/humble/setup.bash
rviz2
```

RViz2 display checklist:

- Global Options: `Fixed Frame = map`
- Add `Map`: `/map`
- Add `LaserScan`: `/scan`, Reliability = `Best Effort`, Durability = `Volatile`
- Add `LaserScan`: `/scan_nav`, Reliability = `Best Effort`, Durability = `Volatile`
- Add `Marker`: `/scan_nav_rays` for red full-ray visualization
- Add `Path`: `/plan`
- Add `Path`: `/local_plan`
- Add `Map`: `/global_costmap/costmap`
- Add `Map`: `/local_costmap/costmap`
- Add `TF`

## 5. Useful Demo Commands

Show node and topic topology:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /system_state
ros2 topic echo /map_evaluation
```

Inject a recoverable FSM fault for the lesson 12 recovery segment:

```bash
ros2 topic pub --once /system_command std_msgs/msg/String "{data: INJECT_ERROR}"
```

Stop the FSM safely:

```bash
ros2 topic pub --once /system_command std_msgs/msg/String "{data: STOP}"
```

Restart exploration:

```bash
ros2 topic pub --once /system_command std_msgs/msg/String "{data: START}"
```

Save the final map:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/embodied_simlite_final_map
```

## 6. Final Packaging Topology

Lesson 12 expects a normalized package. Use this structure when exporting:

```text
Team_X_Final_Project.zip
├── 1_Defense_PPT/
│   └── defense_slides.pptx
├── 2_Demo_Video/
│   └── baseline_test_dual_screen.mp4
├── 3_Final_Report/
│   └── final_report.docx
└── 4_Source_Code/
    ├── embodied-sim-lite/
    └── ros2_slam_ws/
        └── src/custom_slam/
```

Before zipping, rebuild once:

```bash
cd /home/lxj/design_project/Term2/ros2_slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select custom_slam
```
