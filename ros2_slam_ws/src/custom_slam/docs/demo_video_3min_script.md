# 3-Minute Continuous Demo Video Script

This script follows lesson 12: use a continuous unedited video and show both
the physical disturbance view and the logical self-recovery view.

## Screen Layout

Recommended dual-screen or split-screen layout:

- Left: browser at `http://localhost:8000`, showing the Embodied-SimLite Web
  simulation, lidar rays, dynamic obstacles and robot motion.
- Right top: RViz2, showing `/map`, `/scan`, `/plan`, `/local_plan`, costmaps
  `/scan_nav`, `/scan_nav_rays` and TF.
- Right bottom: terminal showing `ros2 launch custom_slam system_launch.py`
  logs and a second terminal ready for `ros2 topic pub`.

## Before Recording

Start these processes before clicking record:

```bash
# Terminal 1
cd /home/lxj/design_project/Term2/embodied-sim-lite
python3 ProductV1.0.py
```

```bash
# Terminal 2
cd /home/lxj/design_project/Term2/embodied-sim-lite
source /opt/ros/humble/setup.bash
python3 sim_ros2_bridgeV1.0.py
```

```bash
# Terminal 3
cd /home/lxj/design_project/Term2/ros2_slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch custom_slam system_launch.py
```

Keep this command ready but do not run it until the 2-minute mark:

```bash
ros2 topic pub --once /system_command std_msgs/msg/String "{data: INJECT_ERROR}"
```

## Recording Timeline

### 0:00-0:20, System proof

Show:

- Browser simulation is online.
- ROS bridge terminal says it connected to the Web engine.
- `system_launch.py` terminal shows project nodes starting.

Say:

> This is the Embodied-SimLite Web digital twin connected to ROS2 Humble. The
> bridge publishes `/odom`, `/scan` and TF, while Nav2 sends `/cmd_vel` back to
> the simulated robot.

### 0:20-0:55, ROS2 data flow

Show RViz2:

- `Fixed Frame = map`
- `/scan` laser points
- `/scan_nav` laser points or `/scan_nav_rays` red full-ray marker
- `/map` growing from SLAM
- TF tree aligned with the robot

Say:

> The data flow follows the course design: `/scan` enters SLAM, SLAM publishes
> `/map`, the exploration node evaluates frontiers, and the state manager sends
> NavigateToPose goals to Nav2.

### 0:55-1:35, Autonomous exploration and navigation

Show:

- Terminal logs with `[FSM State]: IDLE -> EXPLORATION`.
- Then `[FSM State]: EXPLORATION -> NAVIGATION`.
- RViz global/local path and robot movement.
- Browser dynamic obstacles and lidar response.

Say:

> The exploration node searches the boundary between Free(0) and Unknown(-1),
> clusters candidate frontiers, and scores them using information gain minus
> travel cost. The state manager dispatches the selected goal through Nav2.

### 1:35-2:10, Dynamic benchmark behavior

Show:

- Browser dynamic obstacles crossing the field.
- RViz local costmap or local path replanning.
- `/map` remains clean because SLAM uses static-only `/scan`, while Nav2
  avoids moving obstacles with `/scan_nav`.
- Terminal `/map_evaluation` or `/system_state` stream if available.

Say:

> This segment demonstrates the benchmark requirement: the physical view shows
> dynamic obstacle interference, while RViz shows the planning stack reacting
> through costmaps and local path updates.

### 2:10-2:35, Fault injection and recovery

Run:

```bash
ros2 topic pub --once /system_command std_msgs/msg/String "{data: INJECT_ERROR}"
```

Show:

- Terminal logs:
  - `[FSM State]: NAVIGATION/EXPLORATION -> ERROR`
  - `[FSM State]: ERROR -> RECOVERY`
  - `[FSM Recovery] attempt=...`
  - `[FSM State]: RECOVERY -> EXPLORATION`

Say:

> I inject a recoverable fault to make the self-healing path visible. The FSM
> enters ERROR, clears costmaps during RECOVERY, stops the robot briefly, and
> returns to EXPLORATION.

### 2:35-3:00, Result and closure

Show:

- Robot continuing exploration or holding safely.
- `/system_state` showing current state.
- Optional final map in RViz.

Say:

> The final system covers the required integrated modules: Web simulation,
> ROS2 bridge, SLAM, frontier exploration, Nav2 navigation, map evaluation and
> FSM recovery. This recording is continuous and unedited as required by
> lesson 12.

## Video Checklist

- Browser simulation visible.
- RViz2 visible with map, scan, path and costmaps.
- Terminal logs visible with FSM transitions.
- Dynamic obstacle or disturbance visible.
- Recovery log visible.
- Total length about 3 minutes.
- No cuts, no speed-up, no post-editing.
