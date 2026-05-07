# ws_xarm

ROS 2 Jazzy workspace for xArm simulation, MoveIt, pointcloud filtering, and Python behavior-tree arm leaves.

## Source Order

Use this order in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ws_xarm/install/setup.bash
```

`~/ws_moveit` must come before `~/ws_xarm` because `xarm_bt` uses the local MoveItPy build.

## Build

For normal BT/package work:

```bash
cd ~/ws_xarm
colcon build --packages-select xarm_bt
source install/setup.bash
```

If the local MoveItPy helper changed, rebuild MoveItPy first:

```bash
cd ~/ws_moveit
colcon build --packages-select moveit_py --cmake-clean-first --allow-overriding moveit_py
source install/setup.bash
```

## Launch

Start xArm6 Gazebo/MoveIt with the D435i model:

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
  add_realsense_d435i:=true \
  add_d435i_links:=true
```

Then run the BT pose example:

```bash
ros2 launch xarm_bt set_pose_example.launch.py
```

The BT launch also starts the depth-camera static transform and the pointcloud sanitizer used by octomap.

## MoveItPy Note

The current local MoveItPy build has a helper patch in:

```text
~/ws_moveit/src/moveit2/moveit_py/src/moveit/moveit_ros/moveit_cpp/planning_component.cpp
```

The official `PlanningComponent.plan()` Python binding fails in this setup, so `xarm_bt` calls local helpers:

- `plan_with_single_pipeline(...)` for plan-only behavior.
- `plan_and_execute_with_single_pipeline(...)` for `execute=True`.

If this workspace is recreated on another machine, reapply that MoveItPy patch or replace it with an upstream MoveItPy version where the official tutorial API works.
