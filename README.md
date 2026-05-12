# ws_xarm

ROS 2 Jazzy workspace for an xArm6 Gazebo/MoveIt setup with a behavior-tree
grasping scaffold. The current demo detects a YCB apple with the existing
Robotino vision services, transforms the detection into the xArm planning
frame, and moves `link_eef` to a conservative pre-grasp pose above the apple.

## What Is Here

- `xarm_pose_action`: action servers for MoveIt goals:
  - `/set_joints` for joint-space targets.
  - `/set_pose` for end-effector pose/position targets.
- `xarm_bt`: Python behavior-tree leaves and the apple grasping tree.
- `pc_tools`: pointcloud republisher/sanitizer used by the octomap path.
- `xarm_ros2`: upstream xArm ROS 2 packages used for description, Gazebo, MoveIt,
  controllers, and messages.

## Install

These notes assume Ubuntu 24.04 + ROS 2 Jazzy and this workspace at
`~/ws_xarm`.

1. Install ROS/MoveIt/Gazebo dependencies.

   ```bash
   sudo apt update
   sudo apt install ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros-gz
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip
   ```

2. Clone this workspace, including submodules if applicable.

   ```bash
   cd ~
   git clone git@github.com:ofc-tec/ws_xarm.git
   cd ws_xarm
   git submodule update --init --recursive
   ```

3. Source any companion workspaces before building.

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/robotino_ros2_ws/install/setup.bash
   ```

   The Robotino workspace provides the current `/yolo_detect` and `/tts/talk`
   services used by the BT launch.

4. Install ROS dependencies.

   ```bash
   cd ~/ws_xarm
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Install optional Python convenience requirements.

   ```bash
   python3 -m pip install -r requirements.txt
   ```

6. Build.

   ```bash
   cd ~/ws_xarm
   colcon build --packages-select pc_tools xarm_pose_action xarm_bt
   source install/setup.bash
   ```

For normal BT/action iteration, rebuilding the two edited packages is enough:

```bash
colcon build --packages-select xarm_pose_action xarm_bt
source install/setup.bash
```

## Source Order

Use this order in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/ws_xarm/install/setup.bash
```

If you are using a local MoveIt workspace, source it before `~/ws_xarm`.

## Run And Try

Terminal 1: launch xArm6 Gazebo/MoveIt with the D435i model.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_xarm/install/setup.bash

ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
  add_realsense_d435i:=true \
  add_d435i_links:=true
```

Terminal 2: launch the grasping scaffold, perception services, action servers,
BT runner, and optional py_trees viewer.

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/ws_xarm/install/setup.bash

ros2 launch xarm_bt grasping_tree.launch.py \
  use_introspection:=true \
  with_viewer:=true
```

The current tree does:

1. Move to the capture-image joint pose.
2. Say "Scanning table for apple".
3. Call `/yolo_detect`.
4. Select an `apple` detection.
5. Say "Apple found. Grasping."
6. Transform the selected pose to `link_base`.
7. Create a pre-grasp pose 15 cm above the apple.
8. Call `/set_pose` in position-only mode, with up to 3 retries.
9. Say "Success. Thanks for watching."

## Rerun Only The Tree

If Gazebo, MoveIt, YOLO, TTS, `/set_joints`, and `/set_pose` are already
running, rerun just the BT node:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/ws_xarm/install/setup.bash

ros2 run xarm_bt grasping_tree --ros-args \
  -p use_sim_time:=true \
  -p start_delay_sec:=0.0 \
  -p use_introspection:=true
```

Do not run two BT instances at the same time; both can send arm goals.

## Useful Checks

Confirm the action servers:

```bash
ros2 action list -t | grep -E 'set_joints|set_pose'
```

Expected:

```text
/set_joints [xarm_pose_action/action/SetJoints]
/set_pose [xarm_pose_action/action/SetPose]
```

Confirm Robotino services:

```bash
ros2 service list | grep -E 'yolo_detect|tts'
```

Inspect BT/RViz debug frames:

- `xarm_target_apple_*`: selected YOLO target after the current xArm-side
  correction.
- `bt_selected_apple_link_base`: selected apple transformed to `link_base`.
- `bt_apple_approach_pose`: final pre-grasp target sent to `/set_pose`.

## Current Tuning Notes

This is intentionally a scaffold for real robot debugging. The hard parts left
for students are useful robotics problems:

- improve apple pose estimation,
- tune the camera-to-arm frame correction,
- improve grasp approach orientation,
- add gripper control,
- add collision-aware approach/retreat behavior,
- replace the current position-only pre-grasp with a full grasp pose once IK is
  reliable.

The current `/set_pose` action has a `position_only` option because full pose
targets were too brittle for early grasp debugging. Position-only mode asks
MoveIt to place `link_eef` at the target point without forcing a specific
quaternion.
