# xArm Apple Grasping Scaffold

This workspace contains a ROS 2 Jazzy xArm6 simulation scaffold for behavior-tree
grasping experiments. The current demo uses Robotino vision/TTS services to find
an apple, publishes debug TFs for the detected target, and moves the xArm end
effector to a conservative pre-grasp pose above it.

The point of this repo is not a finished gripper pipeline yet. It is a working
student-friendly scaffold for real robot debugging: perception, TFs, MoveIt
planning, action servers, BT introspection, and visible RViz targets are all in
place.

## Main Pieces

### `xarm_pose_action`

Action servers wrapping MoveIt:

- `/set_joints` (`SetJoints.action`): joint-space motion goals.
- `/set_pose` (`SetPose.action`): end-effector pose/position goals.

`/set_pose` supports `position_only`, which asks MoveIt to reach an XYZ target
without forcing a fragile orientation constraint. This is what the current apple
approach uses.

### `xarm_bt`

Behavior-tree package with leaves for:

- `YoloDetect`: calls `/yolo_detect`.
- `SelectYoloTarget`: selects an apple detection and publishes a debug TF.
- `TransformPose`: transforms the selected pose into `link_base`.
- `OffsetPose`: creates the 15 cm-above-apple approach pose.
- `SetJointValues`: calls `/set_joints`.
- `SetPose`: calls `/set_pose`, with retries.
- `SayText`: calls `/tts/talk`.

Current tree flow:

```text
MoveToCaptureImagePose
SayScanningTable
DetectObjects
SelectAppleTarget
SayAppleFound
TransformAppleTargetToBase
MakeAppleApproachPose
MoveAboveApple
SayAppleReached
```

## Required External Services

The grasping launch expects the Robotino auxiliary stack to be sourced/available:

- `/yolo_detect`
- `/tts/talk`
- supporting Robotino vision services/nodes

In this setup those come from `~/robotino_ros2_ws`.

## Install And Build

Use ROS 2 Jazzy. Basic setup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash

cd ~/ws_xarm
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
colcon build --packages-select pc_tools xarm_pose_action xarm_bt
source install/setup.bash
```

For normal iteration:

```bash
cd ~/ws_xarm
colcon build --packages-select xarm_pose_action xarm_bt
source install/setup.bash
```

## Launch

First launch the xArm6 Gazebo/MoveIt simulation with the D435i camera:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_xarm/install/setup.bash

ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
  add_realsense_d435i:=true \
  add_d435i_links:=true
```

Then launch the grasping scaffold:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/ws_xarm/install/setup.bash

ros2 launch xarm_bt grasping_tree.launch.py \
  use_introspection:=true \
  with_viewer:=true
```

This starts the perception helpers, TTS node, `/set_joints`, `/set_pose`, the BT
runner, and optionally the py_trees viewer.

## Rerun Only The Tree

If Gazebo, MoveIt, perception, TTS, and action servers are already running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotino_ros2_ws/install/setup.bash
source ~/ws_xarm/install/setup.bash

ros2 run xarm_bt grasping_tree --ros-args \
  -p use_sim_time:=true \
  -p start_delay_sec:=0.0 \
  -p use_introspection:=true
```

Do not run two tree instances at once; both can send arm goals.

## Debug Frames In RViz

The BT publishes static TF debug frames:

- `xarm_target_apple_*`: selected apple target after current correction.
- `bt_selected_apple_link_base`: selected apple transformed to `link_base`.
- `bt_apple_approach_pose`: final 15 cm-above target sent to `/set_pose`.

These are the first things to inspect when tuning detection and grasp pose
quality.

## Useful Checks

```bash
ros2 action list -t | grep -E 'set_joints|set_pose'
ros2 service list | grep -E 'yolo_detect|tts'
```

Expected actions:

```text
/set_joints [xarm_pose_action/action/SetJoints]
/set_pose [xarm_pose_action/action/SetPose]
```

## Next Student Tasks

- Improve apple pose estimation.
- Tune the camera-to-arm frame correction.
- Replace the conservative pre-grasp with a better grasp pose.
- Add gripper control.
- Add approach/retreat leaves.
- Use BT introspection to compare planned behavior against RViz TFs.
