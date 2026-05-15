import math

import py_trees
from tf_transformations import quaternion_from_euler

from xarm_bt.behaviors.say_text import SayText
from xarm_bt.behaviors.offset_pose import OffsetPose
from xarm_bt.behaviors.select_yolo_target import SelectYoloTarget
from xarm_bt.behaviors.set_joint_values import SetJointValues
from xarm_bt.behaviors.set_pose import SetPose
from xarm_bt.behaviors.transform_pose import TransformPose
from xarm_bt.behaviors.yolo_detect import YoloDetect


CAPTURE_IMAGE_JOINTS = [   # obtained from moveit gui
    math.radians(178.0),
    math.radians(-76.0),
    math.radians(-36.0),
    math.radians(0.0),
    math.radians(112.0),
    math.radians(0.0),
]

ABOVE_GRASP_RPY_DEG = [-180.0, 0.0, 180.0]
ABOVE_GRASP_ORIENTATION = list(quaternion_from_euler(
    math.radians(ABOVE_GRASP_RPY_DEG[0]),
    math.radians(ABOVE_GRASP_RPY_DEG[1]),
    math.radians(ABOVE_GRASP_RPY_DEG[2]),
))


def create_behavior_tree(node):
    root = py_trees.composites.Sequence(name="GraspingTree", memory=True)
    root.add_child(
        SetJointValues(
            name="MoveToCaptureImagePose",
            node=node,
            joint_values=CAPTURE_IMAGE_JOINTS,
            group_name="xarm6",
            execute=True,
        )
    )
    root.add_child(
        SayText(
            name="SayScanningTable",
            node=node,
            text="Scanning table for apple",
        )
    )
    root.add_child(YoloDetect(name="DetectObjects", node=node))

    select_apple = py_trees.composites.Sequence(name="FindApple", memory=True)
    select_apple.add_child(
        SelectYoloTarget(
            name="SelectAppleTarget",
            node=node,
            target_class="apple",
        )
    )

    no_apple = py_trees.composites.Sequence(name="NoAppleFound", memory=True)
    no_apple.add_child(
        SayText(
            name="SayNoApple",
            node=node,
            text="I did not find an apple",
        )
    )
    no_apple.add_child(py_trees.behaviours.Failure(name="NoAppleStopsTree"))

    apple_found = py_trees.composites.Sequence(name="AppleFound", memory=True)
    apple_found.add_child(
        py_trees.composites.Selector(
            name="HandleAppleSelection",
            memory=False,
            children=[select_apple, no_apple],
        )
    )
    apple_found.add_child(
        SayText(
            name="SayAppleFound",
            node=node,
            text="Apple found. Grasping.",
        )
    )
    apple_found.add_child(
        TransformPose(
            name="TransformAppleTargetToBase",
            node=node,
            input_key="selected_yolo_pose",
            output_key="selected_yolo_pose_link_base",
            target_frame="link_base",
            debug_child_frame_id="bt_selected_apple_link_base",
        )
    )
    apple_found.add_child(
        OffsetPose(
            name="MakeAppleApproachPose",
            node=node,
            input_key="selected_yolo_pose_link_base",
            output_key="apple_approach_pose",
            offset_z=0.35,
            orientation_xyzw=ABOVE_GRASP_ORIENTATION,
            debug_child_frame_id="bt_apple_approach_pose",
        )
    )
    apple_found.add_child(
        SetPose(
            name="MoveAboveApple",
            node=node,
            pose_key="apple_approach_pose",
            group_name="xarm6",
            pose_link="link_eef",
            execute=True,
            position_only=False,
            velocity_scaling=0.9,
            acceleration_scaling=0.9,
            position_tolerance=0.03,
            orientation_tolerance=0.35,
            planning_time=25.0,
            planning_attempts=10,
            max_retries=3,
        )
    )
    apple_found.add_child(
        SayText(
            name="SayAppleReached",
            node=node,
            text="Success. Thanks for watching.",
        )
    )
    root.add_child(apple_found)
    return py_trees.trees.BehaviourTree(root)
