import math

import py_trees

from xarm_bt.behaviors.say_text import SayText
from xarm_bt.behaviors.select_yolo_target import SelectYoloTarget
from xarm_bt.behaviors.set_joint_values import SetJointValues
from xarm_bt.behaviors.yolo_detect import YoloDetect


CAPTURE_IMAGE_JOINTS = [   # obtained from moveit gui
    math.radians(178.0),
    math.radians(-76.0),
    math.radians(-36.0),
    math.radians(0.0),
    math.radians(112.0),
    math.radians(0.0),
]


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
    root.add_child(YoloDetect(name="DetectObjects", node=node))

    apple_found = py_trees.composites.Sequence(name="AppleFound", memory=True)
    apple_found.add_child(
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

    root.add_child(
        py_trees.composites.Selector(
            name="HandleAppleDetection",
            memory=False,
            children=[apple_found, no_apple],
        )
    )
    return py_trees.trees.BehaviourTree(root)
