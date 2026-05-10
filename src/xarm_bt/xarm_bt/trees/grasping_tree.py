import math

import py_trees

from xarm_bt.behaviors.set_joint_values import SetJointValues


CAPTURE_IMAGE_JOINTS = [
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
            joint_values=CAPTURE_IMAGE_JOINTS,
            group_name="xarm6",
            execute=True,
        )
    )
    return py_trees.trees.BehaviourTree(root)
