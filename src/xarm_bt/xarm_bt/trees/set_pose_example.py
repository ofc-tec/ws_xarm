import py_trees
from geometry_msgs.msg import PoseStamped

from xarm_bt.behaviors.set_pose import SetPose


def make_example_pose() -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "link_base"
    pose.pose.position.x = 0.30
    pose.pose.position.y = 0.00
    pose.pose.position.z = 0.25
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    return pose


def create_behavior_tree():
    root = py_trees.composites.Sequence(name="XArmSetPoseExample", memory=True)
    root.add_child(
        SetPose(
            name="PlanExamplePose",
            pose=make_example_pose(),
            group_name="xarm6",
            pose_link="link_eef",
            execute=True,
        )
    )
    return py_trees.trees.BehaviourTree(root)
