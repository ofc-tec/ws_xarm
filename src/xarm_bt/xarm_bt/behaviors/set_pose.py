import py_trees
from py_trees.common import Access

from geometry_msgs.msg import PoseStamped

from xarm_bt.arm_commander import XArmMoveItPyCommander


class SetPose(py_trees.behaviour.Behaviour):
    """BT leaf that plans or executes an xArm end-effector pose with MoveItPy."""

    def __init__(
        self,
        name: str,
        pose: PoseStamped = None,
        pose_key: str = "arm_target_pose",
        group_name: str = "xarm6",
        pose_link: str = "",
        execute: bool = False,
    ):
        super().__init__(name)
        self.pose = pose
        self.pose_key = pose_key
        self.group_name = group_name
        self.pose_link = pose_link
        self.execute = bool(execute)

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.pose_key, Access.READ)

        self.arm = None
        self._done = False
        self._success = False

    def setup(self, **kwargs):
        node = kwargs.get("node", None)
        logger = node.get_logger() if node is not None else None
        self.arm = XArmMoveItPyCommander(
            group_name=self.group_name,
            default_pose_link=self.pose_link,
            logger=logger,
        )

    def initialise(self):
        self._done = False
        self._success = False

    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS if self._success else py_trees.common.Status.FAILURE

        target_pose = self.pose
        if target_pose is None:
            target_pose = getattr(self.bb, self.pose_key, None)

        result = self.arm.set_pose(target_pose, pose_link=self.pose_link, execute=self.execute)
        self.logger.info(f"[SetPose] {result.message}")

        self._done = True
        self._success = result.success
        return py_trees.common.Status.SUCCESS if result.success else py_trees.common.Status.FAILURE
