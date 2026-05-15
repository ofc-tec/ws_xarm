import py_trees
from py_trees.common import Access


class SelectDetectionPose(py_trees.behaviour.Behaviour):
    """Write the next detection joint pose to the blackboard."""

    def __init__(
        self,
        name: str,
        joint_values_options,
        joint_values_key: str = "detection_joint_values",
        attempt_key: str = "detection_pose_attempt",
    ):
        super().__init__(name)
        self.joint_values_options = [list(values) for values in joint_values_options]
        self.joint_values_key = joint_values_key
        self.attempt_key = attempt_key

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.joint_values_key, Access.WRITE)
        self.bb.register_key(self.attempt_key, Access.READ)
        self.bb.register_key(self.attempt_key, Access.WRITE)

    def update(self):
        if not self.joint_values_options:
            return py_trees.common.Status.FAILURE

        if self.bb.exists(self.attempt_key):
            attempt = int(self.bb.get(self.attempt_key) or 0)
        else:
            attempt = 0
        if attempt >= len(self.joint_values_options):
            attempt = 0

        setattr(self.bb, self.joint_values_key, self.joint_values_options[attempt])
        setattr(self.bb, self.attempt_key, attempt + 1)
        return py_trees.common.Status.SUCCESS
