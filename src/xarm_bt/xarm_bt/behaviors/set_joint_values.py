import py_trees
from rclpy.action import ActionClient
from py_trees.common import Access

from xarm_pose_action.action import SetJoints


class SetJointValues(py_trees.behaviour.Behaviour):
    """BT leaf that sends a joint-space goal to the xArm SetJoints action server."""

    def __init__(
        self,
        name: str,
        node=None,
        joint_values=None,
        joint_values_key: str = "arm_joint_values",
        group_name: str = "xarm6",
        execute: bool = False,
        action_name: str = "/set_joints",
        velocity_scaling: float = 0.9,
        acceleration_scaling: float = 0.9,
        planning_time: float = 5.0,
        planning_attempts: int = 5,
    ):
        super().__init__(name)
        self.node = node
        self.joint_values = joint_values
        self.joint_values_key = joint_values_key
        self.group_name = group_name
        self.execute = bool(execute)
        self.action_name = action_name
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.joint_values_key, Access.READ)

        self.client = None
        self.goal_future = None
        self.result_future = None
        self._goal_sent = False
        self._failed_to_start = False

    def setup(self, **kwargs):
        self.node = self.node or kwargs.get("node")
        if self.node is None:
            raise RuntimeError("SetJointValues requires a ROS node")
        self.client = ActionClient(self.node, SetJoints, self.action_name)

    def initialise(self):
        self.goal_future = None
        self.result_future = None
        self._goal_sent = False
        self._failed_to_start = False

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn(f"[SetJointValues] Action {self.action_name} not available")
            self._failed_to_start = True
            return

        joint_values = self.joint_values
        if joint_values is None:
            joint_values = getattr(self.bb, self.joint_values_key, None)

        if joint_values is None:
            self.node.get_logger().warn("[SetJointValues] No joint values available")
            self._failed_to_start = True
            return

        goal = SetJoints.Goal()
        goal.joint_values = [float(value) for value in joint_values]
        goal.planning_group = self.group_name
        goal.execute = self.execute
        goal.velocity_scaling = float(self.velocity_scaling)
        goal.acceleration_scaling = float(self.acceleration_scaling)
        goal.planning_time = float(self.planning_time)
        goal.planning_attempts = int(self.planning_attempts)

        self.node.get_logger().info(f"[SetJointValues] Sending joint goal to {self.action_name}")
        self.goal_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        self._goal_sent = True

    def update(self):
        if self._failed_to_start:
            return py_trees.common.Status.FAILURE

        if not self._goal_sent or self.goal_future is None:
            return py_trees.common.Status.RUNNING

        if self.result_future is None:
            if not self.goal_future.done():
                return py_trees.common.Status.RUNNING

            goal_handle = self.goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn("[SetJointValues] Joint goal rejected")
                return py_trees.common.Status.FAILURE

            self.node.get_logger().info("[SetJointValues] Joint goal accepted")
            self.result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result().result
        if result.success:
            self.node.get_logger().info(f"[SetJointValues] {result.message}")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().warn(f"[SetJointValues] {result.message}")
        return py_trees.common.Status.FAILURE

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f"[SetJointValues] {feedback.state}")
