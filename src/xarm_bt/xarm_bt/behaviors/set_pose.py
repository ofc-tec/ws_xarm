from geometry_msgs.msg import PoseStamped
import py_trees
from py_trees.common import Access
from rclpy.action import ActionClient

from xarm_pose_action.action import SetPose as SetPoseAction


class SetPose(py_trees.behaviour.Behaviour):
    """BT leaf that sends an end-effector pose goal to the xArm SetPose action server."""

    def __init__(
        self,
        name: str,
        node=None,
        pose: PoseStamped = None,
        pose_key: str = "arm_target_pose",
        group_name: str = "xarm6",
        pose_link: str = "",
        execute: bool = False,
        cartesian: bool = False,
        position_only: bool = False,
        action_name: str = "/set_pose",
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.1,
        position_tolerance: float = 0.01,
        orientation_tolerance: float = 0.05,
        planning_time: float = 5.0,
        planning_attempts: int = 5,
        max_retries: int = 0,
    ):
        super().__init__(name)
        self.node = node
        self.pose = pose
        self.pose_key = pose_key
        self.group_name = group_name
        self.pose_link = pose_link
        self.execute = bool(execute)
        self.cartesian = bool(cartesian)
        self.position_only = bool(position_only)
        self.action_name = action_name
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.max_retries = max(0, int(max_retries))

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.pose_key, Access.READ)

        self.client = None
        self.goal_future = None
        self.result_future = None
        self._goal_sent = False
        self._failed_to_start = False
        self._retries_used = 0

    def setup(self, **kwargs):
        self.node = self.node or kwargs.get("node")
        if self.node is None:
            raise RuntimeError("SetPose requires a ROS node")
        self.client = ActionClient(self.node, SetPoseAction, self.action_name)

    def initialise(self):
        self.goal_future = None
        self.result_future = None
        self._goal_sent = False
        self._failed_to_start = False
        self._retries_used = 0

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn(f"[SetPose] Action {self.action_name} not available")
            self._failed_to_start = True
            return

        target_pose = self.pose
        if target_pose is None:
            target_pose = getattr(self.bb, self.pose_key, None)

        if target_pose is None:
            self.node.get_logger().warn("[SetPose] No target pose available")
            self._failed_to_start = True
            return

        self._send_goal(target_pose)

    def _send_goal(self, target_pose):
        goal = SetPoseAction.Goal()
        goal.target_pose = target_pose
        goal.planning_group = self.group_name
        goal.end_effector_link = self.pose_link
        goal.execute = self.execute
        goal.cartesian = self.cartesian
        goal.position_only = self.position_only
        goal.velocity_scaling = float(self.velocity_scaling)
        goal.acceleration_scaling = float(self.acceleration_scaling)
        goal.position_tolerance = float(self.position_tolerance)
        goal.orientation_tolerance = float(self.orientation_tolerance)
        goal.planning_time = float(self.planning_time)
        goal.planning_attempts = int(self.planning_attempts)

        p = target_pose.pose.position
        self.node.get_logger().info(
            f"[SetPose] Sending pose goal to {self.action_name} "
            f"(attempt {self._retries_used + 1}/{self.max_retries + 1}): "
            f"{target_pose.header.frame_id} ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        self.goal_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        self.result_future = None
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
                self.node.get_logger().warn("[SetPose] Pose goal rejected")
                return self._retry_or_fail("Pose goal rejected")

            self.node.get_logger().info("[SetPose] Pose goal accepted")
            self.result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result().result
        if result.success:
            self.node.get_logger().info(f"[SetPose] {result.message}")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().warn(f"[SetPose] {result.message}")
        return self._retry_or_fail(result.message)

    def _retry_or_fail(self, reason: str):
        if self._retries_used >= self.max_retries:
            return py_trees.common.Status.FAILURE

        self._retries_used += 1
        target_pose = self.pose
        if target_pose is None:
            target_pose = getattr(self.bb, self.pose_key, None)

        if target_pose is None:
            self.node.get_logger().warn("[SetPose] No target pose available for retry")
            return py_trees.common.Status.FAILURE

        self.node.get_logger().warn(
            f"[SetPose] Retrying after failure ({reason}); "
            f"retry {self._retries_used}/{self.max_retries}"
        )
        self._send_goal(target_pose)
        return py_trees.common.Status.RUNNING

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f"[SetPose] {feedback.state}")
