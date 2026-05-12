import py_trees
import rclpy
from geometry_msgs.msg import TransformStamped
from py_trees.common import Access
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


class TransformPose(py_trees.behaviour.Behaviour):
    """Transform a blackboard PoseStamped into a target frame."""

    def __init__(
        self,
        name: str,
        node,
        input_key: str = "selected_yolo_pose",
        output_key: str = "selected_yolo_pose_link_base",
        target_frame: str = "link_base",
        debug_child_frame_id: str = "",
        timeout_sec: float = 0.5,
    ):
        super().__init__(name)
        self.node = node
        self.input_key = input_key
        self.output_key = output_key
        self.target_frame = target_frame
        self.debug_child_frame_id = debug_child_frame_id
        self.timeout_sec = float(timeout_sec)

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.input_key, Access.READ)
        self.bb.register_key(self.output_key, Access.WRITE)

        self.tf_buffer = None
        self.tf_listener = None
        self.broadcaster = None

    def setup(self, **kwargs):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.broadcaster = StaticTransformBroadcaster(self.node)

    def update(self):
        source_pose = getattr(self.bb, self.input_key, None)
        if source_pose is None:
            self.node.get_logger().warn(f"[TransformPose] No pose available at blackboard key '{self.input_key}'")
            return py_trees.common.Status.FAILURE

        source_frame = source_pose.header.frame_id
        if not source_frame:
            self.node.get_logger().warn("[TransformPose] Source pose has empty header.frame_id")
            return py_trees.common.Status.FAILURE

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.timeout_sec),
            )
            target_pose = do_transform_pose_stamped(source_pose, transform)
        except TransformException as exc:
            self.node.get_logger().warn(
                f"[TransformPose] Could not transform {source_frame} -> {self.target_frame}: {exc}"
            )
            return py_trees.common.Status.FAILURE

        setattr(self.bb, self.output_key, target_pose)
        self._publish_debug_tf(target_pose)
        p = target_pose.pose.position
        self.node.get_logger().info(
            f"[TransformPose] {self.output_key} = {target_pose.header.frame_id} "
            f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        return py_trees.common.Status.SUCCESS

    def _publish_debug_tf(self, pose):
        if self.broadcaster is None or not self.debug_child_frame_id:
            return

        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = self.debug_child_frame_id
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.broadcaster.sendTransform(transform)
