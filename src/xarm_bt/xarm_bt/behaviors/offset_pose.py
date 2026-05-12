from copy import deepcopy

from geometry_msgs.msg import TransformStamped
import py_trees
from py_trees.common import Access
from tf2_ros import StaticTransformBroadcaster


class OffsetPose(py_trees.behaviour.Behaviour):
    """Copy a PoseStamped on the blackboard and apply a simple XYZ offset."""

    def __init__(
        self,
        name: str,
        node,
        input_key: str = "selected_yolo_pose",
        output_key: str = "arm_target_pose",
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        offset_z: float = 0.0,
        orientation_xyzw=None,
        debug_child_frame_id: str = "",
    ):
        super().__init__(name)
        self.node = node
        self.input_key = input_key
        self.output_key = output_key
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)
        self.offset_z = float(offset_z)
        self.orientation_xyzw = orientation_xyzw
        self.debug_child_frame_id = debug_child_frame_id

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.input_key, Access.READ)
        self.bb.register_key(self.output_key, Access.WRITE)
        self.broadcaster = None

    def setup(self, **kwargs):
        self.broadcaster = StaticTransformBroadcaster(self.node)

    def update(self):
        source_pose = getattr(self.bb, self.input_key, None)
        if source_pose is None:
            self.node.get_logger().warn(f"[OffsetPose] No pose available at blackboard key '{self.input_key}'")
            return py_trees.common.Status.FAILURE

        target_pose = deepcopy(source_pose)
        target_pose.pose.position.x += self.offset_x
        target_pose.pose.position.y += self.offset_y
        target_pose.pose.position.z += self.offset_z
        if self.orientation_xyzw is not None:
            target_pose.pose.orientation.x = float(self.orientation_xyzw[0])
            target_pose.pose.orientation.y = float(self.orientation_xyzw[1])
            target_pose.pose.orientation.z = float(self.orientation_xyzw[2])
            target_pose.pose.orientation.w = float(self.orientation_xyzw[3])
        setattr(self.bb, self.output_key, target_pose)
        self._publish_debug_tf(target_pose)

        p = target_pose.pose.position
        self.node.get_logger().info(
            f"[OffsetPose] {self.output_key} = {target_pose.header.frame_id} "
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
