import re
from copy import deepcopy

import py_trees
from geometry_msgs.msg import TransformStamped
from py_trees.common import Access
from tf2_ros import StaticTransformBroadcaster


class SelectYoloTarget(py_trees.behaviour.Behaviour):
    """Select a YOLO detection and publish an xArm-side corrected debug TF."""

    def __init__(
        self,
        name: str,
        node,
        target_class: str = "apple",
        classes_key: str = "yolo_class_names",
        poses_key: str = "yolo_poses",
        target_pose_key: str = "selected_yolo_pose",
        corrected_frame_prefix: str = "xarm_target",
    ):
        super().__init__(name)
        self.node = node
        self.target_class = target_class
        self.classes_key = classes_key
        self.poses_key = poses_key
        self.target_pose_key = target_pose_key
        self.corrected_frame_prefix = corrected_frame_prefix

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.classes_key, Access.READ)
        self.bb.register_key(self.poses_key, Access.READ)
        self.bb.register_key(self.target_pose_key, Access.WRITE)

        self.broadcaster = None

    def setup(self, **kwargs):
        self.broadcaster = StaticTransformBroadcaster(self.node)

    def update(self):
        classes = list(getattr(self.bb, self.classes_key, []) or [])
        poses = list(getattr(self.bb, self.poses_key, []) or [])

        selected_index = self._find_target_index(classes, poses)
        if selected_index is None:
            self.node.get_logger().warn(f"[SelectYoloTarget] No {self.target_class} with valid 3D pose")
            return py_trees.common.Status.FAILURE

        selected_class = classes[selected_index]
        source_pose = poses[selected_index]
        corrected_pose = self._correct_pose(source_pose)
        setattr(self.bb, self.target_pose_key, corrected_pose)
        self._publish_debug_tf(corrected_pose, selected_class, selected_index)

        sp = source_pose.pose.position
        p = corrected_pose.pose.position
        self.node.get_logger().info(
            f"[SelectYoloTarget] {selected_class} -> "
            f"raw {source_pose.header.frame_id}: ({sp.x:.3f}, {sp.y:.3f}, {sp.z:.3f}), "
            f"corrected {corrected_pose.header.frame_id}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        return py_trees.common.Status.SUCCESS

    def _find_target_index(self, classes, poses):
        for i, class_name in enumerate(classes[: len(poses)]):
            if self.target_class.lower() not in class_name.lower():
                continue
            if poses[i].header.frame_id:
                return i
        return None

    def _correct_pose(self, source_pose):
        corrected_pose = deepcopy(source_pose)

        # YOLO back-projection uses optical camera coordinates:
        #   x right, y down, z forward.
        # The xArm depth frame is kept stable for the pointcloud/octomap path,
        # so only the detection point gets remapped for BT/RViz debugging.
        x_opt = source_pose.pose.position.x
        y_opt = source_pose.pose.position.y
        z_opt = source_pose.pose.position.z

        corrected_pose.pose.position.x = z_opt
        corrected_pose.pose.position.y = -x_opt
        corrected_pose.pose.position.z = -y_opt
        corrected_pose.pose.orientation.x = 0.0
        corrected_pose.pose.orientation.y = 0.0
        corrected_pose.pose.orientation.z = 0.0
        corrected_pose.pose.orientation.w = 1.0
        return corrected_pose

    def _publish_debug_tf(self, pose, class_name, index):
        if self.broadcaster is None or not pose.header.frame_id:
            return

        safe_class = re.sub(r"[^A-Za-z0-9_]+", "_", class_name.strip()) or "object"
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = f"{self.corrected_frame_prefix}_{safe_class}_{index}"
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.broadcaster.sendTransform(transform)
