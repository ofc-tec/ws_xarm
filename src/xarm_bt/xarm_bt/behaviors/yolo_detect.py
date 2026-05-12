import py_trees
from py_trees.common import Access

from robotino_interfaces.srv import YoloDetect as YoloDetectSrv


class YoloDetect(py_trees.behaviour.Behaviour):
    """Call Robotino's YOLO service and store aligned classes/poses on the blackboard."""

    def __init__(
        self,
        name: str,
        node,
        service_name: str = "/yolo_detect",
        classes_key: str = "yolo_class_names",
        poses_key: str = "yolo_poses",
    ):
        super().__init__(name)
        self.node = node
        self.service_name = service_name
        self.classes_key = classes_key
        self.poses_key = poses_key

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.classes_key, Access.WRITE)
        self.bb.register_key(self.poses_key, Access.WRITE)

        self.client = None
        self.future = None
        self._failed_to_start = False

    def setup(self, **kwargs):
        self.client = self.node.create_client(YoloDetectSrv, self.service_name)

    def initialise(self):
        self.future = None
        self._failed_to_start = False

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(f"[YoloDetect] Service {self.service_name} not available")
            self._failed_to_start = True
            return

        self.future = self.client.call_async(YoloDetectSrv.Request())
        self.node.get_logger().info(f"[YoloDetect] Request sent to {self.service_name}")

    def update(self):
        if self._failed_to_start:
            return py_trees.common.Status.FAILURE

        if self.future is None or not self.future.done():
            return py_trees.common.Status.RUNNING

        try:
            response = self.future.result()
        except Exception as exc:
            self.node.get_logger().warn(f"[YoloDetect] Service call failed: {exc}")
            return py_trees.common.Status.FAILURE

        classes = list(getattr(response, "class_names", []) or [])
        poses = list(getattr(response, "poses", []) or [])
        count = min(len(classes), len(poses))

        setattr(self.bb, self.classes_key, classes[:count])
        setattr(self.bb, self.poses_key, poses[:count])

        self.node.get_logger().info(f"[YoloDetect] classes={classes[:count]}")
        return py_trees.common.Status.SUCCESS
