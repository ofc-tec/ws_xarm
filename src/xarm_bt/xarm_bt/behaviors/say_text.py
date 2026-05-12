import py_trees

from robotino_interfaces.srv import Talk


class SayText(py_trees.behaviour.Behaviour):
    """Call Robotino's TTS service."""

    def __init__(
        self,
        name: str,
        node,
        text: str,
        service_name: str = "/tts/talk",
        wait_for_speech: bool = True,
    ):
        super().__init__(name)
        self.node = node
        self.text = text
        self.service_name = service_name
        self.wait_for_speech = wait_for_speech
        self.client = None
        self.future = None
        self._failed_to_start = False

    def setup(self, **kwargs):
        self.client = self.node.create_client(Talk, self.service_name)

    def initialise(self):
        self.future = None
        self._failed_to_start = False

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(f"[SayText] Service {self.service_name} not available")
            self._failed_to_start = True
            return

        request = Talk.Request()
        request.text = self.text
        request.wait = self.wait_for_speech
        self.future = self.client.call_async(request)
        self.node.get_logger().info(f"[SayText] {self.text}")

    def update(self):
        if self._failed_to_start:
            return py_trees.common.Status.FAILURE

        if self.future is None or not self.future.done():
            return py_trees.common.Status.RUNNING

        try:
            response = self.future.result()
        except Exception as exc:
            self.node.get_logger().warn(f"[SayText] Service call failed: {exc}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS if response.success else py_trees.common.Status.FAILURE
