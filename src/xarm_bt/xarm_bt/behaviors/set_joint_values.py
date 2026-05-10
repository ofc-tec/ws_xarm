import py_trees
from py_trees.common import Access

class SetJointValues(py_trees.behaviour.Behaviour):
    """MoveItPy-style BT leaf for an xArm joint-space goal."""

    def __init__(
        self,
        name: str,
        joint_values=None,
        joint_values_key: str = "arm_joint_values",
        group_name: str = "xarm6",
        execute: bool = False,
    ):
        super().__init__(name)
        self.joint_values = joint_values
        self.joint_values_key = joint_values_key
        self.group_name = group_name
        self.execute = bool(execute)

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(self.joint_values_key, Access.READ)

        self.moveit = None
        self.arm = None
        self.plan_params = None
        self.robot_state_cls = None
        self._done = False
        self._success = False

    def setup(self, **kwargs):
        from moveit.core.robot_state import RobotState
        from moveit.planning import MoveItPy, PlanRequestParameters

        self.moveit = MoveItPy(node_name="xarm_bt_moveit_py")
        self.arm = self.moveit.get_planning_component(self.group_name)
        self.plan_params = PlanRequestParameters(self.moveit, "")
        self.robot_state_cls = RobotState

    def initialise(self):
        self._done = False
        self._success = False

    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS if self._success else py_trees.common.Status.FAILURE

        joint_values = self.joint_values
        if joint_values is None:
            joint_values = getattr(self.bb, self.joint_values_key, None)

        if joint_values is None:
            self.logger.info("[SetJointValues] No joint values available")
            self._done = True
            self._success = False
            return py_trees.common.Status.FAILURE

        try:
            robot_state = self.robot_state_cls(self.moveit.get_robot_model())
            robot_state.set_joint_group_positions(self.group_name, list(joint_values))
            robot_state.update()

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(robot_state=robot_state)

            if self.execute:
                self._success = self.arm.plan_and_execute_with_single_pipeline(self.plan_params, self.moveit)
            else:
                self._success = self.arm.plan_with_single_pipeline(self.plan_params)

            message = "succeeded" if self._success else "failed"
            self.logger.info(f"[SetJointValues] Joint goal {message}")
        except Exception as exc:
            self._success = False
            self.logger.info(f"[SetJointValues] MoveItPy exception: {exc}")

        self._done = True
        return py_trees.common.Status.SUCCESS if self._success else py_trees.common.Status.FAILURE
