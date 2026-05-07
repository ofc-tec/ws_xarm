from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import PoseStamped


@dataclass
class ArmResult:
    success: bool
    message: str


class XArmMoveItPyCommander:
    """Small Takeshi-style wrapper around MoveItPy.

    This deliberately presents a simple API for BT leaves:
      arm.set_pose(...)
      arm.set_named_target(...)

    MoveItPy must be available in the sourced environment.
    """

    def __init__(
        self,
        group_name: str = "xarm6",
        node_name: str = "xarm_bt_moveit_py",
        default_pose_link: str = "",
        logger=None,
    ):
        from moveit.planning import MoveItPy, PlanRequestParameters

        self.group_name = group_name
        self.default_pose_link = default_pose_link
        self.logger = logger
        self.robot = MoveItPy(node_name=node_name)
        self.group = self.robot.get_planning_component(group_name)
        self.plan_params = PlanRequestParameters(self.robot, "")

    def set_pose(
        self,
        pose: PoseStamped,
        pose_link: Optional[str] = None,
        execute: bool = False,
    ) -> ArmResult:
        if not isinstance(pose, PoseStamped):
            return ArmResult(False, "set_pose expected geometry_msgs/msg/PoseStamped")

        link = pose_link if pose_link is not None else self.default_pose_link
        try:
            self.group.set_start_state_to_current_state()
            kwargs = {"pose_stamped_msg": pose}
            if link:
                kwargs["pose_link"] = link
            self.group.set_goal_state(**kwargs)

            if execute:
                executed = self.group.plan_and_execute_with_single_pipeline(self.plan_params, self.robot)
                if not executed:
                    return ArmResult(False, "Planning or execution failed")
                return ArmResult(True, "Planning and execution succeeded")

            plan_succeeded = self.group.plan_with_single_pipeline(self.plan_params)
            if not plan_succeeded:
                return ArmResult(False, "Planning failed")

            return ArmResult(True, "Planning succeeded; execution disabled")
        except Exception as exc:
            return ArmResult(False, f"MoveItPy set_pose exception: {exc}")

    def set_named_target(self, target_name: str, execute: bool = False) -> ArmResult:
        try:
            self.group.set_start_state_to_current_state()
            self.group.set_goal_state(configuration_name=target_name)

            if execute:
                executed = self.group.plan_and_execute_with_single_pipeline(self.plan_params, self.robot)
                if not executed:
                    return ArmResult(False, f"Planning or execution to named target '{target_name}' failed")
                return ArmResult(True, f"Named target '{target_name}' executed")

            plan_succeeded = self.group.plan_with_single_pipeline(self.plan_params)
            if not plan_succeeded:
                return ArmResult(False, f"Planning to named target '{target_name}' failed")

            return ArmResult(True, f"Named target '{target_name}' planned")
        except Exception as exc:
            return ArmResult(False, f"MoveItPy set_named_target exception: {exc}")
