#!/usr/bin/env python3

import json
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration("dof", default="6")
    robot_type = LaunchConfiguration("robot_type", default="xarm")
    prefix = LaunchConfiguration("prefix", default="")
    hw_ns = LaunchConfiguration("hw_ns", default="xarm")
    limited = LaunchConfiguration("limited", default="true")
    effort_control = LaunchConfiguration("effort_control", default="false")
    velocity_control = LaunchConfiguration("velocity_control", default="false")
    model1300 = LaunchConfiguration("model1300", default="false")
    robot_sn = LaunchConfiguration("robot_sn", default="")

    moveit_config_dump = LaunchConfiguration("moveit_config_dump", default="").perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}

    if not moveit_config_dict:
        moveit_config = MoveItConfigsBuilder(
            context=context,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300=model1300,
            robot_sn=robot_sn,
        ).to_moveit_configs()
        moveit_config_dict = moveit_config.to_dict()

    move_group_interface_params = {
        "robot_description": moveit_config_dict["robot_description"],
        "robot_description_semantic": moveit_config_dict["robot_description_semantic"],
        "robot_description_kinematics": moveit_config_dict["robot_description_kinematics"],
    }

    node_parameters = LaunchConfiguration("node_parameters", default="{}")
    try:
        action_params = json.loads(node_parameters.perform(context))
    except Exception:
        action_params = {}

    return [
        Node(
            package="xarm_pose_action",
            executable="set_joints_action_server",
            name=LaunchConfiguration("node_name", default="set_joints_action_server"),
            output="screen",
            parameters=[
                move_group_interface_params,
                {
                    "planning_group": LaunchConfiguration("planning_group", default="xarm6"),
                    "action_name": LaunchConfiguration("action_name", default="set_joints"),
                    "execute": LaunchConfiguration("execute", default="true"),
                    "velocity_scaling": LaunchConfiguration("velocity_scaling", default="0.2"),
                    "acceleration_scaling": LaunchConfiguration("acceleration_scaling", default="0.1"),
                    "planning_time": LaunchConfiguration("planning_time", default="5.0"),
                    "planning_attempts": LaunchConfiguration("planning_attempts", default="5"),
                    "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
                },
                action_params,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dof", default_value="6"),
        DeclareLaunchArgument("robot_type", default_value="xarm"),
        DeclareLaunchArgument("planning_group", default_value="xarm6"),
        DeclareLaunchArgument("action_name", default_value="set_joints"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
