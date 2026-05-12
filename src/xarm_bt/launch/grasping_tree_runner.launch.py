from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    start_delay_sec = LaunchConfiguration("start_delay_sec", default="0.0")
    use_introspection = LaunchConfiguration("use_introspection", default="false")
    with_viewer = LaunchConfiguration("with_viewer", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_delay_sec", default_value="0.0"),
        DeclareLaunchArgument("use_introspection", default_value="false"),
        DeclareLaunchArgument("with_viewer", default_value="false"),
        Node(
            package="xarm_bt",
            executable="grasping_tree",
            name="xarm_bt_grasping_tree",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"start_delay_sec": start_delay_sec},
                {"use_introspection": use_introspection},
            ],
        ),
        ExecuteProcess(
            condition=IfCondition(with_viewer),
            cmd=["py-trees-tree-viewer"],
            output="screen",
        ),
    ])
