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
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

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
    ).moveit_cpp().to_moveit_configs()

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="xarm_depth_camera_static_tf",
            output="screen",
            arguments=[
                "0.06746",
                "-0.0175",
                "0.0237",
                "0.7071",
                "0",
                "0.7071",
                "0",
                "link6",
                "UF_ROBOT/link6/cameradepth",
            ],
        ),
        Node(
            package="pc_tools",
            executable="pc_repub_sanitized_cpp",
            name="pc_repub_sanitized_cpp",
            output="screen",
            parameters=[
                {
                    "leaf_size": 0.03,
                    "keep_every": 2,
                }
            ],
        ),
        Node(
            package="xarm_bt",
            executable="set_pose_example",
            name="xarm_bt_set_pose_example",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dof", default_value="6"),
        DeclareLaunchArgument("robot_type", default_value="xarm"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
