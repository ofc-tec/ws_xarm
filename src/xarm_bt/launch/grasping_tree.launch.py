from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    object_world = LaunchConfiguration("object_world", default="default")
    object_model = LaunchConfiguration("object_model", default="/home/oscar/gz_models/ycb_013_apple/model-1_4.sdf")
    object_name = LaunchConfiguration("object_name", default="ycb_013_apple")
    object_x = LaunchConfiguration("object_x", default="-0.2")
    object_y = LaunchConfiguration("object_y", default="-0.9")
    object_z = LaunchConfiguration("object_z", default="1.01")
    yolo_image_topic = LaunchConfiguration("yolo_image_topic", default="/camera/color/image_raw")
    yolo_depth_topic = LaunchConfiguration("yolo_depth_topic", default="/camera/depth/image")
    yolo_depth_info_topic = LaunchConfiguration("yolo_depth_info_topic", default="/camera/depth/camera_info")
    yolo_model_path = LaunchConfiguration("yolo_model_path", default="yolo11n-seg.pt")

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
    move_group_interface_params = {
        "robot_description": moveit_config.to_dict()["robot_description"],
        "robot_description_semantic": moveit_config.to_dict()["robot_description_semantic"],
        "robot_description_kinematics": moveit_config.to_dict()["robot_description_kinematics"],
    }

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_world_static_tf",
            output="screen",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "1",
                "map",
                "world",
            ],
        ),
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
            package="ros_gz_sim",
            executable="create",
            name="spawn_object",
            output="screen",
            arguments=[
                "-world",
                object_world,
                "-file",
                object_model,
                "-name",
                object_name,
                "-x",
                object_x,
                "-y",
                object_y,
                "-z",
                object_z,
            ],
        ),
        # Existing Robotino perception service. This package comes from
        # ~/robotino_ros2_ws and expects robotino_interfaces to be sourced.
        Node(
            package="vision",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[
                {
                    "image_topic": yolo_image_topic,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        Node(
            package="vision",
            executable="yolo_server",
            name="yolo_server",
            output="screen",
            parameters=[
                {
                    "image_topic": yolo_image_topic,
                    "depth_topic": yolo_depth_topic,
                    "depth_info_topic": yolo_depth_info_topic,
                    "model_path": yolo_model_path,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        # Existing Robotino auxiliary vision services. The Robotino vision_node
        # waits for these services before opening its OpenCV windows.
        Node(
            package="vision",
            executable="face_recog_service_node",
            name="face_recog_service_node",
            output="screen",
            parameters=[
                {
                    "image_topic": yolo_image_topic,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        Node(
            package="vision",
            executable="pose_service_node",
            name="pose_service_node",
            output="screen",
            parameters=[
                {
                    "image_topic": yolo_image_topic,
                    "depth_topic": yolo_depth_topic,
                    "depth_info_topic": yolo_depth_info_topic,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        # Existing Robotino TTS service from ~/robotino_ros2_ws.
        # Provides /tts/talk for BT fallback messages such as "no apple found".
        Node(
            package="robotino_tts",
            executable="espeak_tts_node",
            name="espeak_tts_node",
            output="screen",
            parameters=[
                {
                    "voice": "",
                    "speed": 140,
                    "pitch": 50,
                    "volume": 100,
                    "interrupt": True,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="xarm_pose_action",
                    executable="set_joints_action_server",
                    name="set_joints_action_server",
                    output="screen",
                    parameters=[
                        move_group_interface_params,
                        {
                            "planning_group": "xarm6",
                            "action_name": "set_joints",
                            "execute": True,
                            "velocity_scaling": 0.9,
                            "acceleration_scaling": 0.9,
                            "planning_time": 25.0,
                            "planning_attempts": 10,
                            "use_sim_time": use_sim_time,
                        },
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dof", default_value="6"),
        DeclareLaunchArgument("robot_type", default_value="xarm"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("object_world", default_value="default"),
        DeclareLaunchArgument("object_model", default_value="/home/oscar/gz_models/ycb_013_apple/model-1_4.sdf"),
        DeclareLaunchArgument("object_name", default_value="ycb_013_apple"),
        DeclareLaunchArgument("object_x", default_value="-0.2"),
        DeclareLaunchArgument("object_y", default_value="-0.9"),
        DeclareLaunchArgument("object_z", default_value="1.01"),
        DeclareLaunchArgument("yolo_image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("yolo_depth_topic", default_value="/camera/depth/image"),
        DeclareLaunchArgument("yolo_depth_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("yolo_model_path", default_value="yolo11n-seg.pt"),
        OpaqueFunction(function=launch_setup),
    ])
