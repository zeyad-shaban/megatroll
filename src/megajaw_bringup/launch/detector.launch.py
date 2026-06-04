import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug_mode_arg = DeclareLaunchArgument("debug", default_value="true", description="Enable debug mode")

    camera_driver = Node(
        package="megajaw_wifi_camera",
        executable="camera_driver_node",
        name="camera_driver_node",
        output="screen",
        parameters=[
            {
                "camera_urls": [
                    "http://192.168.1.11:8080/video",
                    "http://192.168.43.1:8080/video",
                ]
            },
        ],
    )

    detector_node = Node(
        package="megajaw_brain",
        executable="detector_node",
        name="detector_node",
        output="screen",
        parameters=[
            {"max_lost_frames": 30},  # note: phone cam runs at 30fps
            {"conf_thresh": 0.7},
            {"debug": PythonExpression(["'", LaunchConfiguration("debug"), "' == 'true'"])},
            {"is_sim": False},
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [
            # Launch args
            debug_mode_arg,
            # Nodes
            camera_driver,
            detector_node,
        ]
    )
