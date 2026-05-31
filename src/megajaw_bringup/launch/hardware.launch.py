import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("megajaw_description"), "urdf", "megajaw.xacro.urdf"
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("megajaw_bringup"), "config", "diff_drive_controller.yaml"]
    )

    robot_description_content = Command(["xacro ", urdf_path, " sim:=false"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": False}
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            controller_config,
        ],
    )

    # Pass param file to joint_state_broadcaster too
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controller_config,
            "--controller-manager-timeout",
            "50.0",
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--param-file",
            controller_config,
            "--controller-ros-args",
            "-r /diff_drive_base_controller/cmd_vel:=/cmd_vel",
            "--controller-manager-timeout",
            "50.0",
        ],
        output="screen",
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
    )

    gripper_control = Node(
        package="megajaw_hardware",
        executable="gripper_control_node",
        name="gripper_control_node",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/ttyAMA0",
                "baudrate": 115200,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            # Camera driver node for real hardware
            Node(
                package="megajaw_hardware",
                executable="camera_driver_node",
                name="camera_driver_node",
                output="screen",
                parameters=[
                    {
                        "camera_urls": [
                            "http://192.168.1.11:8080/video",
                            "http://10.152.247.225:8080/video",
                        ]
                    },
                    {"use_sim_time": False},
                ],
                respawn=True,
                respawn_delay=5.0,
            ),
            # Detector node (real hardware mode)
            Node(
                package="megajaw_brain",
                executable="detector_node",
                name="detector_node",
                output="screen",
                parameters=[
                    {"max_lost_frames": 30},  # note: phone cam runs at 30fps
                    {"conf_thresh": 0.7},
                    {"debug": True},
                    {"is_sim": False},
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="megajaw_brain",
                executable="fsm_node",
                output="screen",
                parameters=[
                    {
                        "close_thresh": 0.05,
                        "use_sim_time": False,
                    }
                ],
            ),
            rosbridge,
            gripper_control,
        ]
    )
