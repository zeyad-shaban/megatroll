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

    robot_description_content = Command(
        ["xacro ", urdf_path, " backend_driver:=stm"]
    )  # direct | stm

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

    camera_driver = Node(
        package="megajaw_hardware",
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
            {"use_sim_time": False},
            # Reconnection behavior parameters
            {"initial_retry_delay_ms": 1000},
            {"max_retry_delay_ms": 30000},
            {"backoff_multiplier": 1.2},
            {"max_consecutive_failures": 999999999},
        ],
    )

    fsm_node = Node(
        package="megajaw_brain",
        executable="fsm_node",
        output="screen",
        parameters=[
            {
                "W_MAX": 0.7,
                "KW": 0.7,
                "V_MAX": 0.6,
                "KV": 1.3,
                "close_thresh": 0.05,  # meters
                "use_sim_time": False,
            }
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
            {"debug": False},
            {"is_sim": False},
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            camera_driver,
            detector_node,
            fsm_node,
            rosbridge,
            gripper_control,
        ]
    )
