import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend_driver = "direct"  # direct | stm
    urdf_path = os.path.join(get_package_share_directory("megajaw_description"), "urdf", "megajaw.xacro.urdf")
    controller_config = PathJoinSubstitution([FindPackageShare("megajaw_bringup"), "config", "diff_drive_controller.yaml"])
    gripper_controller_config = PathJoinSubstitution([FindPackageShare("megajaw_bringup"), "config", "gripper_controller.yaml"])

    robot_description_content = Command(["xacro ", urdf_path, f" backend_driver:={backend_driver}"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": False}],
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
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--param-file",
            gripper_controller_config,
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

    gripper_control = None
    if backend_driver == "stm":
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

    fsm_node = Node(
        package="megajaw_brain",
        executable="fsm_node",
        output="screen",
        parameters=[
            {
                "W_MAX": 0.7,
                "KW": 0.7,
                "V_MAX": 0.5,
                "KV": 0.9,
                "close_thresh": 0.5,  # meters
                "use_sim_time": False,
            }
        ],
    )

    launch_nodes = [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        gripper_spawner,
        fsm_node,
        rosbridge,
    ]

    if gripper_control is not None:
        launch_nodes.append(gripper_control)

    return LaunchDescription(launch_nodes)
