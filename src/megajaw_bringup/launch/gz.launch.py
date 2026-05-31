import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("megajaw_description"), "urdf", "megajaw.xacro.urdf"
    )
    world_file_path = os.path.join(
        get_package_share_directory("megajaw_bringup"), "worlds", "modelled.sdf"
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("megajaw_bringup"),
            "rviz",
            "rviz_config.rviz",
        ]
    )

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    enable_rviz = LaunchConfiguration("rviz", default="true")
    debug_mode = LaunchConfiguration("debug", default="false")

    def robot_state_publisher_callback(context):
        robot_description_content = Command(["xacro ", urdf_path, " sim:=true"])
        robot_description = {"robot_description": robot_description_content}
        node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
        return [node_robot_state_publisher]

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "diff_drive",
            "-allow_renaming",
            "true",
            "-Y",
            "3.14159",
        ],
    )

    # Controller
    diff_drive_controller_config = PathJoinSubstitution(
        [
            FindPackageShare("megajaw_bringup"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )

    gripper_controller_config = PathJoinSubstitution(
        [
            FindPackageShare("megajaw_bringup"),
            "config",
            "gripper_controller.yaml",
        ]
    )

    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--param-file",
            diff_drive_controller_config,
            "--controller-ros-args",
            "-r /diff_drive_base_controller/cmd_vel:=/cmd_vel",
            "--controller-manager-timeout",
            "50.0",
        ],
    )

    gripper_controller_spawner = Node(
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

    # Bridge
    bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("megajaw_bringup"),
            "config",
            "gz_bridge.yaml",
        ]
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "camera.image.compressed.jpeg_quality": 75,
            },
        ],
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package="topic_tools",
        executable="relay",
        name="relay_camera_info",
        output="screen",
        arguments=["camera/camera_info", "camera/image/camera_info"],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(enable_rviz),
        output="screen",
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            diff_drive_controller_config,
            "--controller-manager-timeout",
            "50.0",
        ],
        output="screen",
    )

    # Brain
    fsm_node = Node(
        package="megajaw_brain",
        executable="fsm_node",
        name="fsm_node",
        output="screen",
        parameters=[
            {
                "W_MAX": 0.7,
                "KW": 0.7,
                "V_MAX": 0.6,
                "KV": 1.3,
                "close_thresh": 0.05,
                "use_sim_time": True,
            },
        ],
    )

    detector_node = Node(
        package="megajaw_brain",
        executable="detector_node",
        name="detector_node",
        output="screen",
        parameters=[
            {"debug": True},
        ],
    )

    ld = LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [" -r -v 1 ", world_file_path])],
            ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=gz_spawn_entity,
            #         on_exit=[joint_state_broadcaster_spawner],
            #     )
            # ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[
                        diff_drive_base_controller_spawner,
                        gripper_controller_spawner,
                    ],
                )
            ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster_spawner,
            #         on_exit=[diff_drive_base_controller_spawner],
            #     )
            # ),
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="Launch RViz2"
            ),
            DeclareLaunchArgument(
                "debug_mode", default_value="false", description="Enable debug mode"
            ),
            # Nodes
            bridge,
            gz_spawn_entity,
            rviz_node,
            gz_image_bridge_node,
            relay_camera_info_node,
            rosbridge,
            fsm_node,
            detector_node,
        ]
    )
    ld.add_action(OpaqueFunction(function=robot_state_publisher_callback))

    return ld
