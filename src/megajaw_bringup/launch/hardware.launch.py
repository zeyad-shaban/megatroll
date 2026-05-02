import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('megajaw_description'),
        'urdf',
        'megajaw.xacro.urdf'
    )
    controller_config = PathJoinSubstitution([
        FindPackageShare('megajaw_bringup'),
        'config',
        'diff_drive_controller.yaml'
    ])

    robot_description_content = Command(['xacro ', urdf_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': False}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Pass param file to joint_state_broadcaster too
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controller_config,
            '--controller-manager-timeout', '50.0'
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file', controller_config,
            '--controller-ros-args',
            '-r /diff_drive_base_controller/cmd_vel:=/cmd_vel',   # combined string
            '--controller-manager-timeout', '50.0'
        ],
        output='screen'
    )
    
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        rosbridge,
    ])
