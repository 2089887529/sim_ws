from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('chassis_description')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_path, 'urdf', 'chassis.urdf.xacro')

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    wait_for_clock = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'until ros2 topic echo /clock --once > /dev/null 2>&1; do '
            'sleep 0.2; '
            'done'
        ],
        output='screen'
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'chassis',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_position_controller'],
        output='screen'
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
        output='screen'
    )

    spawn_after_clock_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_clock,
            on_exit=[spawn_entity_node]
        )
    )

    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                TimerAction(
                    period=2.0,   # 等待 gz_ros2_control 初始化完成
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    load_steering_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[steering_controller_spawner]
        )
    )

    load_wheel_after_steering = RegisterEventHandler(
        OnProcessExit(
            target_action=steering_controller_spawner,
            on_exit=[wheel_controller_spawner]
        )
    )

    ld = LaunchDescription()
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(clock_bridge_node)
    ld.add_action(wait_for_clock)
    ld.add_action(spawn_after_clock_ready)
    ld.add_action(load_jsb_after_spawn)
    ld.add_action(load_steering_after_jsb)
    ld.add_action(load_wheel_after_steering)

    return ld