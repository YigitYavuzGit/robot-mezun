import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world file')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    )

    pkg_description = get_package_share_directory('kuka_kr6_description')
    pkg_gazebo = get_package_share_directory('kuka_kr6_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_description_content = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'kuka_kr6.urdf.xacro'),
        ' use_sim:=true use_fake_hardware:=false sim_gazebo:=true'
    ])
    
    robot_description = {'robot_description': robot_description_content}

    world_file = PathJoinSubstitution([pkg_gazebo, 'worlds', LaunchConfiguration('world')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'kuka_kr6', '-z', '0.0', '-allow_renaming', 'true'],
        output='screen'
    )

    # Delay controller spawners to wait for gz_ros2_control to initialize
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Use TimerAction to delay controller spawning
    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_arm_controller = TimerAction(
        period=7.0,
        actions=[arm_controller_spawner]
    )

    rviz_config_file = os.path.join(pkg_description, 'rviz', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        clock_bridge,
        rviz,
    ])
