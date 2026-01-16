import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = get_package_share_directory('kuka_kr6_description')

    robot_description_content = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'kuka_kr6.urdf.xacro'),
        ' use_sim:=false use_fake_hardware:=true sim_gazebo:=false'
    ])
    robot_description = {'robot_description': robot_description_content}

    rviz_config_file = os.path.join(pkg_description, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
        ),
    ])
