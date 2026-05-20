import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg = get_package_share_directory('robot_gripper_description')

    urdf_file = os.path.join(pkg, 'urdf', 'gripper_delta_robot.urdf')

    rviz_config_file = os.path.join(pkg, 'rviz', 'display.rviz')


    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
        ),
    ])
