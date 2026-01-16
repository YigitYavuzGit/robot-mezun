import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg_description = get_package_share_directory('kuka_kr6_description')
    pkg_moveit = get_package_share_directory('kuka_kr6_moveit')

    # Robot description
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'kuka_kr6.urdf.xacro'),
        ' use_sim:=true use_fake_hardware:=false sim_gazebo:=true'
    ])
    robot_description = {'robot_description': robot_description_content}

    # SRDF
    srdf_path = os.path.join(pkg_moveit, 'config', 'kuka_kr6.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Kinematics
    kinematics_yaml = load_yaml('kuka_kr6_moveit', 'config/kinematics.yaml')
    kinematics = kinematics_yaml['/**']['ros__parameters']

    # OMPL Planning
    ompl_yaml = load_yaml('kuka_kr6_moveit', 'config/ompl_planning.yaml')
    ompl_planning = ompl_yaml['/**']['ros__parameters']

    # Joint limits
    joint_limits_yaml = load_yaml('kuka_kr6_moveit', 'config/joint_limits.yaml')
    joint_limits = joint_limits_yaml['/**']['ros__parameters']

    # MoveIt controllers
    controllers_yaml = load_yaml('kuka_kr6_moveit', 'config/moveit_controllers.yaml')
    moveit_controllers = controllers_yaml['/**']['ros__parameters']

    # Move group node
    move_group_params = {
        'use_sim_time': True,
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
    }
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    move_group_params.update(kinematics)
    move_group_params.update(ompl_planning)
    move_group_params.update(joint_limits)
    move_group_params.update(moveit_controllers)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )

    # RViz
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    rviz_params = {'use_sim_time': True}
    rviz_params.update(robot_description)
    rviz_params.update(robot_description_semantic)
    rviz_params.update(kinematics)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[rviz_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        move_group_node,
        rviz_node,
    ])
