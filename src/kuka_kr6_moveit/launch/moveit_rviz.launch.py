from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    xacro_mappings = {
        "use_sim": "false",
        "use_fake_hardware": "true",
        "sim_gazebo": "false"
    }

    moveit_config = MoveItConfigsBuilder("kuka_kr6r900sixx", package_name="kuka_kr6_moveit").robot_description(mappings=xacro_mappings).to_moveit_configs()

    return generate_moveit_rviz_launch(moveit_config)
