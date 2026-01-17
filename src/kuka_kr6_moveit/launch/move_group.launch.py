from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    xacro_mappings = {
        "use_sim": "false",
        "use_fake_hardware": "true",
        "sim_gazebo": "false"
    }
    
    moveit_config = MoveItConfigsBuilder("kuka_kr6r900sixx", package_name="kuka_kr6_moveit").robot_description(mappings=xacro_mappings).to_moveit_configs()

    return generate_move_group_launch(moveit_config)
