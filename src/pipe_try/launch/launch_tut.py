import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config")
        # .planning_scene_monitor(
        #     publish_robot_description=True, publish_robot_description_semantic=True
        # )
        .robot_description(file_path=load_file("moveit_resources_panda_description", "urdf/panda.urdf"))
        .trajectory_execution(file_path=load_file("moveit_resources_panda_moveit_config", "config/moveit_controllers.yaml"))
        .robot_description_semantic(file_path=load_file("moveit_resources_panda_moveit_config", "config/panda.srdf"))
        .robot_description_kinematics(file_path=load_yaml("moveit_resources_panda_moveit_config", "config/kinematics.yaml"))
        .planning_pipelines(file_path=load_yaml("moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"))
        # .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    print(89765432)
    return LaunchDescription(
        [
            Node(
                package="pipe_try",
                executable="pipe_try",
                name="motion_planning_api_tutorial",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    planning_plugin,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_planning_pipelines,
                ],
            )
        ]
    )