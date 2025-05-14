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
    # parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    # parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None


def generate_launch_description():

    base_path=get_package_share_directory("engineering_robot_moveit")

    moveit_config = (
        MoveItConfigsBuilder("engineering_robot", package_name="engineering_robot_moveit")
        # .planning_scene_monitor(
        #     publish_robot_description=True, publish_robot_description_semantic=True
        # )
        .robot_description(file_path=os.path.join(base_path, "config", "engineering_robot.urdf.xacro"))
        .trajectory_execution(file_path=os.path.join(base_path, "config", "moveit_controllers.yaml"))
        .robot_description_semantic(file_path=os.path.join(base_path, "config", "engineering_robot.srdf"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    print(moveit_config.to_dict())

    # MotionPlanningPipeline demo executable
    motion_planning_pipeline_demo = Node(
        name="pipe_try",
        package="pipe_try",
        executable="pipe_try",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            # moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([motion_planning_pipeline_demo])