from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import launch
import xacro
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    launch_config={
        "robot_ip": "", # real robot_control_ip
        "use_fake_hardware": "true", # use vitrual robot
        "gripper": "", # end effector type
        "dof": "7", # freedom degree
    }

    robot_controllerPath = get_package_share_directory("robot_controller")
    example_robot_model_moveitPath = get_package_share_directory("example_robot_model_moveit")

    robot_description_path = os.path.join(example_robot_model_moveitPath,"config/example_robot_model.urdf.xacro")
    robot_description_semantic_path = os.path.join(example_robot_model_moveitPath,"config/example_robot_model.srdf")
    config_path = os.path.join(robot_controllerPath,"config/sample_config.yaml")
    moveit_controller = os.path.join(example_robot_model_moveitPath,"config","moveit_controllers.yaml")

    robot_description=xacro.process_file(robot_description_path).toxml()

    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic = file.read()


    moveit_config=(
        MoveItConfigsBuilder(
            "example_robot_model", package_name="example_robot_model_moveit"
        )
        .robot_description(file_path=robot_description_path)
        .robot_description_semantic(file_path=robot_description_semantic_path)
        .trajectory_execution(file_path=moveit_controller,moveit_manage_controllers=True)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"] # load some different kind planner
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="robot_manager",
            executable="robot_manager",
            name="robot_manager",
            parameters=[
                config_path,
                moveit_config.to_dict(),
                {"planning_plugins":["ompl_interface/OMPLPlanner", "pilz_industrial_motion_planner/CommandPlanner"]}
                ]
        )
    ])