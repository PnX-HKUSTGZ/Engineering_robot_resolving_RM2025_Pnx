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

def generate_launch_description():
    robot_controllerPath = get_package_share_directory("robot_controller")
    example_robot_model_moveitPath = get_package_share_directory("example_robot_model_moveit")

    robot_description = os.path.join(example_robot_model_moveitPath,"config/example_robot_model.urdf.xacro")
    config_path= os.path.join(robot_controllerPath,"config/sample_config.yaml")


    return LaunchDescription([
        Node(
            package="robot_manager",
            executable="robot_manager",
            name="robot_manager",
            parameters=[
                config_path,
                {"my_robot_description":robot_description},
                ]
        )
    ])