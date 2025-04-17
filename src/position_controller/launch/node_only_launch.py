from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import launch
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo

def generate_launch_description():

    Path = get_package_share_directory("position_controller")

    robot_package_path=get_package_share_directory("example_robot_model_moveit")
    robot_launch_file_path=os.path.join(robot_package_path,"launch","launch.py")

    print(f'robot_launch_file_path : {robot_launch_file_path}')

    robot_launch = IncludeLaunchDescription(
        # 指定是 Python 启动文件源
        PythonLaunchDescriptionSource(robot_launch_file_path)
    )

    all = launch_ros.actions.ComposableNodeContainer(
        name="Engineering_robot_Resoloving_RM2025_Pnx",
        namespace="",
        package="rclcpp_components",
        executable='component_container_mt',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="position_controller",
                plugin="Engineering_robot_RM2025_Pnx::PositionController",
                name="position_controller",
            )
        ],
        output="screen"
    )



    return LaunchDescription([
        all
    ])