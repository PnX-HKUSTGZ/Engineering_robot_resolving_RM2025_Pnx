from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


launch_config={
    "robot_ip": "", # real robot_control_ip
    "use_fake_hardware": "true", # use vitrual robot
    "gripper": "", # end effector type
    "dof": "6", # freedom degree
}

def generate_launch_description():
    location=get_package_share_directory('example_robot_model_moveit')

    print(f'location : {location}')

    moveit_config=(
        MoveItConfigsBuilder(
            "example_robot_model", package_name="example_robot_model_moveit"
        )
        .robot_description(mappings=launch_config)
        .trajectory_execution(file_path=os.path.join(location,"config","moveit_controllers.yaml"))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"] # load some different kind planner
        )
        .to_moveit_configs()
    )
    move_group_node=Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        namespace="example_robot"
    )
    
    return LaunchDescription([
        move_group_node,
    ])
