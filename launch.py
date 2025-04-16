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
        # namespace="example_robot"
    )

    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("example_robot_model_moveit"), "config", rviz_base]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        # namespace="example_robot",
    )

    # tf2 publish

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "base", "--child-frame-id", "virtual_robot_base"],
        # namespace="example_robot",
    )

    # future we will not use static but dynamic
    
    # Publish TF robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        # namespace="example_robot",
    )

    # ros2_control

    ros2_controllers_path = os.path.join(location,"config","ros2_controllers.yaml")
    print(f'ros2_controllers_path : {ros2_controllers_path}')

    ros2_control_node= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ], # because ros2_control_node listen to "/controller_manager/robot_description" but we always publish robot_description on topic "robot_description". we use this to talk node to map this to topic
        output="both",
        # namespace="example_robot",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],# set this node use joint_state_broadcaster controller and conmunicate with /controller_manager
        # namespace="example_robot",
    )
    # launch this to pub robot state to /joint_states

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_controller", "-c", "/controller_manager"], 
        # namespace="example_robot",
    )
    # launch a controller node for "body_controller"
    

    return LaunchDescription([
        move_group_node,
        rviz_config_arg,
        rviz_node,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])
