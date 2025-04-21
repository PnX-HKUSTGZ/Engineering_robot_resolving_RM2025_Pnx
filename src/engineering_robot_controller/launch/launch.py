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
    ros2_controllers_path=os.path.join(example_robot_model_moveitPath,"config","ros2_controllers.yaml")

    robot_description=xacro.process_file(robot_description_path).toxml()

    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic = file.read()


    moveit_config=(
        MoveItConfigsBuilder(
            "example_robot_model", package_name="example_robot_model_moveit"
        )
        .robot_description(mappings=launch_config)
        .trajectory_execution(file_path=moveit_controller,moveit_manage_controllers=True)
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

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "base", "--child-frame-id", "robot_base"],
        # namespace="example_robot",
    )

    # future we will not use static but dynamic

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="log",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.planning_pipelines,
    #         moveit_config.joint_limits,
    #     ],
    #     # namespace="example_robot",
    # )

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

    ros2_control_node= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "robot_description"),
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
            "controller_manager",
        ],# set this node use joint_state_broadcaster controller and conmunicate with /controller_manager
        # namespace="example_robot",
    )
    # launch this to pub robot state to /joint_states

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_controller", "-c", "controller_manager"], 
        # namespace="example_robot",
    )

    main_node=Node(
        package="engineering_robot_controller",
        executable="Engineering_robot_Controller",
        name="Engineering_robot_Controller",
        parameters=[
            config_path,
            moveit_config.to_dict(),
            {"planning_plugins":["ompl_interface/OMPLPlanner", "pilz_industrial_motion_planner/CommandPlanner"]}
            ]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        # rviz_node,
        main_node,
    ])