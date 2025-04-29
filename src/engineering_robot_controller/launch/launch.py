from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import xacro
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch_ros.descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

def generate_launch_description():

    launch_config={
        "robot_ip": "", # real robot_control_ip
        "use_fake_hardware": "true", # use vitrual robot
        "gripper": "", # end effector type
        "dof": "7", # freedom degree
    }

    print(1)
    robot_controllerPath = get_package_share_directory("engineering_robot_controller")
    print(1)
    engineering_robot_moveitPath = get_package_share_directory("engineering_robot_moveit")
    print(1)

    robot_description_path = os.path.join(engineering_robot_moveitPath,"config/engineering_robot.urdf.xacro")
    robot_description_semantic_path = os.path.join(engineering_robot_moveitPath,"config/engineering_robot.srdf")
    config_path = os.path.join(robot_controllerPath,"config/sample_config.yaml")
    moveit_controller = os.path.join(engineering_robot_moveitPath,"config","moveit_controllers.yaml")
    ros2_controllers_path=os.path.join(engineering_robot_moveitPath,"config","ros2_controllers.yaml")

    robot_description=xacro.process_file(robot_description_path).toxml()

    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic = file.read()

    use_sim_time={"use_sim_time": True}

    moveit_config=(
        MoveItConfigsBuilder(
            "engineering_robot", package_name="engineering_robot_moveit"
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
        parameters=[
            moveit_config.to_dict(),
            # use_sim_time,
            ],
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
        parameters=[
            moveit_config.robot_description,
            # use_sim_time
            ],
        # namespace="example_robot",
    )

    # ros2_control

    ros2_control_node= Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="ros2_control_node",
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
        # parameters=[use_sim_time]
    )
    # launch this to pub robot state to /joint_states

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "controller_manager"], 
        # parameters=[use_sim_time]
        # namespace="example_robot",
    )

    main_node=Node(
        package="engineering_robot_controller",
        executable="engineering_robot_controller",
        name="engineering_robot_controller",
        parameters=[
            config_path,
            moveit_config.to_dict(),
            {"planning_plugins":["ompl_interface/OMPLPlanner", "pilz_industrial_motion_planner/CommandPlanner"]},
            use_sim_time
            ],
        # respawn=True,
        # respawn_delay=2.0
    )

    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("engineering_robot_moveit"), "config", rviz_base]
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
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        static_tf,
        rviz_config_arg,
        rviz_node,
        # main_node,
    ])