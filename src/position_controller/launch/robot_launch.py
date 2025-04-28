from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from engineering_robot_controller.launch.launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace

launch_config={
    "robot_ip": "", # real robot_control_ip
    "use_fake_hardware": "true", # use vitrual robot
    "gripper": "", # end effector type
    "dof": "7", # freedom degree
}

def generate_launch_description():
    location=get_package_share_directory('example_robot_model_moveit')
    robot_location=os.path.join(location,"config","example_robot_model.urdf.xacro")
    moveit_controller=os.path.join(location,"config","moveit_controllers.yaml")
    ros2_controllers_path=os.path.join(location,"config","ros2_controllers.yaml")

    print(f'location:{location}')
    print(f'robot_location:{robot_location}')
    print(f'moveit_controller:{moveit_controller}')
    print(f'ros2_controllers_path:{ros2_controllers_path}')

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

    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["--frame-id", "base", "--child-frame-id", "robot_base"],
    #     # namespace="example_robot",
    # )

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
    # launch a controller node for "body_controller"

    declare_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",  # 设置默认命名空间，可以为空""或你想要的名字
        description="Namespace for all nodes launched in this file",
    )
    robot_namespace = LaunchConfiguration("robot_namespace") # 获取配置的命名空间

    namespaced_group = GroupAction(
        actions=[
            # 首先推送命名空间
            PushRosNamespace(robot_namespace),

            # 然后添加所有需要置于该命名空间下的节点和其他动作
            move_group_node,
            rviz_node,
            # static_tf,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )

    return LaunchDescription([
        declare_namespace_arg,
        rviz_config_arg,
        namespaced_group,
    ])
