#ifndef MULTITHREAD_SOLOVE_HPP
#define MULTITHREAD_SOLOVE_HPP

#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <thread>
#include <atomic>
#include <pthread.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pluginlib/pluginlib/class_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/position_constraint.h>
#include <moveit_msgs/msg/object_color.h>
#include <std_msgs/msg/color_rgba.h>

namespace Engineering_robot_RM2025_Pnx {

class MultithreadSolver{

private:
    // if you provide a nodeptr, you have to make sure it init with node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node_;
    moveit::core::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
    

};


}


#endif