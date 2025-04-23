#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <random>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pluginlib/class_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#ifndef MOTION_PLANNING_API_NODE_HPP
#define MOTION_PLANNING_API_NODE_HPP

#define VISUALIZE

namespace Engineering_robot_RM2025_Pnx {
namespace rvt = rviz_visual_tools;
using namespace std::placeholders;
using namespace std::chrono_literals;

class Engineering_robot_Controller: public rclcpp::Node{

public:

Engineering_robot_Controller(rclcpp::NodeOptions node_options=rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

// @return init successfully
bool MoveitInit();

private:

std::string ARM_CONTROL_GROUP;
std::string END_EFFECTOR_CONTROL_GROUP;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
const moveit::core::JointModelGroup* arm_model_group;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_;

rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_trigger_;

tf2_ros::Buffer::SharedPtr tf2_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf2_listenser_;

//receive a sign and control the arm move to target
void planner_trigger_call_back(const std_msgs::msg::Bool::SharedPtr& msg);

};// Engineering_robot_Controller


std::vector<double> eulerToQuaternion(double roll, double pitch, double yaw);
std::vector<double> eulerToQuaternion(const std::vector<double>& euler);
std::vector<double> quaternionToEuler(const std::vector<double>& q);

} // namespace motion_planning_api

#endif // Engineering_robot_RM2025_Pnx
