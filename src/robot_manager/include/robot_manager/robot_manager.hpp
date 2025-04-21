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

class MoveItPlanningNode : public rclcpp::Node{
public:
    // 构造函数
    MoveItPlanningNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // 初始化MoveIt相关功能
    bool initializeMoveIt();

    // 规划和执行一个基于位姿的目标
    // @param target_joint 目标规划末端
    // @param pose 目标位置
    // @param path_constraints_ 移动时目标角度约束
    bool planToPoseGoal(const std::string & target_joint, const geometry_msgs::msg::PoseStamped& pose, moveit_msgs::msg::Constraints::SharedPtr path_constraints_=nullptr);

    // 规划和执行一个基于关节空间的目标
    bool planToJointGoal(const std::vector<double>& joint_values);

    // 规划和执行一个带路径约束的位姿目标
    bool planToConstrainedGoal(const geometry_msgs::msg::PoseStamped& pose);

    // 可视化规划轨迹
    void visualizeTrajectory(const moveit_msgs::msg::MotionPlanResponse& response);

    const std::shared_ptr<moveit_visual_tools::MoveItVisualTools> getVisualTools();

    // get_robot_statement
    void get_robot_statement();

    // print_robot_info
    void print_robot_info();
private:

    // ROS2 相关成员
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher_;

    // MoveIt 相关成员
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    planning_scene::PlanningScenePtr planning_scene_;
    std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
    planning_interface::PlannerManagerPtr planner_instance_;
    std::vector<std::string> planner_plugin_names_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    moveit_msgs::msg::DisplayTrajectory display_trajectory_;

    //tf2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listenser_;

    //trigger
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool> > trigger_sub_;

    //param
    std::string robot_description_path;
    std::string robot_description_semantic_path;
    std::string PLANNING_GROUP;
    std::string EndEffectorJoint;

    //
    std::shared_ptr<rclcpp::TimerBase> print_robot_info_timer;

private:
    //function

    void trigger_sub_callback(const std_msgs::msg::Bool::SharedPtr msg_);


};

} // namespace motion_planning_api

#endif // Engineering_robot_RM2025_Pnx
