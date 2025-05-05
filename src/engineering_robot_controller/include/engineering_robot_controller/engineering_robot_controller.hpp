#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <command_interfaces/msg/computer_state.hpp>
#include <command_interfaces/msg/player_command.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <thread>
#include <atomic>
#include <pthread.h>

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
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/position_constraint.h>
#include <moveit_msgs/msg/object_color.h>
#include <std_msgs/msg/color_rgba.h>

#ifndef MOTION_PLANNING_API_NODE_HPP
#define MOTION_PLANNING_API_NODE_HPP

#define VISUALIZE

#define PLANNING 2
#define FINISH 3
#define FAILED 0
#define MOVING 1

#define REC_SUCCESS 3
#define REC_ING 1
#define REC_FAIL 0

namespace Engineering_robot_RM2025_Pnx {
namespace rvt = rviz_visual_tools;
using namespace std::placeholders;
using namespace std::chrono_literals;

struct PlayerCommandContent{
    rclcpp::Time command_time=rclcpp::Time(0,0);
    bool is_started=0;
    bool is_tuning_finish=0;
    bool is_finish=0;
    bool breakout=0;
};

struct ComputerState{
    uint8_t current_state;
    uint8_t recognition:2;
    uint8_t pos1_state:2;
    uint8_t pos2_state:2;
    uint8_t pos3_state:2;
};

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
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
const moveit::core::JointModelGroup* arm_model_group;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_;

// rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_trigger_;

tf2_ros::Buffer::SharedPtr tf2_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf2_listenser_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_pub_;

//receive a sign and control the arm move to target
// void planner_trigger_call_back(const std_msgs::msg::Bool::SharedPtr& msg);

// state_controll

// load RedeemBox and mine
bool LoadAttachMine();
bool LoadRedeemBox();
bool RemoveRedeemBox();

bool RemoveObject(const std::string & name);
bool disableObjectRobotCollision(const std::string& object_id, const std::vector<std::string>& robot_link_names);
bool disableObjectRobotCollision(const std::string& object_id, const std::string robot_link_name);
bool IsObjectInScene(const std::string& object_id);
void cancel_mine_exchange_pipe_thread_clear();

std::string MineMesh="package://engineering_robot_controller/meshes/Mine.STL";
std::string RedeemBoxMesh="package://engineering_robot_controller/meshes/RedeemBox.STL";
std::string RedeemBoxFram="object/fixedbox";
std::string robot_base="robot_base_link";
std::string end_link="end_link";

// exchange_state_controller

PlayerCommandContent player_command;
std::mutex player_command_mutex;

ComputerState computer_state;
std::mutex computer_state_mutex;

std::shared_ptr<rclcpp::Subscription<command_interfaces::msg::PlayerCommand> > player_command_sub_;
std::shared_ptr<rclcpp::Publisher<command_interfaces::msg::ComputerState> > computer_state_pub_;
// 以30hz的频率发布上位机状态
rclcpp::TimerBase::SharedPtr computer_state_pub_timer_;
std::shared_ptr<std::thread> commmand_executor_thread_;
rclcpp::Duration player_commmand_time_threshold=rclcpp::Duration(0,1e8);

// 0 no ok
// 1 ok
std::atomic<int> mine_exchange_pipe_state;

std::shared_ptr<rclcpp::TimerBase> RedeemBox_pos_pub_timer=nullptr;

void player_command_sub_callback(const command_interfaces::msg::PlayerCommand::ConstSharedPtr & msg);
PlayerCommandContent get_player_command();
ComputerState get_computer_state();
void set_player_command(const PlayerCommandContent & input_command);
void set_computer_state(const ComputerState & input_state);
void computer_state_pub_callback();

void commmand_executor();
void mine_exchange_pipe();
void clear_constraints_state();

};// Engineering_robot_Controller


std::vector<double> eulerToQuaternion(double roll, double pitch, double yaw);
std::vector<double> eulerToQuaternion(const std::vector<double>& euler);
std::vector<double> quaternionToEuler(const std::vector<double>& q);

void doPointTransform(
    const geometry_msgs::msg::Point &data_in,
    geometry_msgs::msg::Point &data_out,
    const geometry_msgs::msg::TransformStamped &transform
);

} // namespace motion_planning_api

#endif // Engineering_robot_RM2025_Pnx
