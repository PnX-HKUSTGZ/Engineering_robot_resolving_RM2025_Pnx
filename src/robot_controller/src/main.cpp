#include "robot_controller/robot_controller.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<Engineering_robot_RM2025_Pnx::MoveItPlanningNode>(
        "robot_controller", node_options);
    
    // 初始化ROS和MoveIt
    if (!node->initializeMoveIt()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize node");
        return -1;
    }
    
    // 创建执行器
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // std::thread([&executor]() { executor.spin(); }).detach();

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}