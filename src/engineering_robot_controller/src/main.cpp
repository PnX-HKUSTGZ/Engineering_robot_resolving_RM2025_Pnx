#include "engineering_robot_controller/engineering_robot_controller.hpp"

int main (int argc,char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Engineering_robot_RM2025_Pnx::Engineering_robot_Controller>();

    if(!node->MoveitInit()){
        RCLCPP_ERROR(node->get_logger(),"MoveitInit fail!");
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
}