#include "engineering_robot_controller/engineering_robot_controller.hpp"
namespace Engineering_robot_RM2025_Pnx{

Engineering_robot_Controller::Engineering_robot_Controller(rclcpp::NodeOptions node_options):
    Node("Engineering_robot_Controller",node_options){

    // load param
    if(!this->has_parameter("ARM_CONTROL_GROUP")){
        this->declare_parameter<std::string>("ARM_CONTROL_GROUP","body");
        RCLCPP_WARN(this->get_logger(),"ARM_CONTROL_GROUP dosen't declare use default val \"body\"");
    }
    if(!this->has_parameter("END_EFFECTOR_CONTROL_GROUP")){
        this->declare_parameter<std::string>("END_EFFECTOR_CONTROL_GROUP","hand");
        RCLCPP_WARN(this->get_logger(),"END_EFFECTOR_CONTROL_GROUP dosen't declare use default val \"hand\"");
    }
    ARM_CONTROL_GROUP=this->get_parameter("ARM_CONTROL_GROUP").as_string();
    END_EFFECTOR_CONTROL_GROUP=this->get_parameter("END_EFFECTOR_CONTROL_GROUP").as_string();

    RCLCPP_INFO(this->get_logger(),"Load param ok!");

    tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listenser_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);
    RCLCPP_INFO(this->get_logger(),"Load tf2 ok!");

    RCLCPP_INFO(this->get_logger(),"Load Engineering_robot_Controller ok!");
}

bool Engineering_robot_Controller::MoveitInit(){
    try{
        move_group_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(),ARM_CONTROL_GROUP);
        planning_scene_interface_=std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        arm_model_group=move_group_->getCurrentState()->getJointModelGroup(ARM_CONTROL_GROUP);
        visual_tools_=std::shared_ptr<moveit_visual_tools::MoveItVisualTools>(new moveit_visual_tools::MoveItVisualTools(this->shared_from_this(),"robot_base","",move_group_->getRobotModel()));
        plan_=std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan>();

        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"MoveitInit fail with %s",e.what());
        return 0;
    }

    RCLCPP_INFO(this->get_logger(),"Load Moveit2 Part ok!");

    planner_trigger_=this->create_subscription<std_msgs::msg::Bool>(
        "/engineering_robot_controller/tigger",
        1,
        [this](const std::shared_ptr<std_msgs::msg::Bool> msg){
            this->planner_trigger_call_back(msg);
        });
    
    RCLCPP_INFO(this->get_logger(),"Load sub Part ok!");

    RCLCPP_INFO(this->get_logger(),"MoveitInit ok!");

    return 1;

}

void Engineering_robot_Controller::planner_trigger_call_back(const std_msgs::msg::Bool::SharedPtr& msg){


    geometry_msgs::msg::TransformStamped box_pos;

    try{
        box_pos=tf2_buffer_->lookupTransform(
            "object/box",
            "robot_base",
            this->now(),
            20ms);
    }
    catch(const std::exception& e){
        RCLCPP_WARN(this->get_logger(),"lookupTransform fail with %s",e.what());
        return;
    }

    geometry_msgs::msg::Pose target;

    target.position.x=box_pos.transform.translation.x;
    target.position.y=box_pos.transform.translation.y;
    target.position.z=box_pos.transform.translation.z;
    target.orientation.w=1;
    //TODO: 这里等待一个确切的坐标系转化以确定具体写法。

    visual_tools_->publishText(target, "target_pose", rvt::WHITE, rvt::XLARGE);

    bool success = (move_group_->plan(*plan_) == moveit::core::MoveItErrorCode::SUCCESS);

    if(!success){
        RCLCPP_WARN(this->get_logger(),"MoveGroup plan failed!");
        return;
    }
    RCLCPP_INFO(this->get_logger(),"MoveGroup plan successfully!");

    moveit::core::MoveItErrorCode execute_state=move_group_->execute(plan_->trajectory_);

    if(execute_state==moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(this->get_logger(),"MoveGroup execute successfully!");
    }
    else{
        RCLCPP_WARN(this->get_logger(),"MoveGroup execute failed!");
    }
}

} // namespace Engineering_robot_RM2025_Pnx
