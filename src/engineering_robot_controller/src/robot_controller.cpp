#include "engineering_robot_controller/engineering_robot_controller.hpp"
namespace Engineering_robot_RM2025_Pnx{

Engineering_robot_Controller::Engineering_robot_Controller(rclcpp::NodeOptions node_options):
    Node("Engineering_robot_Controller",node_options){

    // load param
    if(!this->has_parameter("ARM_CONTROL_GROUP")){
        this->declare_parameter<std::string>("ARM_CONTROL_GROUP","arm");
        RCLCPP_WARN(this->get_logger(),"ARM_CONTROL_GROUP dosen't declare use default val \"arm\"");
    }
    if(!this->has_parameter("END_EFFECTOR_CONTROL_GROUP")){
        this->declare_parameter<std::string>("END_EFFECTOR_CONTROL_GROUP","sucker");
        RCLCPP_WARN(this->get_logger(),"END_EFFECTOR_CONTROL_GROUP dosen't declare use default val \"sucker\"");
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
        move_group_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(),ARM_CONTROL_GROUP, this->tf2_buffer_);
        RCLCPP_INFO(this->get_logger(),"move_group_ init ok! with group name: %s",ARM_CONTROL_GROUP.c_str());
        
        planning_scene_interface_=std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        RCLCPP_INFO(this->get_logger(),"PlanningSceneInterface initialized successfully");
        
        arm_model_group=move_group_->getCurrentState()->getJointModelGroup(ARM_CONTROL_GROUP);
        RCLCPP_INFO(this->get_logger(),"JointModelGroup for %s loaded successfully", ARM_CONTROL_GROUP.c_str());
        
        visual_tools_=std::shared_ptr<moveit_visual_tools::MoveItVisualTools>(new moveit_visual_tools::MoveItVisualTools(this->shared_from_this(),"robot_base","robot",move_group_->getRobotModel()));
        RCLCPP_INFO(this->get_logger(),"MoveItVisualTools initialized with reference frame: robot_base");
        
        plan_=std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan>();
        RCLCPP_INFO(this->get_logger(),"Motion plan container initialized");

        visual_tools_->deleteAllMarkers();
        RCLCPP_INFO(this->get_logger(),"All previous markers deleted");
        
        visual_tools_->loadRemoteControl();
        RCLCPP_INFO(this->get_logger(),"Remote control loaded for visual tools");
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

    RCLCPP_INFO_STREAM(this->get_logger(), "Reference frame: " << move_group_->getPoseReferenceFrame());

    RCLCPP_INFO(this->get_logger(),"MoveitInit ok!");

    return 1;

}

void Engineering_robot_Controller::planner_trigger_call_back(const std_msgs::msg::Bool::SharedPtr& msg){

    RCLCPP_INFO(this->get_logger(),"planner_trigger_call_back called");

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

    auto robot_state_=move_group_->getCurrentState();
    const auto names= robot_state_->getVariableNames();
    std::vector<double> vals;
    robot_state_->copyJointGroupPositions(arm_model_group,vals);

    RCLCPP_INFO_STREAM(this->get_logger(), "Reference frame: " << move_group_->getPoseReferenceFrame());
    vals[0]=-vals[0];

    geometry_msgs::msg::Pose target;

    target.position.x=box_pos.transform.translation.x;
    target.position.y=box_pos.transform.translation.y;
    target.position.z=box_pos.transform.translation.z;


}

} // namespace Engineering_robot_RM2025_Pnx
