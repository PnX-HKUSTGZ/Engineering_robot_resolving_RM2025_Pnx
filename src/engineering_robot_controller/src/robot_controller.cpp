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

    // bool LoadRedeemBoxcheck=LoadRedeemBox();
    // if(!LoadRedeemBoxcheck){
    //     RCLCPP_ERROR(this->get_logger(),"LoadRedeemBoxcheck failed");
    //     return 0;
    // }
    // else RCLCPP_INFO(this->get_logger(),"LoadRedeemBox ok!");

    player_command_sub_=this->create_subscription<command_interfaces::msg::PlayerCommand>("/player_command",1,[this](const command_interfaces::msg::PlayerCommand::SharedPtr & msg){
        player_command_sub_callback(msg);
    });

    RCLCPP_INFO(this->get_logger(),"MoveitInit ok!");

    return 1;

}

void Engineering_robot_Controller::player_command_sub_callback(const command_interfaces::msg::PlayerCommand::SharedPtr & msg){
    PlayerCommandContent input_command;
    input_command.breakout=msg->breakout;
    input_command.is_finish=msg->is_finish;
    input_command.is_started=msg->is_started;
    input_command.is_tuning_finish=msg->is_tuning_finish;
    set_player_command(input_command);
}


PlayerCommandContent Engineering_robot_Controller::get_player_command(){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    return player_command;
}

ComputerState Engineering_robot_Controller::get_computer_state(){
    std::lock_guard<std::mutex> ul(computer_state_mutex);
    return computer_state;
}

void Engineering_robot_Controller::set_player_command(const PlayerCommandContent & input_command){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    player_command=input_command;
}

void Engineering_robot_Controller::set_computer_state(const ComputerState & input_state){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    computer_state=input_state;
}

bool Engineering_robot_Controller::LoadRedeemBox(){

    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.header.stamp=this->now();
    collision_object.header.frame_id=RedeemBoxFram;
    collision_object.id="RedeemBox";
    std::shared_ptr<shapes::Mesh> mesh_(shapes::createMeshFromResource(RedeemBoxMesh));

    if(!mesh_){
        RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", RedeemBoxMesh.c_str());
        RCLCPP_ERROR(this->get_logger(), "Object 'RedeemBox' will NOT be added to the scene.");
        return 0;
    }

    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_base;

    shapes::constructMsgFromShape(mesh_.get(), mesh_msg_base);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_base);

    if (mesh_msg.vertices.empty()){
        RCLCPP_ERROR(this->get_logger(), "111Failed to load mesh from: %s", RedeemBoxMesh.c_str());
        RCLCPP_ERROR(this->get_logger(), "Object 'RedeemBox' will NOT be added to the scene.");
        return 0;
    }

    geometry_msgs::msg::Pose object_pose_relative_to_tf;

    object_pose_relative_to_tf.position.x = 0.0;
    object_pose_relative_to_tf.position.y = 0.0;
    object_pose_relative_to_tf.position.z = 0.0;
    object_pose_relative_to_tf.orientation.x = 0.0;
    object_pose_relative_to_tf.orientation.y = 0.0;
    object_pose_relative_to_tf.orientation.z = 0.0;
    object_pose_relative_to_tf.orientation.w = 1.0; // Identity quaternion (no rotation)

    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(object_pose_relative_to_tf);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface_->addCollisionObjects(collision_objects);

    RCLCPP_INFO(this->get_logger(), "Collision object 'RedeemBox' added to the planning scene attached to frame '%s'.",
        RedeemBoxFram.c_str());

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
