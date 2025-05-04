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
    tf2_pub_=std::make_unique<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(),"Load tf2 ok!");

    auto computer_state=get_computer_state();
    computer_state.current_state=0;
    computer_state.recognition=0;
    computer_state.pos1_state=0;
    computer_state.pos2_state=0;
    computer_state.pos3_state=0;
    set_computer_state(computer_state);

    auto player_command=get_player_command();
    player_command.breakout=0;
    player_command.is_finish=0;
    player_command.is_started=0;
    player_command.is_tuning_finish=0;
    set_player_command(player_command);

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
    move_group_->setEndEffectorLink("J5_end");

    RCLCPP_INFO(this->get_logger(),"set the endeffector link : J5_end");
 
    RCLCPP_INFO(this->get_logger(),"Load Moveit2 Part ok!");

    planner_trigger_=this->create_subscription<std_msgs::msg::Bool>(
        "/engineering_robot_controller/tigger",
        1,
        [this](const std::shared_ptr<std_msgs::msg::Bool> msg){
            this->planner_trigger_call_back(msg);
        });
    
    RCLCPP_INFO(this->get_logger(),"Load sub Part ok!");

    RCLCPP_INFO_STREAM(this->get_logger(), "Reference frame: " << move_group_->getPoseReferenceFrame());

    bool LoadRedeemBoxcheck=LoadRedeemBox();
    if(!LoadRedeemBoxcheck){
        RCLCPP_ERROR(this->get_logger(),"LoadRedeemBoxcheck failed");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"LoadRedeemBox ok!");

    player_command_sub_=this->create_subscription<command_interfaces::msg::PlayerCommand>("/player_command",1,[this](const command_interfaces::msg::PlayerCommand::ConstSharedPtr & msg){
        player_command_sub_callback(msg);
    });
    computer_state_pub_=this->create_publisher<command_interfaces::msg::ComputerState>("/computer_state",1);

    computer_state_pub_timer_=this->create_wall_timer(33ms,[this](){
        this->computer_state_pub_callback();
    });

    commmand_executor_thread_=std::make_shared<std::thread>([this](){
        this->commmand_executor();
    });

    RCLCPP_INFO(this->get_logger(),"MoveitInit ok!");

    return 1;

}

void Engineering_robot_Controller::clear_constraints_state(){
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->clearTrajectoryConstraints();
}


void Engineering_robot_Controller::commmand_executor(){
    while(1){
        // 休息一下，交出CPU
        std::this_thread::sleep_for(33ms);
        auto command=get_player_command();
        // if(this->now()-command.command_time>player_commmand_time_threshold){
        //     RCLCPP_WARN_STREAM(this->get_logger(),"play_command time out! with time :["<<command.command_time.seconds()<<","<<command.command_time.nanoseconds()<<"]");
        //     continue;
        // }
        if(!command.is_started){
            continue;
        }

        RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe start!");

        std::thread mine_exchange_pipe_thread([this](){
            mine_exchange_pipe_state=0;
            this->mine_exchange_pipe();
            mine_exchange_pipe_state=1;
        });

        while(1){
            std::this_thread::sleep_for(33ms);
            auto now_command=get_player_command();
            // if(this->now()-now_command.command_time>player_commmand_time_threshold){
            //     RCLCPP_WARN_STREAM(this->get_logger(),"play_command time out! with time :["<<command.command_time.seconds()<<","<<command.command_time.nanoseconds()<<"]");
            //     continue;
            // }
            if(now_command.breakout){
                RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe stop by player");
                RCLCPP_INFO(this->get_logger(),"try to stop mine_exchange_pipe");
                pthread_cancel(mine_exchange_pipe_thread.native_handle());
                mine_exchange_pipe_thread.join();
                RCLCPP_INFO(this->get_logger(),"stop mine_exchange_pipe!");
                break;
            }
            if(mine_exchange_pipe_state){
                mine_exchange_pipe_thread.join();
                RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe finish!");
                break;
            }
        }
        
        auto computer_state=get_computer_state();
        computer_state.current_state=0;
        computer_state.recognition=0;
        computer_state.pos1_state=0;
        computer_state.pos2_state=0;
        computer_state.pos3_state=0;
        set_computer_state(computer_state);

        RCLCPP_INFO(this->get_logger(),"this command exec ok!");

    }
}

void Engineering_robot_Controller::mine_exchange_pipe(){

    auto computer_state=get_computer_state();

    //tf2初始化==============================================================
    computer_state.current_state=1;
    computer_state.recognition=REC_ING;
    set_computer_state(computer_state);

    geometry_msgs::msg::TransformStamped msg;

    bool get_tranform=0;
    while(!get_tranform){
        try{
            msg=tf2_buffer_->lookupTransform(
                "robot_base",
                "object/box",
                this->now(),
                50ms
            );
            get_tranform=1;
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(this->get_logger(),"look transform of RedeemBox fail with"<<e.what()<<", try again");
            get_tranform=0;
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"get transform of RedeemBox and robot_base");

    auto RedeemBox_pos_pub_timer=this->create_wall_timer(33ms,[this,msg](){
        geometry_msgs::msg::TransformStamped msg_=msg;
        msg_.child_frame_id="object/fixedbox";
        msg_.header.stamp=this->now();
        this->tf2_pub_->sendTransform(msg_);
    });

    //tf2 初始化完成==============================================================
    computer_state.current_state=1;
    computer_state.recognition=REC_SUCCESS;
    set_computer_state(computer_state);

    RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe tf2 init ok!");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::string ee_link=move_group_->getEndEffectorLink();
    std::string reference_frame=move_group_->getPlanningFrame();

    RCLCPP_INFO_STREAM(this->get_logger(),"EndEffector link name :"<<move_group_->getEndEffectorLink());
    RCLCPP_INFO_STREAM(this->get_logger(),"Planning frame name :"<<move_group_->getPlanningFrame());

{    //第一阶段 ====================================================================
    clear_constraints_state();
    if (!move_group_->setEndEffectorLink("J5_end")){
        RCLCPP_ERROR(this->get_logger(),"set fail!");
    }
    else RCLCPP_INFO(this->get_logger(),"set the endeffector link : J5_end");
    computer_state.current_state=2;
    computer_state.pos1_state=PLANNING;
    set_computer_state(computer_state);

    moveit_msgs::msg::Constraints state1_constraints;
    state1_constraints.name="state one constraints";

    geometry_msgs::msg::Point RedeemBoxstate1point;
    geometry_msgs::msg::Point transformedRedeemBoxstate1point;
    RedeemBoxstate1point.x=0;
    RedeemBoxstate1point.y=-0.25;
    RedeemBoxstate1point.z=0;
    doPointTransform(RedeemBoxstate1point,transformedRedeemBoxstate1point,msg);
    RCLCPP_INFO_STREAM(this->get_logger(),"target one pose ("<<transformedRedeemBoxstate1point.x<<","<<transformedRedeemBoxstate1point.y<<","<<transformedRedeemBoxstate1point.z<<")");


    moveit_msgs::msg::PositionConstraint pcon;
    pcon.header.frame_id=reference_frame;
    pcon.header.stamp=this->now();
    pcon.link_name=ee_link;
    pcon.target_point_offset.x=0;
    pcon.target_point_offset.y=0;
    pcon.target_point_offset.z=0;
    pcon.weight=1.0;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.005; // Tolerance in x (meters)
    primitive.dimensions[1] = 0.005; // Tolerance in y
    primitive.dimensions[2] = 0.005; // Tolerance in z  

    geometry_msgs::msg::Pose primitive_pose;
    // Set the target position (example values - replace with your desired position)
    primitive_pose.position.x = transformedRedeemBoxstate1point.x;
    primitive_pose.position.y = transformedRedeemBoxstate1point.y;
    primitive_pose.position.z = transformedRedeemBoxstate1point.z;
    // Orientation is usually identity for a box center
    primitive_pose.orientation=msg.transform.rotation;

    pcon.constraint_region.primitives.push_back(primitive);
    pcon.constraint_region.primitive_poses.push_back(primitive_pose);
    state1_constraints.position_constraints.push_back(pcon);

    moveit_msgs::msg::OrientationConstraint ocon;
    ocon.header.frame_id = reference_frame; // Constraint in the planning frame
    ocon.header.stamp = this->now();
    ocon.link_name = ee_link;
    ocon.weight = 1.0;

    ocon.orientation=msg.transform.rotation;

    ocon.absolute_x_axis_tolerance=0.01;
    ocon.absolute_y_axis_tolerance=M_PI;
    ocon.absolute_z_axis_tolerance=0.01;

    state1_constraints.orientation_constraints.push_back(ocon);

    // move_group_->setPathConstraints(state1_constraints);
    // move_group_->setGoalTolerance()
    move_group_->setPoseTarget(primitive_pose);
    move_group_->setGoalOrientationTolerance(0.1);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setPlanningTime(10);
    move_group_->setMaxVelocityScalingFactor(1.5);
    move_group_->setMaxAccelerationScalingFactor(1.5);

    bool success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    if(success){
        computer_state.current_state=2;
        computer_state.pos1_state=MOVING;
        set_computer_state(computer_state);
        RCLCPP_INFO(this->get_logger(),"state one constract ok!");
    }
    else{
        computer_state.current_state=2;
        computer_state.pos1_state=FAILED;
        set_computer_state(computer_state);
        RCLCPP_ERROR(this->get_logger(),"state one constract failed!");
        return;
    }

    visual_tools_->deleteAllMarkers();
    visual_tools_->publishTrajectoryLine(plan.trajectory_, arm_model_group);

    try{
        RCLCPP_INFO(this->get_logger(),"state one executing.....");
        auto execute_res=move_group_->execute(plan);
        if(execute_res!=moveit::core::MoveItErrorCode::SUCCESS){
            return;
        }
    }
    catch(const std::exception & e){
        RCLCPP_INFO(this->get_logger(),"state_one move failed! with %s",e.what());
        computer_state.current_state=2;
        computer_state.pos1_state=FAILED;
        set_computer_state(computer_state);
        return;
    }
    RCLCPP_INFO(this->get_logger(),"state_one move OK!");
    computer_state.current_state=2;
    computer_state.pos1_state=FINISH;
    set_computer_state(computer_state);

}    //第一阶段完成 ====================================================================


// 第二阶段等待指令。。。。。

    computer_state.current_state=3;
    set_computer_state(computer_state);

    RCLCPP_INFO(this->get_logger(),"waiting player turnning command");
    while(1){
        auto player_command=get_player_command();
        if(player_command.is_tuning_finish){
            RCLCPP_INFO(this->get_logger(),"get play command , turnning ok!");
            break;
        }
        std::this_thread::sleep_for(20ms);
    }

{    //第二阶段 ====================================================================
    clear_constraints_state();
    computer_state.current_state=4;
    computer_state.pos2_state=PLANNING;
    set_computer_state(computer_state);

    moveit_msgs::msg::Constraints state2_constraints;
    state2_constraints.name="state two constraints";

    geometry_msgs::msg::Point RedeemBoxstate2point;
    geometry_msgs::msg::Point transformedRedeemBoxstate2point;
    RedeemBoxstate2point.x=0;
    RedeemBoxstate2point.y=0;
    RedeemBoxstate2point.z=0;
    doPointTransform(RedeemBoxstate2point,transformedRedeemBoxstate2point,msg);

    moveit_msgs::msg::PositionConstraint pcon;
    pcon.header.frame_id=reference_frame;
    pcon.header.stamp=this->now();
    pcon.link_name=ee_link;
    pcon.target_point_offset.x=0;
    pcon.target_point_offset.y=-0.04;
    pcon.target_point_offset.z=0;
    pcon.weight=1.0;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.005; // Tolerance in x (meters)
    primitive.dimensions[1] = 0.005; // Tolerance in y
    primitive.dimensions[2] = 0.005; // Tolerance in z  

    geometry_msgs::msg::PoseStamped current_pose_stamped;
    try {
         current_pose_stamped = move_group_->getCurrentPose(ee_link);
         RCLCPP_INFO(this->get_logger(), "Retrieved current pose successfully.");
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current pose: %s", e.what());
        computer_state.current_state=4;
        computer_state.pos2_state=FAILED;
        set_computer_state(computer_state);
        return;
    }

    geometry_msgs::msg::Pose primitive_pose;
    // Set the target position (example values - replace with your desired position)
    primitive_pose.position.x = transformedRedeemBoxstate2point.x;
    primitive_pose.position.y = transformedRedeemBoxstate2point.y;
    primitive_pose.position.z = transformedRedeemBoxstate2point.z;
    // Orientation is usually identity for a box center
    primitive_pose.orientation=msg.transform.rotation;

    pcon.constraint_region.primitives.push_back(primitive);
    pcon.constraint_region.primitive_poses.push_back(primitive_pose);
    state2_constraints.position_constraints.push_back(pcon);

    moveit_msgs::msg::OrientationConstraint ocon;
    ocon.header.frame_id = reference_frame; // Constraint in the planning frame
    ocon.header.stamp = this->now();
    ocon.link_name = ee_link;
    ocon.weight = 1.0;

    ocon.orientation=current_pose_stamped.pose.orientation;

    state2_constraints.orientation_constraints.push_back(ocon);

    RCLCPP_INFO_STREAM(this->get_logger(),"target two pose ("<<transformedRedeemBoxstate2point.x<<","<<transformedRedeemBoxstate2point.y<<","<<transformedRedeemBoxstate2point.z<<")");

    // move_group_->setPathConstraints(state2_constraints);
    // move_group_->setGoalTolerance()
    move_group_->setPoseTarget(primitive_pose);
    move_group_->setGoalOrientationTolerance(0.5);
    move_group_->setGoalPositionTolerance(0.1);
    move_group_->setPlanningTime(10);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);

    bool success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    if(success){
        computer_state.current_state=4;
        computer_state.pos2_state=MOVING;
        set_computer_state(computer_state);
        RCLCPP_INFO(this->get_logger(),"second state constract ok!");
    }
    else{
        computer_state.current_state=4;
        computer_state.pos2_state=FAILED;
        set_computer_state(computer_state);
        RCLCPP_ERROR(this->get_logger(),"second state constract failed!");
        return;
    }

    try{
        RCLCPP_INFO(this->get_logger(),"state two executing.....");
        auto execute_res=move_group_->execute(plan);
        if(execute_res!=moveit::core::MoveItErrorCode::SUCCESS){
            return;
        }
    }
    catch(const std::exception & e){
        RCLCPP_INFO(this->get_logger(),"state_two move failed! with %s",e.what());
        computer_state.current_state=4;
        computer_state.pos2_state=FAILED;
        set_computer_state(computer_state);
        return;
    }
    RCLCPP_INFO(this->get_logger(),"state_two move OK!");
    computer_state.current_state=4;
    computer_state.pos2_state=FINISH;
    set_computer_state(computer_state);

}   //第二阶段完成 ====================================================================

// 等待 释放矿石

    computer_state.current_state=5;
    set_computer_state(computer_state);

    RCLCPP_INFO(this->get_logger(),"waiting player release ok command....");
    while(1){
        auto player_command=get_player_command();
        if(player_command.is_finish){
            RCLCPP_INFO(this->get_logger(),"get player command , release ok!");
            break;
        }
        std::this_thread::sleep_for(20ms);
    }

{//第三阶段 ====================================================================
    clear_constraints_state();
    computer_state.current_state=6;
    computer_state.pos3_state=PLANNING;
    set_computer_state(computer_state);

    move_group_->setNamedTarget("home");

    bool success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    if(success){
        computer_state.current_state=6;
        computer_state.pos3_state=MOVING;
        set_computer_state(computer_state);
        RCLCPP_INFO(this->get_logger(),"third state constract ok!");
    }
    else{
        computer_state.current_state=6;
        computer_state.pos3_state=FAILED;
        set_computer_state(computer_state);
        RCLCPP_ERROR(this->get_logger(),"third state constract failed!");
        return;
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        RCLCPP_INFO(this->get_logger(),"state_three move failed! with %s",e.what());
        computer_state.current_state=6;
        computer_state.pos3_state=FAILED;
        set_computer_state(computer_state);
        return;
    }
    RCLCPP_INFO(this->get_logger(),"state_three move OK!");
    computer_state.current_state=6;
    computer_state.pos3_state=FINISH;
    set_computer_state(computer_state);

}//第三阶段结束 ====================================================================

    RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe finish!");

}

void Engineering_robot_Controller::computer_state_pub_callback(){
    command_interfaces::msg::ComputerState msg;
    auto current_state=get_computer_state();
    msg.header.frame_id="/computer";
    msg.header.stamp=this->now();
    msg.current_state=current_state.current_state;
    msg.pos1_state=current_state.pos1_state;
    msg.pos2_state=current_state.pos2_state;
    msg.pos3_state=current_state.pos3_state;
    computer_state_pub_->publish(msg);
}

void Engineering_robot_Controller::player_command_sub_callback(const command_interfaces::msg::PlayerCommand::ConstSharedPtr & msg){
    PlayerCommandContent input_command;
    input_command.command_time=msg->header.stamp;
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

    object_pose_relative_to_tf.position.x = -0.144;
    object_pose_relative_to_tf.position.y = 0;
    object_pose_relative_to_tf.position.z = 0.144;
    object_pose_relative_to_tf.orientation.x = -0.7071068;
    object_pose_relative_to_tf.orientation.y = 0.0;
    object_pose_relative_to_tf.orientation.z = 0.0;
    object_pose_relative_to_tf.orientation.w = 0.7071068;

    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(object_pose_relative_to_tf);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::ObjectColor object_color;
    object_color.id=collision_object.id;

    object_color.color.r=0.0;
    object_color.color.g=0.3;
    object_color.color.b=0.4;
    object_color.color.a=1.0;

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff=true;
    planning_scene.object_colors.push_back(object_color);

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    planning_scene_interface_->applyPlanningScene(planning_scene);

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
