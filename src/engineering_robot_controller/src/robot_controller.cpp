#include "engineering_robot_controller/engineering_robot_controller.hpp"
namespace Engineering_robot_RM2025_Pnx{

Engineering_robot_Controller::Engineering_robot_Controller(rclcpp::NodeOptions node_options):
    Node("Engineering_robot_Controller",node_options){

    // load param
    LoadParam();

    RCLCPP_INFO(this->get_logger(),"Load param ok!");

    tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listenser_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);
    tf2_pub_=std::make_unique<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(),"Load tf2 ok!");

    set_computer_state(STATE_WAIT,STATE_WAIT);
    mine_exchange_pipe_state=PIPE_THREAD_NOLAUNCH;

    auto player_command=get_player_command();
    player_command.breakout=0;
    player_command.is_finish=0;
    player_command.is_started=0;
    set_player_command(player_command);



    RCLCPP_INFO(this->get_logger(),"Load Engineering_robot_Controller ok!");
}

void Engineering_robot_Controller::LoadParam(){
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

    if(!this->has_parameter("minOrientationTolerance")){
        this->declare_parameter<double>("minOrientationTolerance",1);
        RCLCPP_WARN(this->get_logger(),"minOrientationTolerance dosen't declare, use default val 0.1");
    }
    if(!this->has_parameter("minPositionTolerance")){
        this->declare_parameter<double>("minPositionTolerance",0.002);
        RCLCPP_WARN(this->get_logger(),"minPositionTolerance dosen't declare, use default val 0.002");
    }
    if(!this->has_parameter("maxOrientationTolerance")){
        this->declare_parameter<double>("maxOrientationTolerance",1.01);
        RCLCPP_WARN(this->get_logger(),"maxOrientationTolerance dosen't declare, use default val 0.5");
    }
    if(!this->has_parameter("maxPositionTolerance")){
        this->declare_parameter<double>("maxPositionTolerance",0.005);
        RCLCPP_WARN(this->get_logger(),"maxPositionTolerance dosen't declare, use default val 0.005");
    }
    if(!this->has_parameter("AllowPlanAttempt")){
        this->declare_parameter<int>("AllowPlanAttempt",3);
        RCLCPP_WARN(this->get_logger(),"AllowPlanAttempt dosen't declare, use default val 3");
    }
    if(!this->has_parameter("minPlanTime")){
        this->declare_parameter<int>("minPlanTime",2);
        RCLCPP_WARN(this->get_logger(),"minPlanTime dosen't declare, use default val 2");
    }
    if(!this->has_parameter("maxPlanTime")){
        this->declare_parameter<int>("maxPlanTime",3.5);
        RCLCPP_WARN(this->get_logger(),"maxPlanTime dosen't declare, use default val 3.5");
    }
    if(!this->has_parameter("AllowPlanAttempt")){
        this->declare_parameter<int>("AllowPlanAttempt",3);
        RCLCPP_WARN(this->get_logger(),"AllowPlanAttempt dosen't declare, use default val 3");
    }

    minOrientationTolerance=this->get_parameter("minOrientationTolerance").as_double();
    minPositionTolerance=this->get_parameter("minPositionTolerance").as_double();
    maxOrientationTolerance=this->get_parameter("maxOrientationTolerance").as_double();
    maxPositionTolerance=this->get_parameter("maxPositionTolerance").as_double();
    AllowPlanAttempt=this->get_parameter("AllowPlanAttempt").as_int();
    minPlanTime=this->get_parameter("minPlanTime").as_int();
    maxPlanTime=this->get_parameter("maxPlanTime").as_int();
    AllowPlanAttempt=this->get_parameter("AllowPlanAttempt").as_int();
    
    if(AllowPlanAttempt<1){
        RCLCPP_ERROR(this->get_logger(),"AllowPlanAttempt must be greater than 0, use default val 3");
        AllowPlanAttempt=3;
    }
    if(AllowPlanAttempt-1){
        OrientationToleranceStep=(maxOrientationTolerance-minOrientationTolerance)/(AllowPlanAttempt-1);
        PositionToleranceStep=(maxPositionTolerance-minPositionTolerance)/(AllowPlanAttempt-1);
        PlanTimeStep=(maxPlanTime-minPlanTime)/(AllowPlanAttempt-1);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameter Values ---");
    RCLCPP_INFO_STREAM(this->get_logger(), "minOrientationTolerance: " << minOrientationTolerance);
    RCLCPP_INFO_STREAM(this->get_logger(), "minPositionTolerance: " << minPositionTolerance);
    RCLCPP_INFO_STREAM(this->get_logger(), "maxOrientationTolerance: " << maxOrientationTolerance);
    RCLCPP_INFO_STREAM(this->get_logger(), "maxPositionTolerance: " << maxPositionTolerance);
    RCLCPP_INFO_STREAM(this->get_logger(), "AllowPlanAttempt: " << AllowPlanAttempt);
    RCLCPP_INFO_STREAM(this->get_logger(), "minPlanTime: " << minPlanTime);
    RCLCPP_INFO_STREAM(this->get_logger(), "maxPlanTime: " << maxPlanTime);
    RCLCPP_INFO_STREAM(this->get_logger(), "------------------------");

}

bool Engineering_robot_Controller::MoveitInit(){
    try{
        move_group_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(),ARM_CONTROL_GROUP, this->tf2_buffer_);
        RCLCPP_INFO(this->get_logger(),"move_group_ init ok! with group name: %s",ARM_CONTROL_GROUP.c_str());
        
        planning_scene_interface_=std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        RCLCPP_INFO(this->get_logger(),"PlanningSceneInterface initialized successfully");
        
        planning_scene_monitor_=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->shared_from_this(),"robot_description"
        );

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();
        RCLCPP_INFO(this->get_logger(),"planning_scene_monitor_ init finish");

        arm_model_group=move_group_->getCurrentState()->getJointModelGroup(ARM_CONTROL_GROUP);
        RCLCPP_INFO(this->get_logger(),"JointModelGroup for %s loaded successfully", ARM_CONTROL_GROUP.c_str());
        
        visual_tools_=std::shared_ptr<moveit_visual_tools::MoveItVisualTools>(new moveit_visual_tools::MoveItVisualTools(this->shared_from_this(),robot_base,"robot",move_group_->getRobotModel()));
        RCLCPP_INFO(this->get_logger(),"MoveItVisualTools initialized with reference frame: %s",robot_base.c_str());
        
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

    if(move_group_->setEndEffectorLink(end_link)){
        RCLCPP_INFO(this->get_logger(),"set the endeffector link : %s",end_link.c_str());
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"fail to set the endeffector link : %s",end_link.c_str());
        return 0;
    }

 
    RCLCPP_INFO(this->get_logger(),"Load Moveit2 Part ok!");
    
    RCLCPP_INFO(this->get_logger(),"Load sub Part ok!");

    RCLCPP_INFO_STREAM(this->get_logger(), "Reference frame: " << move_group_->getPoseReferenceFrame());

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

    regonition_update_thread_=std::make_shared<std::thread>([this](){
        while(rclcpp::ok()){
            this->regonition_update();
            std::this_thread::sleep_for(10ms);
        }
    });

    RCLCPP_INFO(this->get_logger(),"MoveitInit ok!");


    robot_get_min_sub_=this->create_subscription<std_msgs::msg::Bool>("/robot_trigger/get_mine",10,[this](const std_msgs::msg::Bool::ConstSharedPtr & msg){
        (void)msg;
        if(this->robot_go_pose("get_mine")){
            RCLCPP_INFO(this->get_logger(),"robot_trigger get_mine ok!");
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"robot_trigger get_mine fail!");
        }
    });
    robot_go_home_sub_=this->create_subscription<std_msgs::msg::Bool>("/robot_trigger/go_home",10,[this](const std_msgs::msg::Bool::ConstSharedPtr & msg){
        (void)msg;
        if(this->robot_go_pose("home")){
            RCLCPP_INFO(this->get_logger(),"robot_trigger go_home ok!");
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"robot_trigger go_home fail!");
        }
    });

    robot_auto_exchange_sub_=this->create_subscription<std_msgs::msg::Bool>("/robot_trigger/auto_exchange",10,[this](const std_msgs::msg::Bool::ConstSharedPtr & msg){
        (void)msg;
        if(this->AutoExchangeMine()){
            RCLCPP_INFO(this->get_logger(),"robot_trigger auto_exchange ok!");
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"robot_trigger auto_exchange fail!");
        }
    });

    robot_clear_scense_sub_=this->create_subscription<std_msgs::msg::Bool>("/robot_trigger/clear_scense",10,[this](const std_msgs::msg::Bool::ConstSharedPtr & msg){
        (void)msg;
        this->clearPlanScene();
        RCLCPP_INFO(this->get_logger(),"robot_trigger clear_scense ok!");
    });
    return 1;

}

void Engineering_robot_Controller::clear_constraints_state(){
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->clearTrajectoryConstraints();
}

void Engineering_robot_Controller::clearPlanScene(){
    RemoveRedeemBox();
    try{
        move_group_->detachObject("Mine");
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_logger(),"Detach Mine error with %s",e.what());
    }
    RemoveObject("Mine");
    unfix_RedeemBox_pos();
    RCLCPP_INFO(this->get_logger(),"RedeemBox_pos_pub_timer cancel and set to nullptr");
}

bool Engineering_robot_Controller::RemoveRedeemBox(){
    return RemoveObject("RedeemBox");
}

bool Engineering_robot_Controller::RemoveObject(const std::string & name){
    RCLCPP_INFO(this->get_logger(),"Attempting to remove collision object '%s' from planning scene.",name.c_str());

    if(!IsObjectInScene(name)){
        RCLCPP_INFO(this->get_logger(),"collision object '%s' doesn't exist",name.c_str());
        return 1;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"collision object '%s' exist, remove!",name.c_str());
    }

    std::vector<std::string> object_id;
    object_id.push_back(name);

    planning_scene_interface_->removeCollisionObjects(object_id);

    RCLCPP_INFO(this->get_logger(),"Send remove request ok! waiting to check ok");

    bool ok=0;
    int attempt_time=0;
    while((ok=IsObjectInScene(name))){
        RCLCPP_INFO(this->get_logger(),"waiting remove %s...",name.c_str());
        attempt_time++;
        std::this_thread::sleep_for(20ms);
        if(attempt_time>=50) break;
    }

    if(ok){
        RCLCPP_ERROR(this->get_logger(),"remove %s time out",name.c_str());
        return 0;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"remove %s ok!",name.c_str());
        return 1;
    }
}


bool Engineering_robot_Controller::IsObjectInScene(const std::string& object_id) {
    // 检查 planning_scene_interface_ 是否已初始化
    if (!planning_scene_interface_) {
        RCLCPP_ERROR(this->get_logger(), "PlanningSceneInterface is not initialized. Cannot check for object.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Checking if object '%s' is in the planning scene...", object_id.c_str());

    // 1. 获取规划场景中所有已知碰撞体的ID列表
    std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();


    RCLCPP_INFO(this->get_logger(),"object_ids list:");
    for(auto & i : object_ids){
        RCLCPP_INFO(this->get_logger(),"%s",i.c_str());
    }

    // 2. 在获取的ID列表中查找目标对象的ID
    auto it = std::find(object_ids.begin(), object_ids.end(), object_id);

    // 3. 判断是否找到
    bool found = (it != object_ids.end());

    if (found) {
        RCLCPP_INFO(this->get_logger(), "Object '%s' found in the planning scene.", object_id.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Object '%s' NOT found in the planning scene.", object_id.c_str());
    }

    return found;
}

bool Engineering_robot_Controller::LoadAttachMine(){

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.stamp=this->now();
    collision_object.header.frame_id=move_group_->getEndEffectorLink();
    collision_object.id="Mine";

    RCLCPP_INFO_STREAM(this->get_logger(),"load Mine as id: "<<collision_object.id<<", attach to "<<collision_object.header.frame_id);

    std::shared_ptr<shapes::Mesh> mesh_(shapes::createMeshFromResource(MineMesh));

    if(!mesh_){
        RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s at first step", MineMesh.c_str());
        RCLCPP_ERROR(this->get_logger(), "Object 'Mine' will NOT be added to the scene.");
        return 0;
    }

    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_base;

    shapes::constructMsgFromShape(mesh_.get(), mesh_msg_base);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_base);

    if (mesh_msg.vertices.empty()){
        RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s at second step", RedeemBoxMesh.c_str());
        RCLCPP_ERROR(this->get_logger(), "Object 'Mine' will NOT be added to the scene.");
        return 0;
    }

    geometry_msgs::msg::Pose object_pose_relative_to_tf;

    object_pose_relative_to_tf.position.x = 0.1;
    object_pose_relative_to_tf.position.y = -0.1;
    object_pose_relative_to_tf.position.z = 0.0;
    object_pose_relative_to_tf.orientation.x = 0;
    object_pose_relative_to_tf.orientation.y = 0.0;
    object_pose_relative_to_tf.orientation.z = 0.7071068;
    object_pose_relative_to_tf.orientation.w = 0.7071068;

    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(object_pose_relative_to_tf);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface_->addCollisionObjects(collision_objects);

    bool ok=0;
    int attempt_time=0;
    while(!(ok=IsObjectInScene("Mine"))){
        RCLCPP_INFO(this->get_logger(),"waiting load Mine...");
        attempt_time++;
        std::this_thread::sleep_for(20ms);
        if(attempt_time>=50) break;
    }
    if(!ok){
        RCLCPP_ERROR(this->get_logger(),"load Mine time out");
        return 0;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"load Mine ok!");
    }

    bool disablecheck=disableObjectRobotCollision(collision_object.id,collision_object.header.frame_id);
    if(disablecheck){
        RCLCPP_INFO(this->get_logger(), "disableObjectRobotCollision %s , %s ok!",collision_object.id.c_str(), collision_object.header.frame_id.c_str());
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "disableObjectRobotCollision %s , %s failed!",collision_object.id.c_str(), collision_object.header.frame_id.c_str());
        return 0; 
    }

    bool attachObjectCheck=move_group_->attachObject(collision_object.id,collision_object.header.frame_id);
    if(attachObjectCheck){
        RCLCPP_INFO(this->get_logger(), "attachObject %s , %s ok!",collision_object.id.c_str(), collision_object.header.frame_id.c_str());
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "attachObjectCheck %s , %s failed!",collision_object.id.c_str(), collision_object.header.frame_id.c_str());
        return 0; 
    }

    return 1;

}

bool Engineering_robot_Controller::LoadRedeemBox(geometry_msgs::msg::TransformStamped msg){
    static double eps=1e-9;
    while(1){
        auto getmsg=tf2_buffer_->lookupTransform(msg.header.frame_id,msg.child_frame_id,this->now(),1ms);
        if(
            std::abs(getmsg.transform.rotation.w-msg.transform.rotation.w)<=eps&&
            std::abs(getmsg.transform.rotation.x-msg.transform.rotation.x)<=eps&&
            std::abs(getmsg.transform.rotation.y-msg.transform.rotation.y)<=eps&&
            std::abs(getmsg.transform.rotation.z-msg.transform.rotation.z)<=eps&&
            std::abs(getmsg.transform.translation.x-msg.transform.translation.x)<=eps&&
            std::abs(getmsg.transform.translation.y-msg.transform.translation.y)<=eps&&
            std::abs(getmsg.transform.translation.z-msg.transform.translation.z)<=eps
        ){
            RCLCPP_INFO(this->get_logger(),"newest tramsform load! start load model!");
            break;
        }
        else{
            RCLCPP_WARN(this->get_logger(),"newest tramsform not load, waiting...");
        }
    }
    return LoadRedeemBox();
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
    object_pose_relative_to_tf.position.y = -0.144;
    object_pose_relative_to_tf.position.z = 0;
    object_pose_relative_to_tf.orientation.x = 0;
    object_pose_relative_to_tf.orientation.y = 0.0;
    object_pose_relative_to_tf.orientation.z = 0.0;
    object_pose_relative_to_tf.orientation.w = 1;

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

    bool ok=0;
    int attempt_time=0;
    while(!(ok=IsObjectInScene("RedeemBox"))){
        RCLCPP_INFO(this->get_logger(),"waiting load RedeemBox...");
        attempt_time++;
        std::this_thread::sleep_for(20ms);
        if(attempt_time>=50) break;
    }
    if(!ok){
        RCLCPP_ERROR(this->get_logger(),"load RedeemBox time out");
        return 0;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"load RedeemBox ok!");
        return 1;
    }

    return 1;

}

bool Engineering_robot_Controller::disableObjectRobotCollision(const std::string& object_id, const std::string robot_link_name){
    return disableObjectRobotCollision(object_id,std::vector<std::string>{robot_link_name});
}

/**
 * @brief 控制 MoveIt 规划场景中两个指定物体之间是否允许碰撞检测。
 *
 * @param planning_scene_interface 对 PlanningSceneInterface 对象的引用。
 * @param name1 第一个物体的名称（或机器人连杆名称）。
 * @param name2 第二个物体的名称（或机器人连杆名称）。
 * @param enable_collision 如果为 true，则允许碰撞（即恢复碰撞检测）；如果为 false，则禁用碰撞（即忽略碰撞检测）。
 * @return bool 如果成功应用了规划场景更新，则返回 true，否则返回 false。
 */
bool Engineering_robot_Controller::setCollisionsBetween(const std::string& name1, const std::string& name2, bool enable_collision){
    if (name1.empty() || name2.empty()){
        RCLCPP_INFO(this->get_logger(),"Cannot set collision status with empty object names.");
        return false;
    }
    // if(!(IsObjectInScene(name1)&&IsObjectInScene(name2))){
    //     RCLCPP_INFO(this->get_logger(),"object name don't exist");
    //     return false;
    // }
    if (name1 == name2){
        RCLCPP_WARN_STREAM(this->get_logger(),"Attempted to set collision status between object '" << name1 << "' and itself. This usually doesn't have an effect or is not needed.");
        return true; // Consider this a successful no-op
    }

    RCLCPP_INFO_STREAM(this->get_logger(),"Attempting to " << (enable_collision ? "enable" : "disable") << " collision detection between '" << name1 << "' and '" << name2 << "'");

// 创建一个 PlanningScene 消息对象，用于描述要应用的更改
    moveit_msgs::msg::PlanningScene planning_scene;
    moveit_msgs::msg::AllowedCollisionEntry msg;
    msg.enabled.push_back(enable_collision);

// 标记这是一个增量更新，只修改 AllowedCollisionMatrix
    planning_scene.is_diff = true;

    planning_scene.allowed_collision_matrix.entry_names.push_back(name1);
    planning_scene.allowed_collision_matrix.entry_names.push_back(name2);
    planning_scene.allowed_collision_matrix.entry_values.push_back(msg);
    planning_scene.allowed_collision_matrix.entry_values.push_back(msg);

// 应用修改后的规划场景
    bool success = planning_scene_interface_->applyPlanningScene(planning_scene);

    if (success){
        RCLCPP_INFO_STREAM(this->get_logger(),"Successfully " << (enable_collision ? "enabled" : "disabled") << " collision detection between '" << name1 << "' and '" << name2 << "'.");
    }
    else{
        RCLCPP_ERROR_STREAM(this->get_logger(),"Failed to " << (enable_collision ? "enable" : "disable") << " collision detection between '" << name1 << "' and '" << name2 << "'. Check MoveIt nodes.");
    }

    return success;
}

void Engineering_robot_Controller::regonition_update(){
    geometry_msgs::msg::TransformStamped msg;
    try{
        msg=tf2_buffer_->lookupTransform(
            robot_base,
            "object/box",
            this->now(),
            20ms
        );
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(this->get_logger(),"look transform of RedeemBox fail with"<<e.what()<<", try again");
        set_regonition_state(STATE_ONE,REC_FAIL);
    }
    set_regonition_state(STATE_ONE,REC_SUCCESS);
}

bool Engineering_robot_Controller::disableObjectRobotCollision(const std::string& object_id, const std::vector<std::string>& robot_link_names){
//     // Get the planning scene. Lock required if using a monitor.
//     planning_scene_monitor_->requestPlanningSceneState(); // Update scene from ROS
//     planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();

//     if (!planning_scene){
//         RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene.");
//         return 0;
//     }
// {// lock PlanningScene
//     // RW : read and write
//     planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);

//     // Get the non-const AllowedCollisionMatrix
//     collision_detection::AllowedCollisionMatrix& acm = ps->getAllowedCollisionMatrixNonConst();

//     // Disable collision for each specified robot link with the object
//     for (const std::string& link_name : robot_link_names){
//         // Check if the link exists in the robot model (optional but good practice)
//         if (ps->getRobotModel()->hasLinkModel(link_name)){
//             RCLCPP_INFO(this->get_logger(), "Allowing collision between '%s' and '%s'", link_name.c_str(), object_id.c_str());
//             // Set the entry in the ACM to true (allow collision)
//             acm.setEntry(link_name, object_id, true);
//         }
//         else{
//             RCLCPP_WARN(this->get_logger(), "Robot link '%s' not found.", link_name.c_str());
//         }
//     }
// }
    return 1;
//     // planning_scene_monitor_->publishCurrentPlanningScene(); // Or planning_scene->publishPlanningSceneMsg()
    // planning_scene_interface_->applyPlanningScene
}

} // namespace Engineering_robot_RM2025_Pnx
