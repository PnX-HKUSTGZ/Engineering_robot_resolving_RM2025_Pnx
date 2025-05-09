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
        if(command.breakout){
            continue;
        }

        auto computer_state=get_computer_state();
        computer_state.current_state=1;
        computer_state.recognition=REC_ING;
        computer_state.pos1_state=0;
        computer_state.pos2_state=0;
        computer_state.pos3_state=0;
        set_computer_state(computer_state);

        RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe start!");

        std::thread mine_exchange_pipe_thread([this](){
            mine_exchange_pipe_state=0;
            clearPlanScene();
            this->mine_exchange_pipe();
            clearPlanScene();
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
                clearPlanScene();
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

    geometry_msgs::msg::TransformStamped msg=fix_RedeemBox_pos();

    bool LoadRedeemBoxcheck=LoadRedeemBox(msg);
    if(!LoadRedeemBoxcheck){
        RCLCPP_ERROR(this->get_logger(),"LoadRedeemBox failed! pipe end!");
        return;
    }
    bool LoadAttachMineCheck=LoadAttachMine();
    if(!LoadAttachMineCheck){
        RCLCPP_ERROR(this->get_logger(),"LoadAttachMine failed! pipe end!");
        return;
    }

    IsObjectInScene("Mine");
    RCLCPP_INFO(this->get_logger(),"6");

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

    IsObjectInScene("Mine");
    RCLCPP_INFO(this->get_logger(),"5");

{    //第一阶段 ====================================================================
    clear_constraints_state();
    computer_state.current_state=2;
    computer_state.pos1_state=PLANNING;
    set_computer_state(computer_state);

    moveit_msgs::msg::Constraints state1_constraints;
    state1_constraints.name="state one constraints";

    IsObjectInScene("Mine");
    RCLCPP_INFO(this->get_logger(),"3");

    geometry_msgs::msg::Point RedeemBoxstate1point;
    geometry_msgs::msg::Point transformedRedeemBoxstate1point;
    RedeemBoxstate1point.x=0;
    RedeemBoxstate1point.y=0;
    RedeemBoxstate1point.z=-0.210;
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
    
    move_group_->setPoseTarget(primitive_pose);
    move_group_->setGoalOrientationTolerance(minOrientationTolerance);
    move_group_->setGoalPositionTolerance(minPositionTolerance);
    move_group_->setPlanningTime(minPlanTime);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);

    bool success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    for(int i=1;i<=AllowRePlanAttempt&&(!success);i++){
        RCLCPP_WARN(this->get_logger(),"plan failed! try again!");
        clear_constraints_state();
        move_group_->setPoseTarget(primitive_pose);
        move_group_->setGoalOrientationTolerance(minOrientationTolerance+i*OrientationToleranceStep);
        move_group_->setGoalPositionTolerance(minPositionTolerance+i*PositionToleranceStep);
        move_group_->setPlanningTime(minPlanTime+i*PlanTimeStep);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
    }

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

    IsObjectInScene("Mine");
    RCLCPP_INFO(this->get_logger(),"2");

    visual_tools_->deleteAllMarkers();
    visual_tools_->publishTrajectoryLine(plan.trajectory_, arm_model_group);

    try{
        RCLCPP_INFO(this->get_logger(),"state one executing.....");
        move_group_->setMaxAccelerationScalingFactor(1);
        auto execute_res=move_group_->execute(plan);
        if(execute_res==moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO(this->get_logger(),"execute finish!");
        }
        else if(execute_res==moveit::core::MoveItErrorCode::TIMED_OUT){
            RCLCPP_WARN(this->get_logger(),"execute time out");
        }
        else{
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
    IsObjectInScene("Mine");
    RCLCPP_INFO(this->get_logger(),"1");
    RCLCPP_INFO(this->get_logger(),"state_one move OK!");
    computer_state.current_state=2;
    computer_state.pos1_state=FINISH;
    set_computer_state(computer_state);

}    //第一阶段完成 ====================================================================
