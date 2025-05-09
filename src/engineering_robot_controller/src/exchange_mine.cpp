#include "engineering_robot_controller/engineering_robot_controller.hpp"

namespace Engineering_robot_RM2025_Pnx{

bool Engineering_robot_Controller::robot_go_pose(const std::string & name){
    clear_constraints_state();
    auto names=move_group_->getNamedTargets();
    if(std::find(names.begin(),names.end(),name)==names.end()){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose fail! no such target!");
        return 0;
    }

    move_group_->setNamedTarget(name);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success=0;
    for(int i=0;i<AllowPlanAttempt;i++){
        RCLCPP_INFO(this->get_logger(),"robot_go_pose try %d",i);
        success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            RCLCPP_INFO(this->get_logger(),"robot_go_pose plan success!");
            break;
        }
        else{
            RCLCPP_INFO(this->get_logger(),"robot_go_pose plan fail!");
        }
    }

    if(!success){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose fail!");
        return 0;
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        RCLCPP_INFO(this->get_logger(),"robot_go_pose move failed! with %s",e.what());
        return 0;
    }

    return 1;
}

void Engineering_robot_Controller::unfix_RedeemBox_pos(){
    if(RedeemBox_pos_pub_timer!=nullptr){
        RCLCPP_INFO(this->get_logger(),"unfixed RedeemBox pos!");
        RedeemBox_pos_pub_timer->cancel();
        RedeemBox_pos_pub_timer=nullptr;
    }
}

geometry_msgs::msg::TransformStamped Engineering_robot_Controller::fix_RedeemBox_pos(){
    if(RedeemBox_pos_pub_timer!=nullptr){
        RCLCPP_WARN(this->get_logger(),"RedeemBox pos already fixed!, will unfixed and fix again");
        unfix_RedeemBox_pos();
    }

    geometry_msgs::msg::TransformStamped msg;

    bool get_tranform=0;
    std::this_thread::sleep_for(60ms);
    while(!get_tranform){
        try{
            msg=tf2_buffer_->lookupTransform(
                robot_base,
                "object/box",
                this->now(),
                20ns
            );
            get_tranform=1;
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(this->get_logger(),"look transform of RedeemBox fail with"<<e.what()<<", try again");
            get_tranform=0;
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"get transform of RedeemBox and "<<robot_base);

    RedeemBox_pos_pub_timer=this->create_wall_timer(10ms,[this,msg](){
        geometry_msgs::msg::TransformStamped msg_=msg;
        msg_.child_frame_id="object/fixedbox";
        msg_.header.stamp=this->now();
        this->tf2_pub_->sendTransform(msg_);
    });
    RCLCPP_INFO(this->get_logger(),"create RedeemBox_pos_pub_timer!");

    return msg;

}

bool Engineering_robot_Controller::AutoExchangeMine(){

    clearPlanScene();

    if(!robot_go_pose("get_mine")){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose home fail!");
        return 0;
    }

    // wait for robot get mine

    auto msg=fix_RedeemBox_pos();
    bool LoadAttachMinecheck=LoadAttachMine();
    if(!LoadAttachMinecheck){
        RCLCPP_ERROR(this->get_logger(),"LoadAttachMine fail!");
        return 0;
    }
    bool LoadRedeemBoxcheck=LoadRedeemBox(msg);
    if(!LoadRedeemBoxcheck){
        RCLCPP_ERROR(this->get_logger(),"LoadRedeemBox fail!");
        return 0;
    }

    RCLCPP_INFO(this->get_logger(),"LoadAttachMine and LoadRedeemBox success!");

    geometry_msgs::msg::Pose TargetPose;
    geometry_msgs::msg::Pose TransformedTargetPose;
    TargetPose.position.x=0;
    TargetPose.position.y=0;
    TargetPose.position.z=-0.15;
    TargetPose.orientation.x=0;
    TargetPose.orientation.y=0;
    TargetPose.orientation.z=-0.7071068;
    TargetPose.orientation.w=0.7071068;
    doPoseTransform(TargetPose,TransformedTargetPose,msg);

    bool success=0;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    clear_constraints_state();
    move_group_->setPoseTarget(TransformedTargetPose);
    move_group_->setGoalOrientationTolerance(1);
    move_group_->setGoalPositionTolerance(0.005);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
    move_group_->setPlanningTime(2.5);

    for(int i=0;i<=AllowPlanAttempt;i++){
        move_group_->setGoalOrientationTolerance(0.3+i*0.05);
        RCLCPP_INFO(this->get_logger(),"robot_go_pose try %d, %lf",i,0.3+i*0.05);
        success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            break;
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"robot_go_pose fail! try again");
        }
    }

    if(!success){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose fail!");
        return 0;
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        RCLCPP_INFO(this->get_logger(),"robot_go_pose move failed! with %s",e.what());
        return 0;
    }
    RCLCPP_INFO(this->get_logger(),"robot_go_pose success!");

    // clearPlanScene();

    return 1;
}

}