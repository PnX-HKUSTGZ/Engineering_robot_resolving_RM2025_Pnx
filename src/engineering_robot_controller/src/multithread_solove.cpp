#include <engineering_robot_controller/engineering_robot_controller.hpp>

namespace Engineering_robot_RM2025_Pnx{


bool Engineering_robot_Controller::MultithreadedPlanne(
    const planning_interface::MotionPlanRequest& req, 
    planning_interface::MotionPlanResponse &res,
    int threadnum){

        std::vector<std::thread> threads;
        // 线程状态 
        /*
        0 未开始 
        1 正在运行 
        2 运行结束 
        3 运行出错
        */
        std::vector<std::atomic_int> thread_status(threadnum);
        for(int i=0;i<threadnum;i++){
            thread_status[i]=0;
        }
        std::vector<planning_interface::MotionPlanResponse> thread_res(threadnum);
        for(int i=0;i<threadnum;i++){
            threads.push_back(std::thread([i,&thread_res,&thread_status,req,this](){
                thread_status[i]=1;
                planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
                moveit::core::RobotState state=lscene->getCurrentState();

                RCLCPP_INFO(this->get_logger(),"--- Current Joint States ---");

                RCLCPP_INFO(this->get_logger(),"312312313123123");
                moveit_msgs::msg::RobotState msg;
                // RCLCPP_INFO_STREAM(this->get_logger(),state.)
                moveit::core::robotStateMsgToRobotState(msg,state);
                RCLCPP_INFO(this->get_logger(),"312312313123123");

                const std::vector<std::string>& joint_values = state.getVariableNames();

                for (const auto& name : joint_values){
                    RCLCPP_INFO_STREAM(this->get_logger(), name << ": " << *state.getJointPositions(name));
                }

                RCLCPP_INFO(this->get_logger(),"---------------------------");

                bool ok=0;
                try{
                    ok=planning_pipeline_->generatePlan(lscene, req, thread_res[i]);
                }
                catch(const std::exception& e){
                    RCLCPP_ERROR(this->get_logger(), "thread NO.%d Planning failed: %s", i,e.what());
                    thread_status[i]=3;
                    return;
                }
                if(ok){
                    thread_status[i]=2;
                    RCLCPP_INFO(this->get_logger(), "thread NO.%d Planning succeeded!", i);
                    return;
                }
                else{
                    thread_status[i]=3;
                    RCLCPP_INFO(this->get_logger(), "thread NO.%d Planning failed!", i);
                    return;
                }
            }));            
        }

        // 等待有线程算出正确答案，或者所有线程出错
        bool get_ans=0;
        while(!get_ans){
            std::this_thread::sleep_for(20ms);
            int failcount=0;
            for(int i=0;i<threadnum;i++){
                if(thread_status[i]==2){
                    res=thread_res[i];
                    get_ans=true;
                }
                if(thread_status[i]==3){
                    failcount++;
                }
            }
            if(failcount==threadnum){
                break;
            }
        }


        for(int i=0;i<threadnum;i++){
            pthread_cancel(threads[i].native_handle());
        }

        if(get_ans){
            RCLCPP_INFO(this->get_logger(), "MultithreadedPlanne succeeded!");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "MultithreadedPlanne failed!");
        }


        return get_ans;

    }


// bool Engineering_robot_Controller::MultithreadedPlanne(
//     geometry_msgs::msg::PoseStamped &target,
//     double orientation_tolerance,
//     double position_tolerance,

//     int threadnum,
//     const std::string &end_effector_link = "",
//     ){
//         std::vector<std::thread> threads;
//         // 线程状态
//         /*
//         0 未开始
//         1 正在运行
//         2 运行结束
//         3 运行出错
//         */
//         std::vector<std::atomic_int> thread_status(threadnum);
//         for(int i=0;i<threadnum;i++){
//             thread_status[i]=0;
//         }


//     }


bool Engineering_robot_Controller::DemonRun(){
    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(ARM_CONTROL_GROUP);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "home");
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::vector<std::string> planner_plugin_names;

    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        RCLCPP_FATAL(this->get_logger(), "Exception while creating planning plugin loader %s", ex.what());
    }

    const std::string planner_name="ompl_interface/OMPLPlanner";

    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
        if (!planner_instance->initialize(robot_model, this->shared_from_this(),
                                        this->get_namespace()))
        RCLCPP_FATAL(this->get_logger(), "Could not initialize planner instance");
        RCLCPP_INFO(this->get_logger(), "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        RCLCPP_ERROR(this->get_logger(), "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_name.c_str(),
                    ex.what(), ss.str().c_str());
    }

    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();  // clear all old markers
    visual_tools_->trigger();

    // visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    // visual_tools_->trigger();

    bool get_tranform=0;
    geometry_msgs::msg::TransformStamped msg;
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

    geometry_msgs::msg::Pose TargetPose;
    geometry_msgs::msg::PoseStamped TransformedTargetPose;
    TransformedTargetPose.header.frame_id=robot_base;
    TransformedTargetPose.header.stamp=this->now();
    TargetPose.position.x=0;
    TargetPose.position.y=0;
    TargetPose.position.z=0;
    TargetPose.orientation.x=0;
    TargetPose.orientation.y=0;
    TargetPose.orientation.z=-0.7071068;
    TargetPose.orientation.w=0.7071068;
    doPoseTransform(TargetPose,TransformedTargetPose.pose,msg);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints(end_link, TransformedTargetPose, tolerance_pose, tolerance_angle);

    req.group_name = ARM_CONTROL_GROUP;
    req.goal_constraints.push_back(pose_goal);

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
        req.workspace_parameters.min_corner.z = -5.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
        req.workspace_parameters.max_corner.z = 5.0;

    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
        return 0;
    }

    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
        this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",1);

    moveit_msgs::msg::DisplayTrajectory display_trajectory;

    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools_->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools_->trigger();
    display_publisher->publish(display_trajectory);

    RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", res.error_code_.val ? "" : "FAILED");

}




}