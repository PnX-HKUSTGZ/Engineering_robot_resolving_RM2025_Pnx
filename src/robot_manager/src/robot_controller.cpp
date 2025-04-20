#include "robot_manager/robot_manager.hpp"
namespace Engineering_robot_RM2025_Pnx{

    MoveItPlanningNode::MoveItPlanningNode(const std::string &node_name, const rclcpp::NodeOptions &options)
        : Node(node_name, options){

        //load param

        if(!this->has_parameter("PLANNING_GROUP")){
            this->declare_parameter<std::string>("PLANNING_GROUP","body");
        }
        if(!this->has_parameter("EndEffectorJoint")){
            this->declare_parameter<std::string>("EndEffectorJoint","j6");
        }

        robot_description=this->get_parameter("my_robot_description").as_string();
        PLANNING_GROUP=this->get_parameter("PLANNING_GROUP").as_string();
        EndEffectorJoint=this->get_parameter("EndEffectorJoint").as_string();
        RCLCPP_INFO(this->get_logger(), "Declared parameter: robot_description=%s", robot_description.c_str());
        RCLCPP_INFO(this->get_logger(), "Declared parameter: PLANNING_GROUP=%s", PLANNING_GROUP.c_str());
        RCLCPP_INFO(this->get_logger(), "Declared parameter: EndEffectorJoint=%s", EndEffectorJoint.c_str());
        RCLCPP_INFO(this->get_logger(), "ROS param Load successfully");

        //tf2
        tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listenser_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);
        RCLCPP_INFO(this->get_logger(), "ROS tf2 part Load successfully");

        //sub
        trigger_sub_=this->create_subscription<std_msgs::msg::Bool>("/robot_controller/move_trigger",1,std::bind(&MoveItPlanningNode::trigger_sub_callback, this, _1));

        display_publisher_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        RCLCPP_INFO(this->get_logger(), "ROS interfaces initialized");
    }

    bool MoveItPlanningNode::initializeMoveIt(){
        RCLCPP_INFO(this->get_logger(), "Entering initializeMoveIt function");

        // 创建机器人模型加载器
        try{
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "my_robot_description");
            // 这里的 robot_description 是一个ros2 的param，类型为string，指向机器人的描述文件
        }
        catch(const std::exception & e){
            RCLCPP_ERROR(this->get_logger(),"robot_model_loader_ fail with %s",e.what());
            return 0;
        }
        RCLCPP_INFO(this->get_logger(), "Robot model loader created");
        try{
            robot_model_ = robot_model_loader_->getModel();
        }
        catch(const std::exception & e){
            RCLCPP_ERROR(this->get_logger(),"robot_model_ fail with %s",e.what());
            return 0;
        }
        RCLCPP_INFO(this->get_logger(), "model created");

        // 创建机器人状态和关节模型组
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        joint_model_group_ = robot_state_->getJointModelGroup(PLANNING_GROUP);

        RCLCPP_INFO(this->get_logger(), "Robot state and joint model group created");

        // 创建规划场景
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
        //将机械臂复原，这个需要和电控确定如何操控，可以是复位到当前的机械臂状态！！！  
        planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group_, "home");
        RCLCPP_INFO(this->get_logger(), "Planning scene created and state set to default");

        // 加载规划插件
        if (!this->get_parameter("ompl.planning_plugins", planner_plugin_names_)){
            RCLCPP_FATAL(this->get_logger(), "Could not find planner plugin names");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Planner plugin names loaded");

        try{
            planner_plugin_loader_ = std::make_shared<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
                "moveit_core", "planning_interface::PlannerManager");
        }
        catch (pluginlib::PluginlibException &ex){
            RCLCPP_FATAL(this->get_logger(), "Exception while creating planning plugin loader %s", ex.what());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Planning plugin loader created");

        if (planner_plugin_names_.empty()){
            RCLCPP_ERROR(this->get_logger(),
                         "No planner plugins defined. Please make sure that the planning_plugins parameter is not empty.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Planner plugin names checked");

        // 实例化规划器
        const auto &planner_name = planner_plugin_names_.at(0);
        
        RCLCPP_INFO(this->get_logger(),"use planner : %s",planner_name.c_str());

        try{
            planner_instance_ = planner_plugin_loader_->createSharedInstance(planner_name);
            if (!planner_instance_->initialize(robot_model_, this->shared_from_this(), this->get_namespace())){
                RCLCPP_FATAL(this->get_logger(), "Could not initialize planner instance");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Using planning interface '%s'", planner_instance_->getDescription().c_str());
        }
        catch (pluginlib::PluginlibException &ex){
            const std::vector<std::string> &classes = planner_plugin_loader_->getDeclaredClasses();
            std::stringstream ss;
            for (const auto &cls : classes)
                ss << cls << " ";
            RCLCPP_ERROR(this->get_logger(), "Exception while loading planner '%s': %s\nAvailable plugins: %s",
                         planner_name.c_str(), ex.what(), ss.str().c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Planner instance created and initialized");

        // 初始化MoveGroup接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), PLANNING_GROUP);
        RCLCPP_INFO(this->get_logger(), "MoveGroup interface initialized");

        // 初始化可视化工具
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            this->shared_from_this(), "panda_link0", "move_group_tutorial", move_group_->getRobotModel());
        visual_tools_->enableBatchPublishing();
        visual_tools_->deleteAllMarkers(); // 清除所有旧标记
        visual_tools_->trigger();
        RCLCPP_INFO(this->get_logger(), "Visual tools initialized");

        // 加载远程控制
        visual_tools_->loadRemoteControl();
        RCLCPP_INFO(this->get_logger(), "Remote control loaded");

        RCLCPP_INFO(this->get_logger(), "MoveIt components initialized");
        return true;
    }

    bool MoveItPlanningNode::planToPoseGoal(const std::string & target_joint,const geometry_msgs::msg::PoseStamped &pose, moveit_msgs::msg::Constraints::SharedPtr path_constraints_){
        // 创建运动规划请求
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        // 设置位置和姿态容错
        std::vector<double> tolerance_pose(3, 0.01);
        // 设置位置和姿态容错
        std::vector<double> tolerance_angle(3, 0.01);

        // 构造目标约束
        moveit_msgs::msg::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints(target_joint, pose, tolerance_pose, tolerance_angle);

        req.group_name = PLANNING_GROUP;
        req.goal_constraints.push_back(pose_goal);

        if(path_constraints_){
            req.path_constraints=*path_constraints_;
        }

        // 定义工作空间边界
        // TODO: 需要改进 yaml
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
            req.workspace_parameters.min_corner.z = -5.0;
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
            req.workspace_parameters.max_corner.z = 5.0;

        // 构建规划上下文
        planning_interface::PlanningContextPtr context =
            planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

        if (!context){
            RCLCPP_ERROR(this->get_logger(), "Failed to create planning context");
            return false;
        }

        // 求解规划问题
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Plan successfully!");

        // 可视化结果
        moveit_msgs::msg::MotionPlanResponse response;
        res.getMessage(response);
        visualizeTrajectory(response);

        // 更新规划场景中的机器人状态
        robot_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
        planning_scene_->setCurrentState(*robot_state_.get());

        // 显示目标状态
        visual_tools_->publishAxisLabeled(pose.pose, "pose_goal");
        visual_tools_->publishText(Eigen::Isometry3d::Identity(), "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools_->trigger();

        return true;
    }

    bool MoveItPlanningNode::planToJointGoal(const std::vector<double> &joint_values){
        // 创建运动规划请求
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        // 设置关节空间目标
        moveit::core::RobotState goal_state(robot_model_);
        goal_state.setJointGroupPositions(joint_model_group_, joint_values);
        moveit_msgs::msg::Constraints joint_goal =
            kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_);

        req.group_name = PLANNING_GROUP;
        req.goal_constraints.clear();
        req.goal_constraints.push_back(joint_goal);

        // 构建规划上下文
        planning_interface::PlanningContextPtr context =
            planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

        // 求解规划问题
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
            return false;
        }

        // 可视化结果
        moveit_msgs::msg::MotionPlanResponse response;
        res.getMessage(response);
        visualizeTrajectory(response);

        // 更新规划场景中的机器人状态
        robot_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
        planning_scene_->setCurrentState(*robot_state_.get());

        // 显示目标状态
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools_->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools_->trigger();

        return true;
    }

    bool MoveItPlanningNode::planToConstrainedGoal(const geometry_msgs::msg::PoseStamped &pose){
        // 创建运动规划请求
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        // 设置位置和姿态容错
        std::vector<double> tolerance_pose(3, 0.01);
        std::vector<double> tolerance_angle(3, 0.01);

        // 构造目标约束
        moveit_msgs::msg::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

        req.group_name = PLANNING_GROUP;
        req.goal_constraints.push_back(pose_goal);

        // 添加路径约束，让末端执行器保持水平
        geometry_msgs::msg::QuaternionStamped quaternion;
        quaternion.header.frame_id = "panda_link0";
        req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);

        // 定义工作空间边界
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
            req.workspace_parameters.min_corner.z = -2.0;
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
            req.workspace_parameters.max_corner.z = 2.0;

        // 构建规划上下文
        planning_interface::PlanningContextPtr context =
            planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

        // 求解规划问题
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
            return false;
        }

        #ifdef VISUALIZE
        // 可视化结果
        moveit_msgs::msg::MotionPlanResponse response;
        res.getMessage(response);
        visualizeTrajectory(response);
        #endif

        // 更新规划场景中的机器人状态
        robot_state_->setJointGroupPositions(joint_model_group_, response.trajectory.joint_trajectory.points.back().positions);
        planning_scene_->setCurrentState(*robot_state_.get());

        // 显示目标状态
        #ifdef VISUALIZE
        visual_tools_->publishAxisLabeled(pose.pose, "constrained_goal");
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools_->publishText(text_pose, "Orientation Constrained Motion Plan", rvt::WHITE, rvt::XLARGE);
        visual_tools_->trigger();
        #endif

        return true;
    }

    void MoveItPlanningNode::visualizeTrajectory(const moveit_msgs::msg::MotionPlanResponse &response){
        display_trajectory_.trajectory_start = response.trajectory_start;
        display_trajectory_.trajectory.push_back(response.trajectory);
        visual_tools_->publishTrajectoryLine(display_trajectory_.trajectory.back(), joint_model_group_);
        visual_tools_->trigger();
        display_publisher_->publish(display_trajectory_);
    }

    const std::shared_ptr<moveit_visual_tools::MoveItVisualTools> MoveItPlanningNode::getVisualTools(){
        return visual_tools_;
    }

    void MoveItPlanningNode::trigger_sub_callback(const std_msgs::msg::Bool::SharedPtr msg_){

        geometry_msgs::msg::PoseStamped pose;

        pose.header.frame_id = "robot_base";
        pose.pose.position.x = 0.3;
        pose.pose.position.y = 0.4;
        pose.pose.position.z = 0.75;
        pose.pose.orientation.w = 1.0;

        if(this->planToPoseGoal(EndEffectorJoint, pose)){
            RCLCPP_INFO(this->get_logger(),"trigger_sub_callback: planToPoseGoal OK!");
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"trigger_sub_callback: planToPoseGoal Fail!");
        }
        
    }

} // namespace Engineering_robot_RM2025_Pnx
