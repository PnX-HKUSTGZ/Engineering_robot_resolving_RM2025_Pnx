#include "position_controller/position_controller.hpp"

namespace Engineering_robot_RM2025_Pnx{

    PositionController::PositionController(rclcpp::NodeOptions options):
    Node("PositionController",options){

    //declare para

        // RedeemBox_detector part
        RedeemBox_frame=this->declare_parameter<std::string>("RedeemBox_frame","object/box");
        use_virtual_box_position=this->declare_parameter<bool>("use_virtual_box_position",true);

        //robot_position

        robot_base=this->declare_parameter<std::string>("robot_base","robot_base");
        fixed_frame=this->declare_parameter<std::string>("fixed_frame","base");
        use_virtual_robot_position=this->declare_parameter<bool>("use_virtual_robot_position",true);

        // Output declared parameters
        RCLCPP_INFO_STREAM(this->get_logger(), "RedeemBox_frame: " << RedeemBox_frame);
        RCLCPP_INFO_STREAM(this->get_logger(), "use_virtual_box_position: " << (use_virtual_box_position ? "true" : "false"));
        RCLCPP_INFO_STREAM(this->get_logger(), "robot_base: " << robot_base);
        RCLCPP_INFO_STREAM(this->get_logger(), "fixed_frame: " << fixed_frame);
        RCLCPP_INFO_STREAM(this->get_logger(), "use_virtual_robot_position: " << (use_virtual_robot_position ? "true" : "false"));

        RCLCPP_INFO_STREAM(this->get_logger(),"Load param succesfully!");

    // general part
        tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_transform_listensr_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);

        RCLCPP_INFO_STREAM(this->get_logger(),"general part load finish");

    // robot_position

        Robot_base_tf2_pub_=std::make_shared<tf2_ros::TransformBroadcaster>(this);


        if(use_virtual_robot_position){
            virtual_robot_position_timer_=this->create_wall_timer(50ms,[this](){
                geometry_msgs::msg::TransformStamped robot_to_base;

                robot_to_base.header.stamp=this->now();
                robot_to_base.header.frame_id=fixed_frame;
                robot_to_base.child_frame_id=robot_base;
                robot_to_base.transform.translation.x=0;
                robot_to_base.transform.translation.y=0;
                robot_to_base.transform.translation.z=0;
                robot_to_base.transform.rotation.w=1;
                robot_to_base.transform.rotation.x=0;
                robot_to_base.transform.rotation.y=0;
                robot_to_base.transform.rotation.z=0;

                Robot_base_tf2_pub_->sendTransform(robot_to_base);
                RCLCPP_INFO(this->get_logger(),"virtual_robot_position pub");
            });
            RCLCPP_INFO_STREAM(this->get_logger(),"use_virtual_robot_position ture, use virtual robot position");
        }

        RCLCPP_INFO_STREAM(this->get_logger(),"robot_position part load finish");

    // box_position

        if(use_virtual_box_position){
            int k=0;
            virtual_box_position_timer_=this->create_wall_timer(50ms,[this,&k](){
                tf2_ros::TransformBroadcaster tran(this);
                geometry_msgs::msg::TransformStamped robot_to_base;

                robot_to_base.header.stamp=this->now();
                robot_to_base.header.frame_id=fixed_frame;
                robot_to_base.child_frame_id=RedeemBox_frame;
                robot_to_base.transform.translation.x=0.3;
                robot_to_base.transform.translation.y=0.4;
                robot_to_base.transform.translation.z=k%3+1;
                robot_to_base.transform.rotation.w=1;
                robot_to_base.transform.rotation.x=0;
                robot_to_base.transform.rotation.y=0;
                robot_to_base.transform.rotation.z=0;
                tran.sendTransform(robot_to_base);
                k++;
                RCLCPP_INFO(this->get_logger(),"virtual_box_position pub");
            });
            RCLCPP_INFO_STREAM(this->get_logger(),"use_virtual_box_position ture, use virtual box position");
        }

        RCLCPP_INFO_STREAM(this->get_logger(),"box_position part load finish");

        RCLCPP_INFO_STREAM(this->get_logger(),"PositionController init OK!");
        

    }

    geometry_msgs::msg::TransformStamped PositionController::GetBoxPosition(rclcpp::Time time){
        geometry_msgs::msg::TransformStamped res;
        try{
            res=tf2_buffer_->lookupTransform(robot_base, fixed_frame, time,20ns);
        }
        catch (tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(),"%s",ex.what());
            throw ex;
        }
        return res;
    }

}// namespace Engineering_robot_RM2025_Pnx{

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Engineering_robot_RM2025_Pnx::PositionController)
