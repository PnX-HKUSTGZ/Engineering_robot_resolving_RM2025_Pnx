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

    // box_position

        if(use_virtual_box_position){
            int k=0;
            virtual_box_position_timer_=this->create_wall_timer(10ms,[this,&k](){
                tf2_ros::TransformBroadcaster tran(this);
                geometry_msgs::msg::TransformStamped robot_to_base;

                robot_to_base.header.stamp=this->now();
                robot_to_base.header.frame_id="map";
                robot_to_base.child_frame_id=RedeemBox_frame;
                robot_to_base.transform.translation.x=0;
                robot_to_base.transform.translation.y=0.75;
                robot_to_base.transform.translation.z=0.4;
                robot_to_base.transform.rotation.w=0.3826834 ;
                robot_to_base.transform.rotation.x=-0.6159197;
                robot_to_base.transform.rotation.y=0.3079598;
                robot_to_base.transform.rotation.z=-0.6159197;
                // robot_to_base.transform.rotation.w=1 ;
                // robot_to_base.transform.rotation.x=0;
                // robot_to_base.transform.rotation.y=0;
                // robot_to_base.transform.rotation.z=0;
                tran.sendTransform(robot_to_base);

                geometry_msgs::msg::TransformStamped msg;
                msg.header.stamp=this->now();
                msg.header.frame_id="robot_base_link";
                msg.child_frame_id="sensor/RealSense";
                msg.transform.translation.x=0.156999970395237;
                msg.transform.translation.y=-0.033000353576592;
                msg.transform.translation.z=0.36374073844197;
                msg.transform.rotation.w=1;
                msg.transform.rotation.x=0;
                msg.transform.rotation.y=0;
                msg.transform.rotation.z=0;
                tran.sendTransform(msg);
                geometry_msgs::msg::TransformStamped image_to_center_msg;
                geometry_msgs::msg::TransformStamped depth_to_center_msg;

                depth_to_center_msg.header.frame_id="sensor/RealSense";
                depth_to_center_msg.child_frame_id="sensor/RealSense/depth";
                depth_to_center_msg.header.stamp=this->now();
                depth_to_center_msg.transform.translation.x=-0.02;
                depth_to_center_msg.transform.translation.y=-1.1*1e-3;
                depth_to_center_msg.transform.translation.z=0;
                depth_to_center_msg.transform.rotation.x=0.7071068;
                depth_to_center_msg.transform.rotation.y=0;
                depth_to_center_msg.transform.rotation.z=0;
                depth_to_center_msg.transform.rotation.w=-0.7071068;
        
                image_to_center_msg.header.frame_id="sensor/RealSense";
                image_to_center_msg.child_frame_id="sensor/RealSense/image";
                image_to_center_msg.header.stamp=this->now();
                image_to_center_msg.transform.translation.x=-0.035;
                image_to_center_msg.transform.translation.y=-1.1*1e-3;
                image_to_center_msg.transform.translation.z=0;
                image_to_center_msg.transform.rotation.x=0.7071068;
                image_to_center_msg.transform.rotation.y=0;
                image_to_center_msg.transform.rotation.z=0;
                image_to_center_msg.transform.rotation.w=-0.7071068;
                tran.sendTransform(image_to_center_msg);
                tran.sendTransform(depth_to_center_msg);

                geometry_msgs::msg::TransformStamped to_map;

                to_map.header.frame_id="robot_base_link";
                to_map.child_frame_id="sensor/camera";
                to_map.header.stamp=this->now();
                to_map.transform.rotation.w=0.7071068;
                to_map.transform.rotation.x=-0.7071068;
                to_map.transform.rotation.y=0;
                to_map.transform.rotation.z=0;
                to_map.transform.translation.x=-0.080000000000085;
                to_map.transform.translation.y=0.0121000016614134;
                to_map.transform.translation.z=0.36124073844197;
                tran.sendTransform(to_map);

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
