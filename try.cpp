#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Includes doTransform for geometry_msgs
#include <tf2/LinearMath/Quaternion.h> // For creating quaternions

class TransformPointExample : public rclcpp::Node
{
public:
  TransformPointExample()
  : Node("transform_point_example")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Transform Point Example Node");

    // --- 1. Create a sample TransformStamped message ---
    // This transform will move +1 unit along the X axis
    // and rotate 90 degrees around the Z axis (from 'base_link' to 'tool0')
    geometry_msgs::msg::TransformStamped sample_transform;
    sample_transform.header.stamp = this->get_clock()->now();
    sample_transform.header.frame_id = "base_link"; // Source frame
    sample_transform.child_frame_id = "tool0";      // Target frame

    // Translation: move 1 meter in X
    sample_transform.transform.translation.x = 1.0;
    sample_transform.transform.translation.y = 0.0;
    sample_transform.transform.translation.z = 0.0;

    // Rotation: 90 degrees around Z (using quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI / 2.0); // Roll, Pitch, Yaw (radians) -> 90 deg around Z
    sample_transform.transform.rotation.x = q.x();
    sample_transform.transform.rotation.y = q.y();
    sample_transform.transform.rotation.z = q.z();
    sample_transform.transform.rotation.w = q.w();

    RCLCPP_INFO(this->get_logger(), "Sample Transform:");
    RCLCPP_INFO(this->get_logger(), "  From: %s To: %s",
      sample_transform.header.frame_id.c_str(), sample_transform.child_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Translation: (%f, %f, %f)",
      sample_transform.transform.translation.x, sample_transform.transform.translation.y, sample_transform.transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "  Rotation (Quaternion): (%f, %f, %f, %f)",
      sample_transform.transform.rotation.x, sample_transform.transform.rotation.y,
      sample_transform.transform.rotation.z, sample_transform.transform.rotation.w);


    // --- 2. Create a sample PointStamped message ---
    // This point is at (0, 0, 0) in the 'base_link' frame
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.stamp = this->get_clock()->now();
    point_stamped.header.frame_id = "base_link"; // This must match the transform's frame_id
    point_stamped.point.x = 0.0;
    point_stamped.point.y = 0.0;
    point_stamped.point.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "\nOriginal Point:");
    RCLCPP_INFO(this->get_logger(), "  Coordinates: (%f, %f, %f)",
      point_stamped.point.x, point_stamped.point.y, point_stamped.point.z);
    RCLCPP_INFO(this->get_logger(), "  In frame: %s", point_stamped.header.frame_id.c_str());


    // --- 3. Apply the transform to the point ---
    // The point_stamped's frame_id ('base_link') matches the transform's frame_id.
    // The resulting point will be in the transform's child_frame_id ('tool0').
    geometry_msgs::msg::PointStamped transformed_point;
    try {
      // tf2::doTransform(input, output, transform);
      tf2::doTransform(point_stamped, transformed_point, sample_transform);

      RCLCPP_INFO(this->get_logger(), "\nTransformed Point:");
      RCLCPP_INFO(this->get_logger(), "  Coordinates: (%f, %f, %f)",
        transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
      RCLCPP_INFO(this->get_logger(), "  In frame: %s", transformed_point.header.frame_id.c_str()); // This will be 'tool0'

    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Error applying transform: %s", ex.what());
    }
  }

private:
  // Nothing to spin on for this simple example
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformPointExample>();

  // Since the example code is in the constructor and runs once,
  // we don't need to call rclcpp::spin.
  // rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}