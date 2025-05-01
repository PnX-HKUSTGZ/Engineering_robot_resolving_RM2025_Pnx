#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h> // MoveIt Planning Scene Interface
#include <moveit_msgs/msg/collision_object.hpp>                  // Collision Object message
#include <geometric_shapes/shape_operations.h>                  // For loading mesh files
#include <geometry_msgs/msg/pose.hpp>                           // Pose message
#include <std_msgs/msg/header.hpp>                              // Header message

// Define your object properties
const std::string OBJECT_ID = "my_solidworks_object";
const std::string MESH_PATH = "package://my_robot_package/meshes/my_object.stl"; // <replace_with_your_package> and file name
// *** This is the TF2 frame you want the object to be attached to ***
const std::string TARGET_TF_FRAME_ID = "my_tf_frame"; // <replace_with_your_target_tf_frame_id>

class CollisionObjectPublisher : public rclcpp::Node
{
public:
  CollisionObjectPublisher() : Node("collision_object_publisher")
  {
    // Initialize Planning Scene Interface
    // It connects to the MoveIt planning scene manager internally
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    RCLCPP_INFO(this->get_logger(), "CollisionObjectPublisher node started.");
    RCLCPP_INFO(this->get_logger(), "Attempting to add collision object '%s' attached to TF frame '%s'.",
                OBJECT_ID.c_str(), TARGET_TF_FRAME_ID.c_str());


    // Use a timer to add the object after a short delay.
    // This ensures the PlanningSceneInterface has connected to MoveIt.
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&CollisionObjectPublisher::add_collision_object, this));
  }

private:
  void add_collision_object()
  {
    // Stop the timer after the first execution
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Adding collision object...");

    // 1. Create CollisionObject message
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.stamp = this->now();

    // *** CRITICAL: Set the frame_id to the target TF2 frame name ***
    collision_object.header.frame_id = TARGET_TF_FRAME_ID;

    collision_object.id = OBJECT_ID;

    // 2. Load the Mesh file
    shapes::Mesh* mesh = shapes::createMeshFromResource(MESH_PATH);

    if (!mesh)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", MESH_PATH.c_str());
      RCLCPP_ERROR(this->get_logger(), "Object '%s' will NOT be added to the scene.", OBJECT_ID.c_str());
      return;
    }

    // Convert the loaded shapes::Mesh* to a shape_msgs::msg::Mesh
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_base;
    shapes::constructMsgFromShape(mesh, mesh_msg_base);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_base);

    // *** IMPORTANT: Remember to delete the allocated shapes::Mesh object ***
    delete mesh;
    mesh = nullptr; // Good practice to set pointer to null after delete

    if (mesh_msg.vertices.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Loaded mesh data is empty for %s.", MESH_PATH.c_str());
        RCLCPP_ERROR(this->get_logger(), "Object '%s' will NOT be added to the scene.", OBJECT_ID.c_str());
        return;
    }


    // 3. Define the pose of the Mesh RELATIVE TO the TARGET_TF_FRAME_ID
    geometry_msgs::msg::Pose object_pose_relative_to_tf;

    // Example: Place the mesh's local origin at the TF frame's origin,
    // with its axes aligned with the TF frame's axes.
    object_pose_relative_to_tf.position.x = 0.0;
    object_pose_relative_to_tf.position.y = 0.0;
    object_pose_relative_to_tf.position.z = 0.0;
    object_pose_relative_to_tf.orientation.x = 0.0;
    object_pose_relative_to_tf.orientation.y = 0.0;
    object_pose_relative_to_tf.orientation.z = 0.0;
    object_pose_relative_to_tf.orientation.w = 1.0; // Identity quaternion (no rotation)

    // If your Mesh's local origin is NOT at the point you want to align with the TF frame,
    // or if your Mesh's default axes are NOT aligned, you'll need to adjust this pose.
    // Example: Move the object 0.1m up from the TF frame's origin
    // object_pose_relative_to_tf.position.z = 0.1;
    // Example: Rotate the mesh 90 degrees around the TF frame's Z axis relative to its default orientation
    // #include <tf2/LinearMath/Quaternion.h>
    // tf2::Quaternion q;
    // q.setRPY(0, 0, M_PI_2); // Roll=0, Pitch=0, Yaw=90deg (PI/2 rad)
    // object_pose_relative_to_tf.orientation.x = q.x();
    // object_pose_relative_to_tf.orientation.y = q.y();
    // object_pose_relative_to_tf.orientation.z = q.z();
    // object_pose_relative_to_tf.orientation.w = q.w();


    // 4. Add the Mesh and its relative pose to the CollisionObject
    collision_object.meshes.push_back(mesh_msg);
    // The pose in mesh_poses is relative to the collision_object's frame_id
    collision_object.mesh_poses.push_back(object_pose_relative_to_tf);

    // 5. Set the operation to ADD
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 6. Use PlanningSceneInterface to add the object to the scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface_->addCollisionObjects(collision_objects);

    RCLCPP_INFO(this->get_logger(), "Collision object '%s' added to the planning scene attached to frame '%s'.",
                OBJECT_ID.c_str(), TARGET_TF_FRAME_ID.c_str());

    // You can verify in RViz by checking the "Planning Scene" display
    // and looking under "Scene Geometry" -> "Collision Objects"
  }

  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionObjectPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}