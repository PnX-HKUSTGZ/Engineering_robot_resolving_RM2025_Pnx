#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/SolidPrimitive.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2::createQuaternionMsgFromRollPitchYaw

#include <cmath> // For M_PI

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constrained_planning_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Starting MoveIt constrained planning example...");

  // Set up MoveGroupInterface
  // Replace "your_robot_planning_group" with your robot's planning group name
  static const std::string PLANNING_GROUP = "your_robot_planning_group";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Get planning frame and end effector link
  const std::string planning_frame = move_group_interface.getPlanningFrame();
  const std::string eef_link = move_group_interface.getEndEffectorLink();
  ROS_INFO_STREAM("Planning frame: " << planning_frame);
  ROS_INFO_STREAM("End effector link: " << eef_link);

  // Allow replanning to increase the chances of finding a path
  move_group_interface.allowReplanning(true);
  // Set a high number of planning attempts
  move_group_interface.setNumPlanningAttempts(10);
  // Set planning time
  move_group_interface.setPlanningTime(5.0); // Give it more time with constraints

  // 1. Create Constraints object
  moveit_msgs::Constraints path_constraints;
  path_constraints.name = "end_effector_pose_constraint";

  // 2. Create Position Constraint
  moveit_msgs::PositionConstraint pcon;
  pcon.header.frame_id = planning_frame; // Use the planning frame as reference
  pcon.link_name = eef_link;             // Constraint applies to the EE link

  // Define the center of the allowed position region
  geometry_msgs::Point target_position;
  target_position.x = 1.0;
  target_position.y = 0.5;
  target_position.z = 0.8;

  // Define the shape of the allowed region (a small sphere)
  geometry_msgs::SolidPrimitive sphere;
  sphere.type = geometry_msgs::SolidPrimitive::SPHERE;
  sphere.dimensions.resize(1);
  sphere.dimensions[0] = 0.01; // Radius in meters (small tolerance)

  // Define the pose of the allowed region (center of the sphere)
  geometry_msgs::Pose sphere_pose;
  sphere_pose.position = target_position;
  sphere_pose.orientation.w = 1.0; // Identity quaternion

  pcon.constraint_region.primitives.push_back(sphere);
  pcon.constraint_region.primitive_poses.push_back(sphere_pose);

  pcon.weight = 1.0; // Weight of the constraint

  // 3. Create Orientation Constraint
  moveit_msgs::OrientationConstraint ocon;
  ocon.header.frame_id = planning_frame; // Use the planning frame as reference
  ocon.link_name = eef_link;             // Constraint applies to the EE link

  // Define the target orientation (e.g., pointing straight down if Z is robot's up)
  // Let's assume you want the EE's local Z axis to point roughly towards the global Z axis,
  // and allow rotation around that axis.
  // Example: RPY (0, PI, 0) or (0, -PI, 0) might point the EE's local Z axis upwards or downwards
  // depending on its definition. Let's use RPY (0, 0, 0) for simplicity, assuming EE Z aligns with global Z.
  tf2::Quaternion target_quat_tf;
  target_quat_tf.setRPY(0, 0, 0); // RPY in radians
  ocon.orientation = tf2::toMsg(target_quat_tf);

  // Define the tolerances for rotation around the target orientation's axes
  // Set the tolerance for the axis you want to rotate freely around to a large value (M_PI or 2*M_PI)
  // Set the tolerances for the other two axes to a small value (e.g., 0.01 radians)
  // Example: Allow free rotation around the target orientation's Z axis
  ocon.absolute_x_axis_tolerance = 0.01; // radians
  ocon.absolute_y_axis_tolerance = 0.01; // radians
  ocon.absolute_z_axis_tolerance = 2 * M_PI; // radians (allow full rotation)

  // The parameterization IDENTITY is usually used with absolute tolerances
  ocon.parameterization = moveit_msgs::OrientationConstraint::IDENTITY;

  ocon.weight = 1.0; // Weight of the constraint

  // 4. Add constraints to the Constraints object
  path_constraints.position_constraints.push_back(pcon);
  path_constraints.orientation_constraints.push_back(ocon);

  // 5. Apply path constraints to the move group
  move_group_interface.setPathConstraints(path_constraints);
  ROS_INFO("Path constraints applied.");

  // 6. Set a target pose for planning
  // The target pose should be reachable and satisfy the constraints.
  // Let's set a target pose that is at the center of our position constraint
  // and has an orientation that is the constraint orientation rotated by some amount around the allowed axis.
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = planning_frame;
  target_pose.pose.position = target_position;

  // Rotate the target orientation by PI/2 around its Z axis for the target pose
  tf2::Quaternion target_quat_rotated_tf;
  tf2::Quaternion rotation_around_z;
  rotation_around_z.setRPY(0, 0, M_PI / 2.0); // Rotate by 90 degrees around Z
  target_quat_rotated_tf = target_quat_tf * rotation_around_z; // Multiply quaternions

  target_pose.pose.orientation = tf2::toMsg(target_quat_rotated_tf);

  move_group_interface.setPoseTarget(target_pose);
  ROS_INFO_STREAM("Setting pose target: " << target_pose);


  // 7. Plan the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ROS_INFO("Attempting to plan trajectory with constraints...");
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_STREAM("Planning " << (success ? "SUCCEEDED" : "FAILED"));

  // 8. (Optional) Execute the planned path
  // if (success)
  // {
  //   ROS_INFO("Executing planned path...");
  //   move_group_interface.execute(my_plan);
  //   ROS_INFO("Path execution complete.");
  // }

  // 9. Clear constraints
  move_group_interface.clearPathConstraints();
  ROS_INFO("Path constraints cleared.");

  ros::shutdown();
  return 0;
}