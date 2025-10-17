#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include "tf2/LinearMath/Transform.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "moveit_client",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_client");

  std::thread spin_thread([node]()
                          { rclcpp::spin(node); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Wait for the CurrentStateMonitor to receive joint states
  RCLCPP_INFO(logger, "Waiting for MoveGroupInterface to receive current state...");
  move_group_interface.setStartStateToCurrentState();

  // Get and log the current pose
  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Current pose: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f",
              current_pose.position.x, current_pose.position.y, current_pose.position.z,
              current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);

  // The user wants to move the hand along its x-axis.
  // 1. Define a transform representing a 5cm translation along the x-axis.
  tf2::Transform transform_in_hand_frame;
  transform_in_hand_frame.setOrigin(tf2::Vector3(-0.1, 0.0, 0.0));
  transform_in_hand_frame.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  // 2. Convert the current pose to a tf2 transform.
  tf2::Transform current_pose_tf;
  tf2::fromMsg(current_pose, current_pose_tf);

  // 3. Compute the target pose by composing the current pose with the desired translation.
  // The order of multiplication is important. Post-multiplication applies the transform in the local frame.
  tf2::Transform target_pose_tf = current_pose_tf * transform_in_hand_frame;

  // 4. Convert the result back to a ROS message.
  geometry_msgs::msg::Pose target_pose;
  tf2::toMsg(target_pose_tf.getOrigin(), target_pose.position);
  target_pose.orientation = tf2::toMsg(target_pose_tf.getRotation());

  RCLCPP_INFO(logger, "New target pose: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);

  move_group_interface.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful, executing the plan.");
    if (move_group_interface.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Execution successful");
    }
    else
    {
      RCLCPP_ERROR(logger, "Execution failed");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}