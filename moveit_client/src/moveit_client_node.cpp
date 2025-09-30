#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

// Global variable to store the last received joint state message
sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg;

void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  last_joint_state_msg = msg;
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "moveit_client",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_client");
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;

  joint_state_subscription = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, &topic_callback);

  std::thread spin_thread([node]()
                          { rclcpp::spin(node); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Wait for the CurrentStateMonitor to receive joint states
  RCLCPP_INFO(logger, "Waiting for MoveGroupInterface to receive current state...");
  move_group_interface.setStartStateToCurrentState();

  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Current pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
              current_pose.position.x, current_pose.position.y, current_pose.position.z,
              current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.015758;
  target_pose.position.y = 0.006706;
  target_pose.position.z = 0.365018;
  target_pose.orientation.x = 0.527242;
  target_pose.orientation.y = 0.471197;
  target_pose.orientation.z = -0.227045;
  target_pose.orientation.w = 0.669657;

  RCLCPP_INFO(logger, "Target pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  move_group_interface.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful! Executing plan to move to the target pose.");
    move_group_interface.execute(my_plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed to move to the target pose!");
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}