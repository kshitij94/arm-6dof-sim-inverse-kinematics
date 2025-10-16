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

  // Parameters are automatically declared from the launch file

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

  // Get and log the current pose
  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Current pose: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f",
              current_pose.position.x, current_pose.position.y, current_pose.position.z,
              current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);

  auto target_pose = current_pose;

  target_pose.position.x = -0.375;
  target_pose.position.y = 0.157;
  target_pose.position.z = 0.552;

  move_group_interface.setPoseTarget(target_pose);

  if (move_group_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "Execution successful");
  }
  else
  {
    RCLCPP_ERROR(logger, "Execution failed ");
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}