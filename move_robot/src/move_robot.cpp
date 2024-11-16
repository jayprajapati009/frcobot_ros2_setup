/**
 * @file move_robot.cpp
 * @brief Example program to control a robot using the MoveIt MoveGroup
 * Interface in ROS 2.
 *
 * This program initializes a ROS node, sets a target pose for the robot, plans
 * a trajectory, and executes the plan using MoveIt.
 */

#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Main function for moving the robot using MoveIt MoveGroupInterface.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Program exit status (0 for success, non-zero for failure).
 */
int main(int argc, char* argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_robot",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_robot");

  /**
   * @brief Create the MoveIt MoveGroup Interface.
   *
   * This interface is used to interact with the robot's planning group.
   * The group name "fairino3_v6_group" should match the group name defined
   * in the MoveIt configuration.
   */
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "fairino3_v6_group");

  /**
   * @brief Set a target pose for the robot.
   *
   * The target pose specifies the desired position and orientation
   * for the end-effector of the robot.
   */
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  /**
   * @brief Plan a trajectory to the target pose.
   *
   * This block generates a motion plan to move the robot's end-effector
   * to the specified target pose.
   *
   * @return A pair containing the success flag and the generated plan.
   */
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  /**
   * @brief Execute the planned trajectory if planning was successful.
   *
   * If the planning fails, an error is logged.
   */
  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
