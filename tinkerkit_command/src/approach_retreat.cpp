#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Tinkerkit_move");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "arm";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  // Ready State
  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "READY");

  // joint_group_positions_arm[0] = 0.00;  // Base pan
  joint_group_positions_arm[1] = 0.68; // Shoulder Lift
  joint_group_positions_arm[2] = -1.55;  // Elbow
  joint_group_positions_arm[3] = -0.994838; // arm
  joint_group_positions_arm[4] = 0; // wrist


  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Pregrasp

  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -0.688;
  target_pose1.orientation.y = -0.000;
  target_pose1.orientation.z = 0.000;
  target_pose1.orientation.w = 0.725;
  target_pose1.position.x = -0.000;
  target_pose1.position.y = 2.388;
  target_pose1.position.z = 0.922;


  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  move_group_arm.execute(my_plan_arm);

  // Approach

  RCLCPP_INFO(LOGGER, "Approach to object");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  float delta = 0.3;
  target_pose1.orientation.x = -0.688;
  target_pose1.orientation.y = -0.000;;
  target_pose1.orientation.z = 0.000;
  target_pose1.orientation.w = 0.725;
  target_pose1.position.x = -0.000;
  target_pose1.position.y = 2.388;
  target_pose1.position.z = target_pose1.position.z - delta;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  move_group_arm.execute(my_plan_arm);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  target_pose1.orientation.x = -0.688;
  target_pose1.orientation.y = -0.000;;
  target_pose1.orientation.z = 0.000;
  target_pose1.orientation.w = 0.725;
  target_pose1.position.x = -0.000;
  target_pose1.position.y = 2.388;
  target_pose1.position.z = target_pose1.position.z + delta;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  move_group_arm.execute(my_plan_arm);

  rclcpp::shutdown();
  return 0;
}