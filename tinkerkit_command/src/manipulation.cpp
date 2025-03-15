// --------------------- Moveit Library
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// --------------------- Transform Frame Library
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick_and_Place");
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_and_place_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    using moveit::planning_interface::MoveGroupInterface;
    
    // Initialize MoveGroup Interfaces
    moveit::planning_interface::MoveGroupInterface move_group_arm(node, PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_gripper(node, PLANNING_GROUP_GRIPPER);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    // Set Planning Time
    move_group_arm.setPlanningTime(45.0);
    move_group_gripper.setPlanningTime(45.0);

    // Planning Parameters
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    // Logging Setup
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_arm.getEndEffectorLink().c_str());

    // Open Gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper");
    move_group_gripper.setNamedTarget("open");
    if (move_group_gripper.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to open gripper!");
        return 1;
    }  

    // Move Arm to Ready Position
    RCLCPP_INFO(LOGGER, "Moving Arm to Ready Position");
    std::vector<double> joint_positions = {0.0, 0.767945, -1.72788, -0.593412, 0.0};
    move_group_arm.setJointValueTarget(joint_positions);
    if (move_group_arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to move arm to ready position!");
        return 1;
    }

    // Move to Object
    RCLCPP_INFO(LOGGER, "Moving Arm to Object Position");
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(-1.55, 0.0, 0.0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0.0;
    target_pose.position.y = 2.29;
    target_pose.position.z = 1.5;
    move_group_arm.setPoseTarget(target_pose);
    if (move_group_arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to move arm to object position!");
        return 1;
    }

    //------------------------------- Approach -------------------------------
    RCLCPP_INFO(LOGGER, "Approach to object!");
    geometry_msgs::msg::Pose target_pose1 = target_pose; // Define target_pose1
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    target_pose1.position.z -= 0.5;
    approach_waypoints.push_back(target_pose1);
    target_pose1.position.z -= 0.5;
    approach_waypoints.push_back(target_pose1);
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    double fraction1 = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);
    move_group_arm.execute(trajectory_approach);

    // Close Gripper to Pick Object
    RCLCPP_INFO(LOGGER, "Closing Gripper");
    move_group_gripper.setNamedTarget("close");
    if (move_group_gripper.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to close gripper!");
        return 1;
    }

    // Place Object with Quaternion Values
    RCLCPP_INFO(LOGGER, "Placing Object");
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation.w = 0.722;
    target_pose2.orientation.x = -0.692;
    target_pose2.orientation.y = 0.002;
    target_pose2.orientation.z = 0.003;
    target_pose2.position.x = 0;
    target_pose2.position.y = 2.140;
    target_pose2.position.z = 1.978;
    move_group_arm.setPoseTarget(target_pose2);
    if (move_group_arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to place object!");
        return 1;
    }

    // Open Gripper to Release Object
    RCLCPP_INFO(LOGGER, "Releasing Object");
    move_group_gripper.setNamedTarget("open");
    if (move_group_gripper.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to open gripper!");
        return 1;
    }

    // Move Arm to Ready Position
    RCLCPP_INFO(LOGGER, "Moving Arm to Ready Position");
    move_group_arm.setJointValueTarget(joint_positions);
    if (move_group_arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to move arm to ready position!");
        return 1;
    }

    RCLCPP_INFO(LOGGER, "Pick and Place Operation Completed Successfully!");
    rclcpp::shutdown();
    return 0;
}
