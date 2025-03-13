#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "tinkerkit_msgs/action/tinkerkit_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


using namespace std::placeholders;

namespace tinkerkit_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<tinkerkit_msgs::action::TinkerkitTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<tinkerkit_msgs::action::TinkerkitTask>::SharedPtr action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const tinkerkit_msgs::action::TinkerkitTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<tinkerkit_msgs::action::TinkerkitTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
    arm_move_group.stop();
    gripper_move_group.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<tinkerkit_msgs::action::TinkerkitTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tinkerkit_msgs::action::TinkerkitTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<tinkerkit_msgs::action::TinkerkitTask::Result>();

    // MoveIt 2 Interface
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

    std::vector<double> arm_joint_goal;
    std::vector<double> gripper_joint_goal;

    if (goal_handle->get_goal()->task_number == 0)
    {
      //Ready
      arm_joint_goal = {0, 0.68, -1.55,-0.994838,0};
      gripper_joint_goal = {-1.5708};
    }
    else if (goal_handle->get_goal()->task_number == 1)
    {
      arm_joint_goal = {0, -0.733038, -1.39626, 0.610865, 0};
      gripper_joint_goal = {0.261799};
    }
    else if (goal_handle->get_goal()->task_number == 2)
    {
      arm_joint_goal = {0, -0.0349066, -0.523599, -0.994838, 0};
      gripper_joint_goal = {0.261799};
    }
    else if (goal_handle->get_goal()->task_number == 3)//Can
    {
      arm_joint_goal = {3.14159, 0.785398, -1.309, -1.0472, 0.139626};
      gripper_joint_goal = {0.261799};
    }
    else if (goal_handle->get_goal()->task_number == 4)
    {
      arm_joint_goal = {3.14159, -1.0472, -1.13446, 0.610865, 0};
      gripper_joint_goal = {-1.5708};
    }
    else if (goal_handle->get_goal()->task_number == 5)
    {
      arm_joint_goal = {3.14159, 0.785398, -1.309, -1.0472, 0.139626};
      gripper_joint_goal = {-1.5708};
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      return;
    }
    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(arm_plan_success && gripper_plan_success)
    {
      RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
      arm_move_group.move();
      gripper_move_group.move();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "One or more planners failed!");
      return;
    }
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
};
}  // namespace tinkerkit_remote

RCLCPP_COMPONENTS_REGISTER_NODE(tinkerkit_remote::TaskServer)