#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "tinkerkit_msgs/action/PickPlaceTask.hpp"  // Update with your package and action name

using namespace std::placeholders;

class PickPlaceActionServer : public rclcpp::Node {
public:
  using PickPlaceTask = your_package::action::PickPlaceTask;
  using GoalHandlePickPlaceTask = rclcpp_action::ServerGoalHandle<PickPlaceTask>;

  PickPlaceActionServer() : Node("pick_place_action_server") {
    action_server_ = rclcpp_action::create_server<PickPlaceTask>(
      this,
      "pick_place_task",
      std::bind(&PickPlaceActionServer::handle_goal, this, _1, _2),
      std::bind(&PickPlaceActionServer::handle_cancel, this, _1),
      std::bind(&PickPlaceActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<PickPlaceTask>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceTask::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received request for task: %s", goal->task_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlaceTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Canceling task");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlaceTask> goal_handle) {
    std::thread{std::bind(&PickPlaceActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePickPlaceTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing task");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PickPlaceTask::Feedback>();
    auto result = std::make_shared<PickPlaceTask::Result>();

    // Perform each sub-task in sequence
    if (!move_to_target(goal->x, goal->y, goal->z)) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    feedback->progress = 0.5;
    goal_handle->publish_feedback(feedback);

    if (!grasp_object()) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    if (!place_object(goal->x, goal->y, goal->z)) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  bool move_to_target(double x, double y, double z) {
    // Implement motion to the target (e.g., inverse kinematics)
    RCLCPP_INFO(this->get_logger(), "Moving to target (%.2f, %.2f, %.2f)", x, y, z);
    // Simulate a task delay
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
  }

  bool grasp_object() {
    // Implement grasp logic (e.g., open/close gripper)
    RCLCPP_INFO(this->get_logger(), "Grasping object");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
  }

  bool place_object(double x, double y, double z) {
    // Implement place logic (e.g., position at release point)
    RCLCPP_INFO(this->get_logger(), "Placing object at (%.2f, %.2f, %.2f)", x, y, z);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
