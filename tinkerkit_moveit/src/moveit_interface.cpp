#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Ready
    std::vector<double> arm_joint_goal {0, 0.68, -1.55,-0.994838,0};
    std::vector<double> gripper_joint_goal {-0.698, 0.698};
    // move
    //  std::vector<double> arm_joint_goal {0, -0.733038, -1.5708, 0.785398, 0};
    //  std::vector<double> gripper_joint_goal {-0.7, 0.7};
    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
    

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm_plan_success && gripper_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner SUCCEED, moving the arme and the gripper");
        arm_move_group.move();
        gripper_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "One or more planners failed!");
        return;
    }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("moveit_interface");
  move_robot(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}