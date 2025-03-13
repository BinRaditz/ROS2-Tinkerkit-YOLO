#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>

const double tau = 2 * M_PI;

class MoveGroupInterfaceNode : public rclcpp::Node
{
public:
    MoveGroupInterfaceNode() : Node("move_group_interface_tutorial")
    {
        // Create a publisher for displaying planned trajectories
        display_publisher_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/move_group/display_planned_path", 1);

        // Initialize MoveGroupInterface for the "arm" planning group
        auto node = shared_from_this();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End-effector link: %s", move_group_->getEndEffectorLink().c_str());

        // Set the target position
        geometry_msgs::msg::Pose target_pose1;
        tf2::Quaternion orientation;
        orientation.setRPY(-tau / 4, -tau / 4, 0);
        target_pose1.orientation = tf2::toMsg(orientation);
        target_pose1.position.x = 1.0;
        target_pose1.position.y = 0.0;
        target_pose1.position.z = 0.5;
        move_group_->setPoseTarget(target_pose1);

        // Visualize the planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto success = move_group_->plan(my_plan);

        RCLCPP_INFO(this->get_logger(), "Visualizing plan %s", success == moveit::planning_interface::MoveItErrorCode::SUCCESS ? "SUCCESS" : "FAILED");

        // Execute the plan
        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group_->move();
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveGroupInterfaceNode>();

    // Spin the node to keep it active
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
