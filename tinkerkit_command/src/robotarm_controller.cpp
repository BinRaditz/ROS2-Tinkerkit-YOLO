#include <rclcpp/rclcpp.hpp>
#include "tinkerkit_msgs/srv/quaternion_to_euler.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class RobotArmController : public rclcpp::Node
{
public:
    RobotArmController() : Node("robot_arm_control")
    {
        quaternion_subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "target_quaternion", 10, std::bind(&RobotArmController::quaternionCallback, this, std::placeholders::_1));

        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        client_ = this->create_client<tinkerkit_msgs::srv::QuaternionToEuler>("quaternion_to_euler");

        RCLCPP_INFO(this->get_logger(), "Robot Arm Controller Node Started");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Client<tinkerkit_msgs::srv::QuaternionToEuler>::SharedPtr client_;

    void quaternionCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        // Wait for the service
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for quaternion_to_euler service...");
        }

        auto request = std::make_shared<tinkerkit_msgs::srv::QuaternionToEuler::Request>();
        request->x = msg->x;
        request->y = msg->y;
        request->z = msg->z;
        request->w = msg->w;

        auto future_result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == 
            rclcpp::FutureReturnCode::SUCCESS) 
        {
            auto response = future_result.get();
            RCLCPP_INFO(this->get_logger(), "Received Euler Angles: Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                        response->roll, response->pitch, response->yaw);

            // Send joint angles to the robot (assuming a simple mapping)
            auto joint_msg = sensor_msgs::msg::JointState();
            joint_msg.header.stamp = this->now();
            joint_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            joint_msg.position = {response->roll, response->pitch, response->yaw, 0.0, 0.0, 0.0}; // Modify for real kinematics

            joint_publisher_->publish(joint_msg);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call quaternion_to_euler service");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmController>();

    // Use spin() directly instead of manually adding the node to an executor
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
