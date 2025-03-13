#include "rclcpp/rclcpp.hpp"

class RobotArmNode : public rclcpp::Node
{
public:
    RobotArmNode() : Node("robotarm_node")
    {
        this->declare_parameter<double>("joint1_limit", 1.57);
        this->declare_parameter<double>("joint2_limit", 3.14);
        this->declare_parameter<double>("joint3_limit", 1.57);
        this->declare_parameter<double>("velocity_limit", 0.5);
        this->declare_parameter<double>("control_gains.p_gain", 1.0);
        this->declare_parameter<double>("control_gains.i_gain", 0.01);
        this->declare_parameter<double>("control_gains.d_gain", 0.001);
        
        // Load parameters
        joint1_limit_ = this->get_parameter("joint1_limit").as_double();
        joint2_limit_ = this->get_parameter("joint2_limit").as_double();
        joint3_limit_ = this->get_parameter("joint3_limit").as_double();
        velocity_limit_ = this->get_parameter("velocity_limit").as_double();
        p_gain_ = this->get_parameter("control_gains.p_gain").as_double();
        i_gain_ = this->get_parameter("control_gains.i_gain").as_double();
        d_gain_ = this->get_parameter("control_gains.d_gain").as_double();

        RCLCPP_INFO(this->get_logger(), "Joint 1 limit: %.2f", joint1_limit_);
        RCLCPP_INFO(this->get_logger(), "Joint 2 limit: %.2f", joint2_limit_);
        RCLCPP_INFO(this->get_logger(), "Joint 3 limit: %.2f", joint3_limit_);
        RCLCPP_INFO(this->get_logger(), "Velocity limit: %.2f", velocity_limit_);
        RCLCPP_INFO(this->get_logger(), "Control P Gain: %.2f", p_gain_);
        RCLCPP_INFO(this->get_logger(), "Control I Gain: %.2f", i_gain_);
        RCLCPP_INFO(this->get_logger(), "Control D Gain: %.2f", d_gain_);
    }

private:
    double joint1_limit_;
    double joint2_limit_;
    double joint3_limit_;
    double velocity_limit_;
    double p_gain_;
    double i_gain_;
    double d_gain_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotArmNode>());
    rclcpp::shutdown();
    return 0;
}
