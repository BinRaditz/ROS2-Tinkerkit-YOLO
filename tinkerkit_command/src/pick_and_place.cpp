#include <rclcpp/rclcpp.hpp>
// --------------------- Moveit Library
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// --------------------- Transform Frame Library
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick_and_Place");
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class RobotArm {
public:
    RobotArm(rclcpp::Node::SharedPtr node) : node_(node) {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

        executor.add_node(move_group_node);
        std::thread([this]() { this->executor.spin(); }).detach();

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_ARM);
        joint_model_group_arm = move_group_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        
        move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_GRIPPER);
        joint_model_group_gripper = move_group_gripper->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        
        node_->declare_parameter<bool>("is_sim", false);
        
        // Initialize TF2 buffer and listener for object tracking
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(LOGGER, "---------------------------ROBOT ARM INITIALIZED!----------------------------------");
    }

    bool get_is_sim() {
        return node_->get_parameter("is_sim").as_bool();
    }

    void move_joint_space(float angle_0, float angle_1, float angle_2, float angle_3, float angle_4) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE TO JOINT STATE!----------------------------------");
        moveit::core::RobotStatePtr current_state = move_group_arm->getCurrentState(10);

        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

        joint_group_positions[0] = angle_0; // Base Joint   
        joint_group_positions[1] = angle_1; // Shoulder Joint
        joint_group_positions[2] = angle_2; // Elbow Joint 
        joint_group_positions[3] = angle_3; // Arm Joint    
        joint_group_positions[4] = angle_4; // Hand Joint
    
        move_group_arm->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Joint space movement plan failed!");
        }
    }

    void move_end_effector(double x, double y, double z, double roll_deg, double pitch_deg, double yaw_deg) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE END EFFECTOR!----------------------------------");
        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion q;
        q.setRPY(roll_deg * M_PI/180.0, pitch_deg * M_PI/180.0, yaw_deg * M_PI/180.0);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_arm->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "End effector movement plan failed!");
        }
    }

    void move_waypoint(double delta, const std::string& direction) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE WAYPOINT %s: %f----------------------------------", 
                   direction.c_str(), delta);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;

        if(direction == "x") {
            target_pose.position.x += delta;
        } else if(direction == "y") {
            target_pose.position.y += delta;
        } else if(direction == "z") {
            target_pose.position.z += delta;
        } else {
            RCLCPP_ERROR(LOGGER, "Invalid direction!");
            return;
        }

        waypoints.push_back(target_pose);
 
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if(fraction >= 0.0) {
            move_group_arm->execute(trajectory);
        } else {
            RCLCPP_ERROR(LOGGER, "Waypoint movement plan failed!");
        }
    }

    void cmd_arm(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, "---------------------------ARM COMMAND: %s----------------------------------", 
                   target_name.c_str());

        move_group_arm->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group_arm->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_arm) {
            move_group_arm->execute(my_plan_arm);
        } else {
            RCLCPP_ERROR(LOGGER, "Arm plan failed!");
        }
    }

    void cmd_gripper(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, "---------------------------GRIPPER COMMAND: %s----------------------------------", 
                   target_name.c_str());

        move_group_gripper->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        bool success_gripper = (move_group_gripper->plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_gripper) {
            move_group_gripper->execute(my_plan_gripper); 
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper plan failed!");
        }
    }

    void move_gripper_space(double distance) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE GRIPPER JOINTS: %f----------------------------------",
                   distance);

        std::vector<double> joint_group_positions;

        move_group_gripper->getCurrentState()->copyJointGroupPositions(move_group_gripper->getCurrentState()->getRobotModel()->getJointModelGroup(move_group_gripper->getName()), joint_group_positions);

        joint_group_positions[0] = distance; // rg2_gripper_finger_left_joint
        joint_group_positions[1] = 0; // rg2_gripper_thumb_left_joint
        joint_group_positions[2] = 0; // rg2_gripper_finger_right_joint
        joint_group_positions[3] = 0; // rg2_gripper_thumb_right_joint

        move_group_gripper->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_gripper->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_gripper->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper space movement plan failed!");
        }
    }

    void print_end_effector_position() {
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Position: (%f, %f, %f)", 
                   current_pose.pose.position.x, 
                   current_pose.pose.position.y, 
                   current_pose.pose.position.z);
                   
        // Extract orientation as RPY
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Convert to degrees
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;
        
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Orientation (RPY deg): (%f, %f, %f)", roll, pitch, yaw);
    }
    
    void move2xy(double x, double y) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE TO XY: (%f, %f)----------------------------------", 
                   x, y);
                   
        // Get current end effector position
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();

        // Calculate difference in x and y
        double dx = x - current_pose.pose.position.x;
        double dy = y - current_pose.pose.position.y;

        // Define step size
        double step_size = 0.01;

        // Calculate number of steps
        int num_steps_x = std::abs(dx / step_size);
        int num_steps_y = std::abs(dy / step_size);

        // Move end effector to target position in steps
        for (int i = 0; i < num_steps_x; ++i) {
            move_waypoint(dx > 0 ? step_size : -step_size, "x");
        }
        for (int i = 0; i < num_steps_y; ++i) {
            move_waypoint(dy > 0 ? step_size : -step_size, "y");
        }
    }
    
    // Enhanced object position detection with orientation
    bool get_object_position(const std::string& object_frame, double& x, double& y, double& z, 
                            double& roll, double& pitch, double& yaw) {
        RCLCPP_INFO(LOGGER, "---------------------------LOOKING FOR OBJECT: %s----------------------------------", 
                   object_frame.c_str());
                   
        try {
            // Wait for the transform to be available
            geometry_msgs::msg::TransformStamped transform;
            
            // Wait up to 5 seconds for the transform to become available
            if (tf_buffer_->canTransform("world", object_frame, tf2::TimePointZero, tf2::durationFromSec(5.0))) {
                transform = tf_buffer_->lookupTransform("world", object_frame, tf2::TimePointZero);
                
                x = transform.transform.translation.x;
                y = transform.transform.translation.y;
                z = transform.transform.translation.z;
                
                // Convert quaternion to roll, pitch, yaw
                tf2::Quaternion q(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
                tf2::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                
                // Convert to degrees for easier use
                roll = roll * 180.0 / M_PI;
                pitch = pitch * 180.0 / M_PI;
                yaw = yaw * 180.0 / M_PI;
                
                RCLCPP_INFO(LOGGER, "Object found at position: (%f, %f, %f) with orientation (RPY deg): (%f, %f, %f)", 
                           x, y, z, roll, pitch, yaw);
                return true;
            } else {
                RCLCPP_ERROR(LOGGER, "Failed to get transform for object: %s", object_frame.c_str());
                return false;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(LOGGER, "Transform exception: %s", ex.what());
            return false;
        }
    }
    
    // For backward compatibility
    bool get_object_position(const std::string& object_frame, double& x, double& y, double& z) {
        double roll, pitch, yaw;
        return get_object_position(object_frame, x, y, z, roll, pitch, yaw);
    }
    
    // Calculate the best approach orientation based on object type and position
    void calculate_approach_orientation(const std::string& object_type, double obj_x, double obj_y, double obj_z,
                                      double obj_roll, double obj_pitch, double obj_yaw,
                                      double& approach_roll, double& approach_pitch, double& approach_yaw) {
        // Get current end-effector position
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();
        double robot_x = current_pose.pose.position.x;
        double robot_y = current_pose.pose.position.y;
        
        // Calculate direction vector from robot to object in XY plane
        double dx = obj_x - robot_x;
        double dy = obj_y - robot_y;
        
        // Calculate angle in XY plane (this gives the yaw angle to face the object)
        double approach_angle = std::atan2(dy, dx) * 180.0 / M_PI;
        
        if (object_type == "can" || object_type == "cylinder") {
            // For cylindrical objects, approach from the side is often better
            // but can also be grasped from the top depending on the gripper design
            approach_roll = -87.629;  // Approach from top (default)
            approach_pitch = -0.026;
            approach_yaw = obj_yaw;  // Match object's yaw for better alignment
        } else if (object_type == "box") {
            // For boxes, you might want to align with the box edges
            approach_roll = 180.0;  // Approach from top
            approach_pitch = 0.0;
            // Align gripper with box edges for better grasping
            approach_yaw = std::round(obj_yaw / 90.0) * 90.0;  
        } else {
            // Default approach from top
            approach_roll = 180.0;
            approach_pitch = 0.0;
            approach_yaw = 0.0;
        }
        
        RCLCPP_INFO(LOGGER, "Calculated approach orientation (RPY deg): (%f, %f, %f) for object type: %s", 
                   approach_roll, approach_pitch, approach_yaw, object_type.c_str());
    }
    
    // Enhanced picking function with proper orientation
    bool pick_object(const std::string& object_name, const std::string& object_type = "generic") {
        RCLCPP_INFO(LOGGER, "---------------------------PICKING OBJECT: %s (TYPE: %s)----------------------------------", 
                   object_name.c_str(), object_type.c_str());
        
        // Get object position and orientation
        double x, y, z, roll, pitch, yaw;
        if (!get_object_position(object_name, x, y, z, roll, pitch, yaw)) {
            RCLCPP_ERROR(LOGGER, "Failed to get object position");
            return false;
        }
        
        // Add collision object to the planning scene
        add_collision_object(object_name, x, y, z, 0.05, 0.05, 0.15); // Example dimensions for a can
        
        // Calculate best approach orientation
        double approach_roll, approach_pitch, approach_yaw;
        calculate_approach_orientation(object_type, x, y, z, roll, pitch, yaw, 
                                     approach_roll, approach_pitch, approach_yaw);
        
        // Move to pre-grasp position (slightly above the object)
        move_end_effector(x, y, z + 0.15, approach_roll, approach_pitch, approach_yaw);
        
        // Open gripper
        cmd_gripper("open");
        
        // Use cartesian path for more precise approach to the object
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;
        target_pose.position.z = z + 0.02;  // Slightly above object for final approach
        waypoints.push_back(target_pose);
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction >= 0.9) {
            move_group_arm->execute(trajectory);
        } else {
            RCLCPP_WARN(LOGGER, "Cartesian approach planning less optimal (%.2f%%), trying direct movement", fraction*100.0);
            move_waypoint(-0.13, "z");
        }
        
        // Close gripper
        cmd_gripper("close");
        
        // Attach the object to the end-effector
        attach_object(object_name);
        
        // Move up with object
        move_waypoint(0.15, "z");
        
        RCLCPP_INFO(LOGGER, "Successfully picked object: %s", object_name.c_str());
        return true;
    }

    bool place_object(double x, double y, double z, double roll = 180.0, double pitch = 0.0, double yaw = 0.0) {
        RCLCPP_INFO(LOGGER, "---------------------------PLACING OBJECT AT: (%f, %f, %f) with RPY: (%f, %f, %f)----------------------------------", 
                   x, y, z, roll, pitch, yaw);
        
        // Move to pre-place position with specified orientation
        move_end_effector(x, y, z + 0.15, roll, pitch, yaw);
        
        // Use cartesian path for more precise placement
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;
        target_pose.position.z = z + 0.02;  // Slightly above the place position
        waypoints.push_back(target_pose);
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction >= 0.9) {
            move_group_arm->execute(trajectory);
        } else {
            RCLCPP_WARN(LOGGER, "Cartesian placement planning less optimal (%.2f%%), trying direct movement", fraction*100.0);
            move_waypoint(-0.13, "z");
        }
        
        // Detach the object from the end-effector
        detach_object("can");
        
        // Open gripper to release object
        cmd_gripper("open");
        
        // Move up
        move_waypoint(0.15, "z");
        
        RCLCPP_INFO(LOGGER, "Successfully placed object");
        return true;
    }

    void add_collision_object(const std::string& object_id, double x, double y, double z, double size_x, double size_y, double size_z) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_arm->getPlanningFrame();
        collision_object.id = object_id;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = size_x;
        primitive.dimensions[1] = size_y;
        primitive.dimensions[2] = size_z;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        planning_scene_interface.addCollisionObjects(collision_objects);
    }

    void attach_object(const std::string& object_id) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        move_group_arm->attachObject(object_id);
    }

    void detach_object(const std::string& object_id) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        move_group_arm->detachObject(object_id);
    }

    // Perform complete pick and place operation with object detection
    bool pick_and_place_object(const std::string& object_name, const std::string& object_type,
                              double place_x, double place_y, double place_z) {
        // First, try to pick the object
        bool pick_success = pick_object(object_name, object_type);
        
        if (pick_success) {
            // Place the object at the specified location
            return place_object(place_x, place_y, place_z);
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to pick object %s, aborting place operation", object_name.c_str());
            return false;
        }
    }
    
    // Handle detected objects with improved approach based on object type
    bool handle_detected_object(const std::string& object_name, const std::string& object_type, 
                               double place_x = 0.0, double place_y = 0.0, double place_z = 0.1) {
        RCLCPP_INFO(LOGGER, "---------------------------HANDLING DETECTED OBJECT: %s----------------------------------", 
                   object_name.c_str());
        
        // Get object position and orientation
        double x, y, z, roll, pitch, yaw;
        if (!get_object_position(object_name, x, y, z, roll, pitch, yaw)) {
            RCLCPP_ERROR(LOGGER, "Failed to get object position");
            return false;
        }
        
        // Record detected position
        RCLCPP_INFO(LOGGER, "Detected object at (%f, %f, %f) with orientation (%f, %f, %f)",
                   x, y, z, roll, pitch, yaw);
                   
        // Calculate best approach orientation
        double approach_roll, approach_pitch, approach_yaw;
        calculate_approach_orientation(object_type, x, y, z, roll, pitch, yaw, 
                                      approach_roll, approach_pitch, approach_yaw);
        
        // Approach and pick the object with the calculated orientation
        // Move to a safe position above the object
        move_end_effector(x, y, z + 0.15, approach_roll, approach_pitch, approach_yaw);
        
        // Open gripper
        cmd_gripper("open");
        
        // Approach object carefully
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;
        target_pose.position.z = z + 0.02;  // Slightly above object for final approach
        waypoints.push_back(target_pose);
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction < 0.9) {
            RCLCPP_WARN(LOGGER, "Path planning for approach less optimal (%.2f%%), trying direct movement", fraction*100.0);
            move_waypoint(-0.13, "z");
        } else {
            move_group_arm->execute(trajectory);
        }
        
        // Close gripper
        cmd_gripper("close");
        
        // Move up with object
        move_waypoint(0.15, "z");
        
        // Place the object at the specified location
        if (place_x != 0.0 || place_y != 0.0 || place_z != 0.1) {
            // Use provided place coordinates
            place_object(place_x, place_y, place_z, approach_roll, approach_pitch, approach_yaw);
        } else if (get_is_sim()) {
            // Use default simulation place position
            place_object(0.2, 0.2, 0.1, approach_roll, approach_pitch, approach_yaw);
        } else {
            // Use default real robot place position
            place_object(0.3, 0.0, 0.1, approach_roll, approach_pitch, approach_yaw);
        }
        
        RCLCPP_INFO(LOGGER, "Successfully handled object: %s", object_name.c_str());
        return true;
    }

private:
    rclcpp::Node::SharedPtr move_group_node;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;
    const moveit::core::JointModelGroup* joint_model_group_arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group_gripper;
    rclcpp::executors::SingleThreadedExecutor executor;
    bool is_sim;
    rclcpp::Node::SharedPtr node_;
    
    // TF2 components for object tracking
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pick_place_can");
    RobotArm robotArm(node);
    
    // Wait a moment for TF2 to initialize
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Move to home position
    robotArm.cmd_arm("home");
    
    // Print current position and orientation
    robotArm.print_end_effector_position();
    
    // Move to a position ready for picking
    robotArm.move_joint_space(0.0, 0.767945, -1.72788, -0.593412, 0.0);
    
    // Get object position and handle picking with proper orientation
    bool handle_success = robotArm.handle_detected_object("can", "can");
    
    if (!handle_success) {
        // Fallback to basic pick and place if enhanced handling fails
        RCLCPP_WARN(LOGGER, "Enhanced object handling failed, trying basic pick operation...");
        
        // Get object position
        double obj_x, obj_y, obj_z;
        if (robotArm.get_object_position("can", obj_x, obj_y, obj_z)) {
            // Move to pre-grasp position
            robotArm.move_end_effector(-1.033, 1.020, 1.355, -79.444, 0.0, 45.364);
            
            // Open gripper
            robotArm.cmd_gripper("open");
            
            // Move down to object
            robotArm.move_waypoint(-0.14, "z");
            
            // Close gripper
            robotArm.cmd_gripper("close");
            
            // Move up with object
            robotArm.move_waypoint(0.15, "z");
            
            // Place object
            if (robotArm.get_is_sim()) {
                robotArm.place_object(0.2, 0.2, 0.1);
            } else {
                robotArm.place_object(0.3, 0.0, 0.1);
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Could not detect object position!");
        }
    }
    
    // Return to home position
    robotArm.cmd_arm("home");
    
    RCLCPP_INFO(LOGGER, "Pick and place operation completed!");
    
    rclcpp::shutdown();
    return 0;
}