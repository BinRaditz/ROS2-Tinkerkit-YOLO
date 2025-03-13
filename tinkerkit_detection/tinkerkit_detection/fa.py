#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point
import math

# Import moveit messages for collision objects
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class FakeObjectPublisherWithCollision(Node):
    def __init__(self):
        super().__init__('fake_object_publisher')
        
        # Use callback group to allow service calls during timer callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters (you can adjust these as needed)
        self.frame_id = "world"
        self.object_name = "can"
        # Explicitly define position values as floats
        self.object_position = {"x": -2.0, "y": 2.0, "z": 0.6}
        
        # Cylinder dimensions (radius and height)
        self.cylinder_radius = 0.4  # 40cm radius
        self.cylinder_height = 1.2  # 120cm height
        
        # Set up TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up marker publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Add collision object publisher for MoveIt
        self.collision_pub = self.create_publisher(
            CollisionObject, 
            '/collision_object', 
            10
        )
        
        # Create list to track collision objects in the scene
        self.collision_objects = {}
        
        # Timer for publishing can at 10Hz
        self.timer = self.create_timer(
            0.1, 
            self.publish_object, 
            callback_group=self.callback_group
        )
        
        # Timer for collision checking at 5Hz
        self.collision_timer = self.create_timer(
            0.2, 
            self.check_collisions,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Fake object publisher initialized. Publishing a can at position: "
                      f"x={self.object_position['x']}, y={self.object_position['y']}, z={self.object_position['z']}")
        
    def publish_object(self):
        # Publish TF2 transform
        self.publish_tf()
        
        # Publish marker for RViz
        self.publish_marker()
        
        # Update collision object
        self.update_can_collision_object()
        
    def publish_tf(self):
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        
        # Child frame ID
        transform.child_frame_id = self.object_name
        
        # Translation - explicitly convert all values to float
        transform.transform.translation.x = float(self.object_position["x"])
        transform.transform.translation.y = float(self.object_position["y"])
        transform.transform.translation.z = float(self.object_position["z"])
        
        # Rotation (identity quaternion - no rotation)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)
        
    def publish_marker(self):
        marker = Marker()
        
        # Header
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Namespace and ID
        marker.ns = "fake_objects"
        marker.id = 0
        
        # Shape
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position - explicitly convert all values to float
        marker.pose.position.x = float(self.object_position["x"])
        marker.pose.position.y = float(self.object_position["y"])
        marker.pose.position.z = float(self.object_position["z"])
        
        # Orientation (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale - explicitly convert to float
        marker.scale.x = float(self.cylinder_radius * 2)  # Diameter in x
        marker.scale.y = float(self.cylinder_radius * 2)  # Diameter in y
        marker.scale.z = float(self.cylinder_height)      # Height in z
        
        # Color (red with some transparency)
        marker.color.r = 0.8
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.8
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # Half-second lifetime (500ms)
        
        # Publish marker
        self.marker_pub.publish(marker)
    
    def update_can_collision_object(self):
        # Create a collision object for the can
        can = CollisionObject()
        can.header.frame_id = self.frame_id
        can.id = self.object_name
        
        # Define the cylinder primitive
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [self.cylinder_height, self.cylinder_radius]  # height, radius
        
        # Set the pose
        can.primitives = [cylinder]
        can.primitive_poses = [self.create_pose(
            self.object_position["x"], 
            self.object_position["y"], 
            self.object_position["z"]
        )]
        can.operation = CollisionObject.ADD
        
        # Publish the collision object
        self.collision_pub.publish(can)
        
        # Store in our tracking dictionary
        self.collision_objects[self.object_name] = {
            "position": self.object_position.copy(),
            "dimensions": {"radius": self.cylinder_radius, "height": self.cylinder_height},
            "type": "cylinder"
        }
    
    def create_pose(self, x, y, z):
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        return pose
    
    def check_collisions(self):
        # Check for collisions between can and other objects
        collisions = []
        
        # Only check for collisions if we have the can
        if self.object_name in self.collision_objects:
            can_pos = self.collision_objects[self.object_name]["position"]
            can_dim = self.collision_objects[self.object_name]["dimensions"]
            
            # Check against each object
            for obj_id, obj_data in self.collision_objects.items():
                if obj_id != self.object_name:
                    # Simple collision check between cylinder and other objects
                    if self.check_cylinder_box_collision(
                        can_pos, 
                        can_dim["radius"], 
                        can_dim["height"],
                        obj_data["position"],
                        obj_data["dimensions"]
                    ):
                        collisions.append(obj_id)
        
        # Report any collisions
        if collisions:
            self.get_logger().warning(f"Collision detected between {self.object_name} and {', '.join(collisions)}")
            
            # We could also change the color of the can to indicate collision
            self.update_marker_color_on_collision(True)
        else:
            # Reset marker color when no collisions
            self.update_marker_color_on_collision(False)
    
    def check_cylinder_box_collision(self, cyl_pos, cyl_radius, cyl_height, box_pos, box_dim):
        """
        Simple collision detection between a vertical cylinder and a box
        """
        # Get cylinder bottom and top z coordinates
        cyl_bottom_z = cyl_pos["z"] - cyl_height/2
        cyl_top_z = cyl_pos["z"] + cyl_height/2
        
        # Get box min and max z coordinates
        box_min_z = box_pos["z"] - box_dim["z"]/2
        box_max_z = box_pos["z"] + box_dim["z"]/2
        
        # Check vertical overlap
        if cyl_top_z < box_min_z or cyl_bottom_z > box_max_z:
            return False  # No vertical overlap
        
        # Check horizontal distance between cylinder center and box
        # First, find closest point on box to cylinder center in XY plane
        closest_x = max(box_pos["x"] - box_dim["x"]/2, 
                       min(cyl_pos["x"], box_pos["x"] + box_dim["x"]/2))
        closest_y = max(box_pos["y"] - box_dim["y"]/2, 
                       min(cyl_pos["y"], box_pos["y"] + box_dim["y"]/2))
        
        # Calculate distance squared from cylinder center to closest point on box
        dist_squared = (closest_x - cyl_pos["x"])**2 + (closest_y - cyl_pos["y"])**2
        
        # Collision if this distance is less than cylinder radius
        return dist_squared < cyl_radius**2
    
    def update_marker_color_on_collision(self, is_colliding):
        marker = Marker()
        
        # Header
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Namespace and ID
        marker.ns = "fake_objects"
        marker.id = 0
        
        # Shape
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position - explicitly convert all values to float
        marker.pose.position.x = float(self.object_position["x"])
        marker.pose.position.y = float(self.object_position["y"])
        marker.pose.position.z = float(self.object_position["z"])
        
        # Orientation (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale - explicitly convert to float
        marker.scale.x = float(self.cylinder_radius * 2)
        marker.scale.y = float(self.cylinder_radius * 2)
        marker.scale.z = float(self.cylinder_height)
        
        # Color (change depending on collision state)
        if is_colliding:
            # Bright yellow for collision
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            # Original red with transparency
            marker.color.r = 0.8
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.8
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # Half-second lifetime (500ms)
        
        # Publish marker
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FakeObjectPublisherWithCollision()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()