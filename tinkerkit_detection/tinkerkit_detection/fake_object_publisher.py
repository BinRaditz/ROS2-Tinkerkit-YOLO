#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped

class FakeObjectPublisher(Node):
    def __init__(self):
        super().__init__('fake_object_publisher')
        
        # Parameters (you can adjust these as needed)
        self.frame_id = "world"
        self.object_name = "can"
        # Explicitly define position values as floats
        self.object_position = {"x": 0.0, "y": 3.0, "z": 0.6}
        
        # Cylinder dimensions (radius and height)
        self.cylinder_radius = 0.4  # 4cm radius
        self.cylinder_height = 1.2  # 12cm height
        
        # Set up TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up marker publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.publish_object)
        
        self.get_logger().info("Fake object publisher initialized. Publishing a can at position: "
                      f"x={self.object_position['x']}, y={self.object_position['y']}, z={self.object_position['z']}")
        
    def publish_object(self):
        # Publish TF2 transform
        self.publish_tf()
        
        # Publish marker for RViz
        self.publish_marker()
        
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

def main(args=None):
    rclpy.init(args=args)
    node = FakeObjectPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()