#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import ast
import math
from rclpy.time import Time

class ObjectTF2Publisher(Node):
    def __init__(self):
        super().__init__('object_tf2_publisher')
        
        # Create subscription to bounding boxes topic
        self.subscription = self.create_subscription(
            String,
            'bounding_boxes',
            self.bbox_callback,
            10
        )
        
        # Initialize the TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Dictionary to store camera parameters
        self.camera_params = {
            'fx': 640.0,  # Focal length x
            'fy': 640.0,  # Focal length y
            'cx': 320.0,  # Principal point x
            'cy': 240.0,  # Principal point y
            'height': 480,  # Camera image height
            'width': 640   # Camera image width
        }
        
        # Approximate depth for demonstration (in meters)
        self.default_depth = 0.5
        
        self.get_logger().info('Object TF2 Publisher has started')

    def pixel_to_3d(self, pixel_x, pixel_y):
        """Convert pixel coordinates to approximate 3D coordinates."""
        # Calculate normalized image coordinates
        x_normalized = (pixel_x - self.camera_params['cx']) / self.camera_params['fx']
        y_normalized = (pixel_y - self.camera_params['cy']) / self.camera_params['fy']
        
        # Calculate 3D coordinates using the default depth
        x = x_normalized * self.default_depth
        y = y_normalized * self.default_depth
        z = self.default_depth
        
        return x, y, z

    def publish_transform(self, x, y, z, object_id, confidence):
        """Publish TF2 transform for detected object."""
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        
        # Set child frame ID using object ID and confidence
        t.child_frame_id = f'object_{object_id}_{confidence:.2f}'
        
        # Set transform translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Set transform rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(
            f'Published transform for {t.child_frame_id} at position '
            f'({x:.3f}, {y:.3f}, {z:.3f})'
        )

    def bbox_callback(self, msg):
        """Process incoming bounding box messages."""
        try:
            # Parse bounding boxes from message
            bounding_boxes = ast.literal_eval(msg.data)
            
            for bbox in bounding_boxes:
                x1, y1, x2, y2, class_id, confidence = bbox
                
                # Calculate center point of bounding box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                # Convert pixel coordinates to 3D coordinates
                x, y, z = self.pixel_to_3d(center_x, center_y)
                
                # Publish TF2 transform for this object
                self.publish_transform(x, y, z, class_id, confidence)
                
        except Exception as e:
            self.get_logger().error(f'Error processing bounding boxes: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTF2Publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()