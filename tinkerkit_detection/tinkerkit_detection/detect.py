#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import tf2_ros

class CameraDetectionNode(Node):
    def __init__(self):
        super().__init__('camera_detection_node')
        
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            100)
            
        # Create publishers
        self.bbox_publisher = self.create_publisher(String, 'detection_boxes', 100)
        
        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize CvBridge
        self.bridge = CvBridge() 
        
        # Load YOLO model
        self.model = YOLO("/home/binn/robot_ws/manipulation/tinkerkit/src/object_detection/object_detection/lated.pt")
        
        # Camera intrinsic parameters from calibration
        self.camera_matrix = np.array([
            [530.433222, 0.000000, 322.125991],
            [0.000000, 529.753688, 245.805633],
            [0.000000, 0.000000, 1.000000]
        ])
        
        # Update camera parameters from calibration
        self.fx = self.camera_matrix[0, 0]  # focal length x
        self.fy = self.camera_matrix[1, 1]  # focal length y
        self.cx = self.camera_matrix[0, 2]  # optical center x
        self.cy = self.camera_matrix[1, 2]  # optical center y
        
        # Distortion coefficients from calibration
        self.dist_coeffs = np.array([0.164113,-0.312223,-0.001316,-0.007928,0.000000])
        
        # Fixed Z distance in meters (you may want to adjust this based on your setup)
        self.fixed_z = 0.3
        
        # Store the latest image
        self.latest_image = None
        self.latest_header = None
        
        # Create timer for detection publishing 
        self.detection_timer = self.create_timer(1.0, self.detection_callback)
        
        self.get_logger().info('Camera detection node initialized with 5-second timer')

    def image_callback(self, msg):
        try:
            # Store the latest image and header
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def detection_callback(self):
        if self.latest_image is None:
            self.get_logger().warn('No image received yet')
            return
            
        try:
            # Undistort the image using calibration parameters
            undistorted_image = cv2.undistort(self.latest_image, self.camera_matrix, self.dist_coeffs)
            
            # Run detection on undistorted image
            results = self.model(undistorted_image)
            
            bounding_boxes = []
            for result in results[0].boxes:
                # Get bounding box coordinates, class id, and confidence
                x1, y1, x2, y2 = map(int, result.xyxy[0])
                class_id = int(result.cls[0])
                confidence = float(result.conf[0])
                class_name = self.model.names[class_id]
                
                # Store detection results
                bounding_boxes.append({
                    'coordinates': (x1, y1, x2, y2),
                    'class': class_name,
                    'confidence': confidence
                })
                
                # Draw bounding box
                cv2.rectangle(undistorted_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Add label
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(undistorted_image, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Calculate center of bounding box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Convert pixel coordinates to meters using calibrated parameters
                X = (center_x - self.cx) * self.fixed_z / self.fx
                Y = (center_y - self.cy) * self.fixed_z / self.fy
                Z = self.fixed_z
                
                # Publish transform
                self.publish_transform(X, Y, Z, class_name, self.latest_header.stamp)
            
            # Publish bounding box data
            bbox_msg = String()
            bbox_msg.data = str(bounding_boxes)
            self.bbox_publisher.publish(bbox_msg)
            
            # Display undistorted image with detections
            cv2.imshow("Detections", undistorted_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {str(e)}')

    def publish_transform(self, x, y, z, object_name, timestamp):
        t = TransformStamped()
        
        t.header.stamp = timestamp
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"object_{object_name}"
        
        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Set rotation (identity rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()