#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        
        # Add retry logic for camera initialization
        self.init_camera()
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/60, self.publish_image)  # 10 Hz
        self.consecutive_failures = 0
        self.max_failures = 5  # Maximum number of consecutive failures before reinit

    def init_camera(self):
        self.get_logger().info('Initializing camera...')
        self.cap = cv2.VideoCapture(0)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)  # Increase brightness (0.0 to 1.0 or higher)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 0.5)  # Adjust contrast if needed
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -4)  # Lower exposure (some cameras need negative values)

        
        # Wait for camera to initialize
        time.sleep(2)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return False
        return True

    def publish_image(self):
        if not self.cap.isOpened():
            self.get_logger().error('Camera is not opened')
            if self.init_camera():
                self.get_logger().info('Camera reinitialized successfully')
            return

        # Try to read frame multiple times if failed
        for _ in range(3):
            ret, frame = self.cap.read()
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing image')
                    self.consecutive_failures = 0
                    return
                except Exception as e:
                    self.get_logger().error(f'Error converting/publishing image: {str(e)}')
            time.sleep(0.1)  # Short delay between retries

        # If we get here, all attempts failed
        self.consecutive_failures += 1
        self.get_logger().warn('Failed to capture image')
        
        # If too many consecutive failures, try to reinitialize camera
        if self.consecutive_failures >= self.max_failures:
            self.get_logger().warn('Too many consecutive failures, reinitializing camera...')
            self.cap.release()
            self.init_camera()
            self.consecutive_failures = 0

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()