#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast  # ใช้ในการแปลงข้อความ bounding box เป็นรายการข้อมูล

class BoundingBoxProcessor(Node):
    def __init__(self):
        super().__init__('bounding_box_processor')
        
        # สร้าง Subscriber เพื่อติดตามข้อมูล bounding box จาก topic
        self.subscription = self.create_subscription(
            String,
            'bounding_boxes',
            self.callback,
            10)

    def callback(self, msg):
        # รับข้อมูล bounding box เป็นข้อความและแปลงเป็นรายการ
        bounding_boxes = ast.literal_eval(msg.data)

        for bbox in bounding_boxes:
            x1, y1, x2, y2, class_id, confidence = bbox

            # คำนวณจุดศูนย์กลางของ bounding box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # แสดงข้อมูลใน terminal
            self.get_logger().info(
                f"Object {class_id} with confidence {confidence:.2f} at center (x: {center_x}, y: {center_y})"
            )

def main(args=None):
    rclpy.init(args=args)
    bounding_box_processor = BoundingBoxProcessor()

    try:
        rclpy.spin(bounding_box_processor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()