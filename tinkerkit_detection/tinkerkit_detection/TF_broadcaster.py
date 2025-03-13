import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class StaticTF2Broadcaster(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Parent frame
        t.child_frame_id = 'can'         # Object frame
        
        # Set the position of the can relative to base_link
        t.transform.translation.x = 0.5  # Change this to the actual can position
        t.transform.translation.y = 0.2
        t.transform.translation.z = 0.3

        # Set the orientation (quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published static transform from base_link to can.")

def main():
    rclpy.init()
    node = StaticTF2Broadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
