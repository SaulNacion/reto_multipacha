#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MessagePublisherNode(Node):
    def __init__(self):
        super().__init__('message_publisher')
        self.publisher_ = self.create_publisher(String, 'message_publisher', 10)
        self.timer_ = self.create_timer(0.1, self.publish_message)
        self.get_logger().info('Message Publisher Node has been started.')

    def publish_message(self):
        msg = String()
        msg.data = 'M6 Reto'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MessagePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()