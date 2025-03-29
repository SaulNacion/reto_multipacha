#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MessageSubscriberNode(Node):
    def __init__(self):
        super().__init__("message_subscriber")
        self.subscriber_ = self.create_subscription(
            String, "message_publisher", self.callback_message, 10)
        self.get_logger().info("Message Subscriber Node has been started.")

    def callback_message(self, msg:String):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MessageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()