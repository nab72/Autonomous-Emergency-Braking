#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ControlSubscriberNode(Node):
    def __init__(self):
        super().__init__('control_subscriber')

        self.throttle_sub = self.create_subscription(
            Float32,
            'throttle',
            self.throttle_callback,
            10
        )
        self.brake_sub = self.create_subscription(
            Float32,
            'brake',
            self.brake_callback,
            10
        )

    def throttle_callback(self, msg):
        self.get_logger().info(f"Throttle: {msg.data}")

    def brake_callback(self, msg):
        self.get_logger().info(f"Brake: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
