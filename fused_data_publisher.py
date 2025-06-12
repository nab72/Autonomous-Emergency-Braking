#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class DataPublisher(Node):
    def __init__(self):
        super().__init__('fused_data_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/fused_data', 10)

    def publish_series(self):
        scenarios = [
	    [2.0, 1.5, 0.0, 0.2, 4.0, 10.0, 100.0],
	    [5.0, 1.5, 0.0, 1.5, 12.0, 9.0, 110.0],
	    [0.0, 1.5, 0.0, 0.0, 10.0, 10.0, 130.0],
	    [0.0, 1.5, 0.0, 0.2, 3.0, 5.0, 150.0],
	    [5.0, 1.5, 0.0, 0.0, 12.0, 8.0, 110.0],# Add more scenarios as needed
        ]

        for scenario in scenarios:
            msg = Float32MultiArray()
            msg.data = scenario
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
            time.sleep(1)  # Pause to allow the AEB node to process each message

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    try:
        node.publish_series()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
