#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray  # Adjust message types if needed

class AEBControllerNode(Node):
    def __init__(self):
        super().__init__('aeb_controller')

        # Constants and PID parameters
        self.max_throttle = 100.0
        self.max_brake = 100.0
        self.emergency_brake_ttc = 1.0
        self.min_safety_ttc = 1.2
        self.max_safety_ttc = 3.5
        self.vehicle_width = 1.5
        self.kp = 200.0
        self.ki = 50.0
        self.kd = 20.0
        self.dt = 0.1

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # Subscribers and Publishers
        self.create_subscription(
            Float32,  # Input message type
            'fused_data',       # Input topic
            self.callback_fused_data,
            10
        )

        self.throttle_publisher = self.create_publisher(Float32, 'throttle', 10)
        self.brake_publisher = self.create_publisher(Float32, 'brake', 10)

    def callback_fused_data(self, msg):
        """
        Callback to process fused data and compute control actions.
        """
        try:
            # Extract data from the incoming message
            target_velocity = msg.data[0]
            target_width = msg.data[1]
            target_pos = [msg.data[2], msg.data[3]]
            ego_velocity = msg.data[4]
            distance = msg.data[5]
            traveled_distance = msg.data[6]

            # Compute throttle and brake values using the AEB algorithm
            throttle, brake = self.aeb_algorithm(
                target_velocity, target_width, target_pos, ego_velocity, distance, traveled_distance
            )

            # Publish computed control values
            self.publish_control(throttle, brake)
        except IndexError as e:
            self.get_logger().error(f"Error processing fused_data: {e}")

    def aeb_algorithm(self, target_velocity, target_width, target_pos, ego_velocity, distance, traveled_distance):
        """
        Implements the AEB algorithm to calculate throttle and brake control.
        """
        y_target = target_pos[1]
        lateral_distance = abs(y_target)

        # Calculate overlap factor
        overlap = max(0, (self.vehicle_width + target_width - lateral_distance) / (self.vehicle_width + target_width))
        overlap = min(overlap, 1)

        # Case 1: Accelerate if safe and vehicle hasn't traveled far
        if traveled_distance < 100 and ego_velocity < target_velocity:
            return self.max_throttle, 0.0

        # Calculate Time to Collision (TTC)
        rel_vel = ego_velocity - target_velocity
        ttc = distance / rel_vel if rel_vel > 0 and distance > 0 else float('inf')

        # Dynamic safety TTC based on overlap
        dynamic_safety_ttc = self.min_safety_ttc + (1 - overlap) * (self.max_safety_ttc - self.min_safety_ttc)

        # Case 2: Emergency brake
        if ttc < self.emergency_brake_ttc:
            return 0.0, self.max_brake

        # Case 3: Gradual braking with PID control
        if ttc < dynamic_safety_ttc:
            error = dynamic_safety_ttc - ttc
            proportional = self.kp * error
            self.integral += self.ki * error * self.dt
            derivative = self.kd * (error - self.prev_error) / self.dt
            pid_output = proportional + self.integral + derivative
            self.prev_error = error
            brake = min(self.max_brake, max(0.0, pid_output))
            return 0.0, brake

        # Case 4: Maintain or accelerate if safe
        return self.max_throttle, 0.0

    def publish_control(self, throttle, brake):
        """
        Publishes throttle and brake control commands.
        """
        # Prepare throttle and brake messages
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        brake_msg = Float32()
        brake_msg.data = float(brake)

        # Publish messages
        self.throttle_publisher.publish(throttle_msg)
        self.brake_publisher.publish(brake_msg)

def main(args=None):
    """
    Main entry point for the AEB Controller Node.
    """
    rclpy.init(args=args)
    aeb_node = AEBControllerNode()

    try:
        rclpy.spin(aeb_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        aeb_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
