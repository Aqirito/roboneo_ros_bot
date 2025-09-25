#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RoboneoBotTester(Node):
    def __init__(self):
        super().__init__('roboneo_bot')
        # Create subscriber for lidar distance data
        self.lidar_distance_subscriber = self.create_subscription(
            Float32,
            '/tfluna/distance',
            self.lidar_distance_callback,
            10)
        
        # Initial values
        self.lidar_distance = 0.0

        # Log startup info
        self.get_logger().info('Roboneo Bot Tester started')
        self.get_logger().info('Subscribing to /tfluna/distance for pillar detection')

    def lidar_distance_callback(self, msg):
        """
        Update current lidar distance.
        """
        self.lidar_distance = msg.data
        print(f'Lidar Distance: {self.lidar_distance} cm')
        self.get_logger().debug(f'Received lidar distance: {self.lidar_distance:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    
    node = RoboneoBotTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()