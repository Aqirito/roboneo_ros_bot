#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class RoboneoBotTester(Node):
    def __init__(self):
        super().__init__('roboneo_bot_tester')
        
        # Create publisher for Twist messages
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for ultrasonic distance data
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/ultrasonic/distance',
            self.distance_callback,
            10)
        
        self.get_logger().info('Roboneo Bot Tester started')
        self.get_logger().info('Publishing Twist messages to /cmd_vel')
        self.get_logger().info('Subscribing to distance data on /ultrasonic/distance')

    def send_twist_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        self.twist_publisher.publish(msg)
        self.get_logger().info(f'Published Twist: linear.x={linear_x}, angular.z={angular_z}')

    def distance_callback(self, msg):
        self.get_logger().info(f'Received distance: {msg.data:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    tester = RoboneoBotTester()
    
    # Send a series of test commands
    try:
        # Move forward
        tester.send_twist_command(0.7, 0.0)
        time.sleep(3)
        
        # Turn left
        tester.send_twist_command(0.0, 0.7)
        time.sleep(3)
        
        # Move backward
        tester.send_twist_command(-0.7, 0.0)
        time.sleep(3)
        
        # Turn right
        tester.send_twist_command(0.0, -0.7)
        time.sleep(3)

        # Move forward while turning left
        tester.send_twist_command(0.7, 0.7)
        time.sleep(3)

        # Move backward while turning right
        tester.send_twist_command(-0.7, 0.7)
        time.sleep(3)

        # Move backward while turning left
        tester.send_twist_command(-0.7, -0.7)
        time.sleep(3)

        # Diagonal movement
        tester.send_twist_command(1.0, 0.7)
        time.sleep(3)
        
        # Stop
        tester.send_twist_command(0.0, 0.0)
        time.sleep(1)

        # receive distance data for 10 times
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=1)
            time.sleep(.5)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
