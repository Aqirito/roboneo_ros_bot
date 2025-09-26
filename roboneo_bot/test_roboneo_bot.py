#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import requests

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

    def send_http_requests(self, payload):
        """
        Send an HTTP POST with json data.
        example json data: {"key": "value"}
        """
        url = 'http://api_url_here/color_detection'
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'x-secret-key': 'secret_key_here'
            }
        
        try:
            response = requests.post(url, headers=headers, json=payload, timeout=3)
            if response.status_code == 200:
                self.get_logger().info(f'HTTP POST to {url} succeeded: {response.text}')
            else:
                self.get_logger().error(f'HTTP POST to {url} failed with status code {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'HTTP POST to {url} failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    tester = RoboneoBotTester()
    time.sleep(1)
    
    # Send a series of test commands
    try:
        # Move forward
        tester.send_twist_command(0.7, 0.0)
        tester.send_http_requests({"key": "value"})
        time.sleep(2)
        
        # Turn left
        tester.send_twist_command(0.0, 1.0)
        time.sleep(2)
        
        # Move backward
        tester.send_twist_command(-0.7, 0.0)
        time.sleep(2)
        
        # Turn right
        tester.send_twist_command(0.0, -0.7)
        time.sleep(2)

        # Move forward while turning left
        tester.send_twist_command(0.7, 0.7)
        time.sleep(2)

        # Move forward while turning right
        tester.send_twist_command(0.7, -0.7)
        time.sleep(2)
        
        # Stop
        tester.send_twist_command(0.0, 0.0)
        time.sleep(2)

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
