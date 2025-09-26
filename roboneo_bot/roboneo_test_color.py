#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
import requests
from datetime import datetime

class RoboneoBotTester(Node):
    def __init__(self):
        super().__init__('roboneo_bot')

        self.url = 'http://api_url_here/color_detection'
        self.secret_key = 'secret_key_here'
        self.robot_id = 'robot_id_here'

        self.detected_color = ""

        # Create subscriber for color rgb data
        self.rgb_subscriber = self.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.rgb_callback,
            10)
        
        # Create subscriber for color rgb data
        self.color_name_subscriber = self.create_subscription(
            String,
            '/color_sensor/color_name',
            self.color_name_callback,
            10)
            
        # Log startup info
        self.get_logger().info('Roboneo Bot Tester started')
        self.get_logger().info('Subscribing to /color_sensor/rgb for color detection')
        self.get_logger().info('Subscribing to /color_sensor/color_name for color name detection')

    def rgb_callback(self, msg):
        """
        Update current rgb values.
        """
        print(f'R: {msg.r}, G: {msg.g}, B: {msg.b}, A: {msg.a}')
        self.get_logger().debug(f'Received R:{msg.r} G:{msg.g} B:{msg.b} A:{msg.a}')
    
    def color_name_callback(self, msg):
        """
        Update current color name.
        """
        print(f'Color Name: {msg.data}')
        self.get_logger().debug(f'Received Color Name: {msg.data}')

        match(msg.data):
            case "Red":
                self.get_logger().info("🔴 Detected Red Color")
                self.detected_color = "Red"
            case "Green":
                self.get_logger().info("🟢 Detected Green Color")
                self.detected_color = "Green"
            case "Blue":
                self.get_logger().info("🔵 Detected Blue Color")
                self.detected_color = "Blue"
            case "Yellow":
                self.get_logger().info("🟡 Detected Yellow Color")
                self.detected_color = "Yellow"
            case "White":
                self.get_logger().info("⚪ Detected White Color")
                self.detected_color = "White"
            case "Black":
                self.get_logger().info("⚫ Detected Black Color")
                self.detected_color = "Black"
            case _:
                self.get_logger().info("❓ Detected Unknown Color")

        if self.detected_color == "Red":
            payload = {
                        "robot_id": self.robot_id,
                        "color": "Red",
                        "device_timestamp": datetime.now().isoformat()
                    }
            self.send_http_requests(payload)
    

    def send_http_requests(self, payload):
        """
        Send an HTTP POST with json data.
        example payload json data: {"color": "Red"}
        """
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'x-secret-key': self.secret_key
            }
        
        try:
            response = requests.post(self.url, headers=headers, json=payload, timeout=3)
            if response.status_code == 200:
                self.get_logger().info(f'HTTP POST to {self.url} succeeded: {response.text}')
            else:
                self.get_logger().error(f'HTTP POST to {self.url} failed with status code {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'HTTP POST to {self.url} failed: {e}')

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