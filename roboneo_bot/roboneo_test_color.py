#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String, Float32

class RoboneoBotTester(Node):
    def __init__(self):
        super().__init__('roboneo_bot_tester')

        # Create subscriber for color rgb data
        self.rgb_subscriber = self.create_subscription(
            ColorRGBA,
            '/color_sensor/rgb',
            self.rgb_callback,
            10)
        
        # Create subscriber for color raw rgb data
        self.raw_rgb_subscriber = self.create_subscription(
            String,
            '/color_sensor/raw_rgb',
            self.raw_rgb_callback,
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
        self.get_logger().info('Subscribing to /color_sensor/raw_rgb for color name detection')

    def rgb_callback(self, msg):
        """
        Update current rgb values.
        """
        print(f'MAPPED: R: {msg.r}, G: {msg.g}, B: {msg.b}, A: {msg.a}')
        self.get_logger().debug(f'Received R:{msg.r} G:{msg.g} B:{msg.b} A:{msg.a}')

    
    def raw_rgb_callback(self, msg):
        """
        Update current raw rgb values.
        """
        print(f'RAW: {msg.data}')
    
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