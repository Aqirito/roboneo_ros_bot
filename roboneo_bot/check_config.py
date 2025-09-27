#!/usr/bin/env python3

"""
Test script to verify configuration loading functionality.
"""

import rclpy
from rclpy.node import Node
from .config import load_config

class ConfigNode(Node):
    def __init__(self):
        super().__init__('config_node')
        self.config = load_config()
        self.get_logger().info('Configuration loaded successfully:')
        for key, value in self.config.items():
            self.get_logger().info(f'  {key}: {value}')

def main(args=None):
    rclpy.init(args=args)
    node = ConfigNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
