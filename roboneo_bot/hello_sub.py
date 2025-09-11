#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/esp32/hello',
            self.hello_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Hello Subscriber started, listening on /esp32/hello')

    def hello_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    hello_subscriber = HelloSubscriber()
    
    try:
        rclpy.spin(hello_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        hello_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()