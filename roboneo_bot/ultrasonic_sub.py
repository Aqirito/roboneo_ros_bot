#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/ultrasonic/distance',
            self.ultrasonic_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Ultrasonic Subscriber started, listening on /ultrasonic/distance')

    def ultrasonic_callback(self, msg):
        self.get_logger().info(f'Ultrasonic Distance: {msg.data:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_subscriber = UltrasonicSubscriber()
    
    try:
        rclpy.spin(ultrasonic_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        ultrasonic_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
