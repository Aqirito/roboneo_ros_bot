#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class RoboneoBotTester(Node):
    def __init__(self):
        super().__init__('roboneo_bot')
        
        # Create publisher for Twist messages
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for ultrasonic distance data
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/ultrasonic/distance',
            self.distance_callback,
            10)
        
        self.distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.robot_state = 'IDLE'  # IDLE, OBSTACLE_DETECTED, MEASURING_LEFT, MEASURING_RIGHT, TURNING
        
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
        # self.get_logger().info(f'Published Twist: linear.x={linear_x}, angular.z={angular_z}')

    def distance_callback(self, msg):
        # self.get_logger().info(f'Received distance: {msg.data:.2f} cm')
        self.distance = msg.data
        if self.robot_state == 'IDLE':
            self.auto_movement()

    def auto_movement(self):
        # Example auto movement logic based on distance
        if self.distance < 20.0 and self.robot_state == 'IDLE':
            self.get_logger().info('Obstacle detected! Stopping.')
            self.send_twist_command(0.0, 0.0)
            self.robot_state = 'OBSTACLE_DETECTED'
            self.measure_left()
        else:
            self.send_twist_command(0.5, 0.0)  # Move forward

    def measure_left(self):
        if self.robot_state != 'OBSTACLE_DETECTED':
            return
        self.get_logger().info('Measuring left distance')
        self.send_twist_command(0.0, 1.0)  # Turn left
        time.sleep(1)  # Simulate time taken to measure
        self.left_distance = self.distance # Assume we get the distance from the callback
        print(f"Left distance measured: {self.left_distance}")
        self.send_twist_command(0.0, -1.0)  # Turn right
        time.sleep(1)  # Return to original position
        self.send_twist_command(0.0, 0.0)  # Stop
        time.sleep(2)
        self.robot_state = 'MEASURING_LEFT'
        self.measure_right()
    
    def measure_right(self):
        if self.robot_state != 'MEASURING_LEFT':
            return
        self.get_logger().info('Measuring right distance')
        self.send_twist_command(0.0, -1.0)  # Turn right
        time.sleep(1)  # Simulate time taken to measure
        self.right_distance = self.distance # Assume we get the distance from the callback
        print(f"Right distance measured: {self.right_distance}")
        self.send_twist_command(0.0, 1.0)  # Turn left
        time.sleep(1)  # Return to original position
        self.send_twist_command(0.0, 0.0)  # Stop
        time.sleep(2)
        self.robot_state = 'MEASURING_RIGHT'
        self.decide_turn()

    def decide_turn(self):
        if self.robot_state != 'MEASURING_RIGHT':
            return
        if self.left_distance > self.right_distance:
            self.get_logger().info('Turning left')
            self.send_twist_command(0.0, 1.0)  # Turn left
        else:
            self.get_logger().info('Turning right')
            self.send_twist_command(0.0, -1.0)  # Turn right
        time.sleep(1)  # Simulate turn duration
        self.send_twist_command(0.5, 0.0)  # Move forward
        time.sleep(2)  # Move forward duration
        self.send_twist_command(0.0, 0.0)  # Stop
        self.robot_state = 'IDLE'
        self.left_distance = 0.0
        self.right_distance = 0.0
        

def main(args=None):
    rclpy.init(args=args)
    tester = RoboneoBotTester()
    time.sleep(1)
    
    # Send a series of test commands
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
