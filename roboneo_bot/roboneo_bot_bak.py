#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

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
        
        self.get_logger().info('Roboneo Bot Tester started')
        self.get_logger().info('Publishing Twist messages to /cmd_vel')
        self.get_logger().info('Subscribing to distance data on /ultrasonic/distance')

        # Robot state variables
        self.right_distance = 0.0
        self.left_distance = 0.0
        self.distance = 0.0
        
        # State machine for robot behavior
        self.robot_state = 'IDLE'  # IDLE, OBSTACLE_DETECTED, MEASURING_LEFT, MEASURING_RIGHT, TURNING
        
        # Timer for handling delays without blocking
        self.delay_timer = None
        
        # Counter for state transitions
        self.state_counter = 0

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
        self.distance = msg.data
        
        # Only trigger auto movement if we're in IDLE state
        if self.robot_state == 'IDLE':
            self.auto_movement()

    def auto_movement(self):
        # Check if distance is less than 15 CM
        if self.distance < 15.0 and self.robot_state == 'IDLE':
            self.get_logger().info('Distance under 15 CM - Stopping motor')
            # Stop the motor
            self.send_twist_command(0.0, 0.0)
            self.robot_state = 'OBSTACLE_DETECTED'
            
            # Start timer for 5 seconds before measuring left distance
            self.delay_timer = self.create_timer(5.0, self.measure_left_distance)
        elif self.robot_state == 'IDLE':
            # Normal forward movement when no obstacle is detected
            self.get_logger().info('Moving forward')
            self.send_twist_command(0.5, 0.0)  # Move forward
            
            # Check if we have measured both sides and decide direction
            if self.left_distance > 0.0 and self.right_distance > 0.0:
                if self.left_distance > self.right_distance:
                    self.get_logger().info('Turning left')
                    self.send_twist_command(0.0, 1.0)  # Turn left
                    self.robot_state = 'TURNING'
                    # Timer to stop turning after 1 second
                    self.delay_timer = self.create_timer(1.0, self.stop_turning_left)
                elif self.right_distance > self.left_distance:
                    self.get_logger().info('Turning right')
                    self.send_twist_command(0.0, -1.0)  # Turn right
                    self.robot_state = 'TURNING'
                    # Timer to stop turning after 1 second
                    self.delay_timer = self.create_timer(1.0, self.stop_turning_right)
                # Reset measurements after using them
                self.left_distance = 0.0
                self.right_distance = 0.0

    def measure_left_distance(self):
        # Clean up the timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer.destroy()
            self.delay_timer = None
            
        self.get_logger().info('Measuring distance on the left')
        self.send_twist_command(0.0, 1.0)  # Turn left slowly
        self.robot_state = 'MEASURING_LEFT'
        
        # Timer to stop turning and measure distance
        self.delay_timer = self.create_timer(1.0, self.stop_and_measure_left)

    def stop_and_measure_left(self):
        # Clean up the timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer.destroy()
            self.delay_timer = None
            
        self.send_twist_command(0.0, 0.0)  # Stop
        self.left_distance = self.distance
        self.get_logger().info(f'Distance on the left: {self.left_distance:.2f} cm')
        self.robot_state = 'MEASURING_LEFT'
        
        # Timer before measuring right distance
        self.delay_timer = self.create_timer(1.0, self.measure_right_distance)

    def measure_right_distance(self):
        # Clean up the timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer.destroy()
            self.delay_timer = None
            
        self.get_logger().info('Measuring distance on the right')
        self.send_twist_command(0.0, -1.0)  # Turn right slowly
        self.robot_state = 'MEASURING_RIGHT'
        
        # Timer to stop turning and measure distance
        self.delay_timer = self.create_timer(2.0, self.stop_and_measure_right)

    def stop_and_measure_right(self):
        # Clean up the timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer.destroy()
            self.delay_timer = None
            
        self.send_twist_command(0.0, 0.0)  # Stop
        self.right_distance = self.distance
        self.get_logger().info(f'Distance on the right: {self.right_distance:.2f} cm')
        
        # Decide which direction to turn
        if self.left_distance > self.right_distance:
            self.get_logger().info('Turning left')
            self.send_twist_command(0.0, 1.0)  # Turn left
            self.robot_state = 'TURNING'
            # Timer to stop turning after 1 second
            self.delay_timer = self.create_timer(1.0, self.stop_turning_left)
        elif self.right_distance > self.left_distance:
            self.get_logger().info('Turning right')
            self.send_twist_command(0.0, -1.0)  # Turn right
            self.robot_state = 'TURNING'
            # Timer to stop turning after 1 second
            self.delay_timer = self.create_timer(1.0, self.stop_turning_right)
        else:
            # If distances are equal, just move forward
            self.get_logger().info('Distances equal, moving forward')
            self.send_twist_command(0.5, 0.0)  # Move forward
            self.robot_state = 'IDLE'
            
            # Reset measurements
            self.left_distance = 0.0
            self.right_distance = 0.0

    def stop_turning_left(self):
        # Clean up the timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer.destroy()
            self.delay_timer = None
            
        self.send_twist_command(0.0, 0.0)  # Stop turning
        self.robot_state = 'IDLE'
        self.get_logger().info('Finished turning left, resuming normal operation')

    def stop_turning_right(self):
        # Clean up the timer
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer.destroy()
            self.delay_timer = None
            
        self.send_twist_command(0.0, 0.0)  # Stop turning
        self.robot_state = 'IDLE'
        self.get_logger().info('Finished turning right, resuming normal operation')

def main(args=None):
    rclpy.init(args=args)
    tester = RoboneoBotTester()

    try:
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
