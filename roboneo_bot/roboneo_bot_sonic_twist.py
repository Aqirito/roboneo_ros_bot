#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from enum import IntEnum
from time import sleep

class RobotState(IntEnum):
    IDLE = 1
    OBSTACLE_DETECTED = 2
    TURNING_LEFT_TO_MEASURE = 3
    MEASURING_LEFT = 4
    TURNING_BACK_FROM_LEFT = 5
    TURNING_RIGHT_TO_MEASURE = 6
    MEASURING_RIGHT = 7
    TURNING_BACK_FROM_RIGHT = 8
    DECIDING_TURN = 9
    TURNING_FINAL = 10


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
        
        # Initial values
        self.distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.state = RobotState.IDLE
        
        # Timer for non-blocking delays
        self.timer = None

        # Log startup info
        self.get_logger().info('Roboneo Bot Tester started')
        self.get_logger().info('Publishing Twist commands to /cmd_vel')
        self.get_logger().info('Subscribing to /ultrasonic/distance for obstacle detection')

    def send_twist_command(self, linear_x, angular_z):
        """
        Publish a Twist message with given linear.x and angular.z.
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        self.twist_publisher.publish(msg)

    def distance_callback(self, msg):
        """
        Update current distance and trigger behavior based on robot state.
        """
        self.distance = msg.data
        self.get_logger().debug(f'Received distance: {self.distance:.2f} cm')

        # Only react if we're waiting to capture a measurement
        if self.state == RobotState.MEASURING_LEFT:
            self.left_distance = self.distance
            self.get_logger().info(f'✅ Left distance captured: {self.left_distance:.2f} cm')
            self.turn_back_from_left()

        elif self.state == RobotState.MEASURING_RIGHT:
            self.right_distance = self.distance
            self.get_logger().info(f'✅ Right distance captured: {self.right_distance:.2f} cm')
            self.decide_and_turn()

        elif self.state == RobotState.IDLE and self.distance < 20.0:
            self.get_logger().warn('🛑 Obstacle detected within 20 cm! Initiating avoidance...')
            self.send_twist_command(-0.8, 0.0)  # Reverse a bit
            self.state = RobotState.OBSTACLE_DETECTED
            sleep(0.2)  # Brief pause to ensure reverse
            self.measure_left_start()
        elif self.state == RobotState.IDLE and self.distance > 20.0:
            # self.on_final_turn_done()  # Continue moving forward
            self.send_twist_command(0.8, 0.0)  # Move forward

    def update_behavior(self):
        """
        Legacy placeholder – now handled directly in callback.
        You can expand logic here if needed.
        """
        pass

    def measure_left_start(self):
        """
        Start turning left to measure the left-side clearance.
        """
        self.get_logger().info('🔄 Turning left to measure side distance...')
        self.send_twist_command(0.0, 1.0)  # Turn left (positive angular z)
        self.state = RobotState.TURNING_LEFT_TO_MEASURE
        self.timer = self.create_timer(0.8, self.on_left_turn_finished)  # Wait ~90 deg turn

    def on_left_turn_finished(self):
        """
        Called after timer expires — stop turning and prepare to capture left distance.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info("🛑 Stopped. Now capturing left distance...")
        self.state = RobotState.MEASURING_LEFT  # Next distance read will be used as left
        sleep(0.2)  # Brief pause to ensure stable reading

    def turn_back_from_left(self):
        """
        Return to forward-facing orientation after measuring left.
        """
        self.get_logger().info('↩️ Turning back to center (from left)...')
        self.send_twist_command(0.0, -1.0)  # Turn right
        self.state = RobotState.TURNING_BACK_FROM_LEFT
        self.timer = self.create_timer(0.8, self.on_turned_back_from_left)

    def on_turned_back_from_left(self):
        """
        After returning to center, begin measuring right side.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        sleep(0.2)  # Brief pause to ensure stable reading
        self.get_logger().info("✅ Back to center. Preparing to measure right side.")
        self.measure_right_start()

    def measure_right_start(self):
        """
        Turn right to measure right-side clearance.
        """
        self.get_logger().info('🔄 Turning right to measure side distance...')
        self.send_twist_command(0.0, -1.0)  # Turn right
        self.state = RobotState.TURNING_RIGHT_TO_MEASURE
        self.timer = self.create_timer(0.8, self.on_right_turn_finished)

    def on_right_turn_finished(self):
        """
        After turning right, stop and prepare to capture right distance.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info("🛑 Stopped. Now capturing right distance...")
        self.state = RobotState.MEASURING_RIGHT  # Next distance read is right
        sleep(0.2)  # Brief pause to ensure stable reading

    def decide_and_turn(self):
        """
        Compare left and right distances and turn toward the clearer path.
        """
        self.get_logger().info(f"📊 Comparison: Left = {self.left_distance:.2f} cm, "
                               f"Right = {self.right_distance:.2f} cm")
        self.state = RobotState.DECIDING_TURN

        if self.left_distance > self.right_distance:
            self.get_logger().info('🟢 Turning left — more space available.')
            self.send_twist_command(0.0, 1.0)  # Turn left
            self.timer = self.create_timer(1.4, self.on_final_turn_done)
        else:
            self.get_logger().info('🟢 Turning right — equal or more space.')
            # self.send_twist_command(0.0, -1.0)  # Turn right
            self.timer = self.create_timer(0.1, self.on_final_turn_done)

        self.state = RobotState.TURNING_FINAL

    def on_final_turn_done(self):
        """
        Finalize maneuver: stop turn and resume forward motion.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info("🚀 Resuming forward movement.")
        self.send_twist_command(0.8, 0.0)  # Move forward
        self.reset_state()

    def reset_state(self):
        """
        Reset internal state to allow next obstacle response.
        """
        self.state = RobotState.IDLE
        self.left_distance = 0.0
        self.right_distance = 0.0

    def destroy_timer_or_cancel(self):
        """
        Safely cancel and destroy the active timer.
        """
        if self.timer is not None:
            self.timer.cancel()
            try:
                self.timer.destroy()
            except:
                pass
            self.timer = None


def main(args=None):
    rclpy.init(args=args)
    
    node = RoboneoBotTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down gracefully...")
    finally:
        node.send_twist_command(0.0, 0.0)  # Ensure robot stops
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()