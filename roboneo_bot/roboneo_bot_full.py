#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from enum import IntEnum
from time import sleep
import requests
from datetime import datetime
from .config import load_config
import threading


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
    PILLAR_DETECTED = 11
    PILLAR_SCAN_START = 12          # Start 360 scan for pillar
    PILLAR_SCANNING = 13            # Currently rotating to scan
    PILLAR_SCAN_COMPLETE = 14       # Finished scanning
    MOVING_TO_PILLAR = 15           # Moving toward detected pillar
    STOPPED = 16                     # Stopped after reaching pillar
    TURNING_360 = 17                 # Performing 360-degree rotation


class StageLevel(IntEnum):
    LEVEL_1 = 1 # Maze
    LEVEL_2 = 2 # Obstacle avoidance & Find the pillar

class RoboneoBotTester(Node):
    def __init__(self):
        super().__init__('roboneo_bot')

        self.config = load_config()

        # Create publisher for Twist messages
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for ultrasonic distance data
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/ultrasonic/distance',
            self.distance_callback,
            10)

        # Create subscriber for lidar distance data
        self.lidar_distance_subscriber = self.create_subscription(
            Float32,
            '/tfluna/distance',
            self.lidar_distance_callback,
            10)
        
        # Create subscriber for color rgb data
        self.color_name_subscriber = self.create_subscription(
            String,
            '/color_sensor/color_name',
            self.color_name_callback,
            10)
        
        # Initial values
        self.distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.state = RobotState.IDLE
        self.color_name = 'Unknown'

        self.lidar_distance = 0.0
        self.stage_level = StageLevel.LEVEL_1  # Change this to LEVEL_2 for normal mode

        # Timer for non-blocking delays
        self.timer = None

        # Pillar detection variables
        self.pillar_detected = False
        self.pillar_distance = 0.0
        self.scan_start_time = None
        self.last_scan_time = None

        # Log startup info
        self.get_logger().info('Roboneo Bot Tester started')
        self.get_logger().info('Publishing Twist commands to /cmd_vel')
        self.get_logger().info('Subscribing to /ultrasonic/distance for obstacle detection')
        self.get_logger().info('Subscribing to /tfluna/distance for pillar detection')
        self.get_logger().info('Subscribing to /color_sensor/color_name for color name detection')
        self.get_logger().info(f'Stage Level: {self.stage_level.name}')

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

    def lidar_distance_callback(self, msg):
        """
        Update current lidar distance.
        """
        self.lidar_distance = msg.data
        self.get_logger().debug(f'Received lidar distance: {self.lidar_distance:.2f} cm')

    def distance_callback(self, msg):
        """
        Update current distance and trigger behavior based on robot state.
        """
        self.distance = msg.data
        self.get_logger().debug(f'Received distance: {self.distance:.2f} cm')

        # Stage 2: Pillar detection mode
        if self.stage_level == StageLevel.LEVEL_2:
            self.handle_stage_2_logic()
        else:
            self.handle_normal_mode_logic()
    
    def color_name_callback(self, msg):
        """
        Update current color name.
        """
        match(msg.data):
            case 'Red':
                self.color_name = 'Red'
            case 'Green':
                self.color_name = 'Green'
            case 'Blue':
                self.color_name = 'Blue'
            case 'Yellow':
                self.color_name = 'Yellow'

        if self.color_name == self.config['floor_color_arena_border']:
            self.get_logger().info(f'{self.config['floor_color_arena_border']} detected - Performing 180-degree rotation')
            self.send_events(f"{self.config['floor_color_arena_border']} detected - Performing 180-degree rotation")
            self.perform_180_rotation()
            self.color_name = 'Unknown'  # Reset to avoid repeated triggers
        elif self.color_name == self.config['floor_color_zone_two_entrance']:
            self.stage_level = StageLevel.LEVEL_2
            self.get_logger().info(f'{self.config['floor_color_zone_two_entrance']} detected - Switching to Stage Level 2 (Pillar Detection Mode)')
            self.send_events(f"{self.config['floor_color_zone_two_entrance']} detected - Switching to Stage Level 2 (Pillar Detection Mode)")
            self.color_name = 'Unknown'  # Reset to avoid repeated triggers
        elif self.color_name == self.config['floor_color_zone_one_color_0']:
            self.get_logger().info(f'{self.config["floor_color_zone_one_color_0"]} detected - Zone 1 color for points')
            self.send_events(f"{self.config['floor_color_zone_one_color_0']} detected - Zone 1 color for points")
            self.color_name = 'Unknown'  # Reset to avoid repeated triggers
        elif self.color_name == self.config['floor_color_zone_one_color_1']:
            self.get_logger().info(f'{self.config["floor_color_zone_one_color_1"]} detected - Zone 1 color for points')
            self.send_events(f"{self.config['floor_color_zone_one_color_1']} detected - Zone 1 color for points")
            self.color_name = 'Unknown'  # Reset to avoid repeated triggers


    def handle_stage_2_logic(self):
        """
        Handle Stage 2 logic with pillar detection
        """
        # Handle pillar scanning
        self.check_pillar_scan_complete()
        self.check_for_pillar_during_scan()

        # Standard obstacle avoidance logic
        if self.state == RobotState.MEASURING_LEFT:
            self.left_distance = self.distance
            self.get_logger().info(f'✅ Left distance captured: {self.left_distance:.2f} cm')
            # self.send_events(f'Left distance captured: {self.left_distance:.2f} cm')
            self.turn_back_from_left()

        elif self.state == RobotState.MEASURING_RIGHT:
            self.right_distance = self.distance
            self.get_logger().info(f'✅ Right distance captured: {self.right_distance:.2f} cm')
            # self.send_events(f'Right distance captured: {self.right_distance:.2f} cm')
            self.decide_and_turn()

        elif self.state == RobotState.IDLE:
            current_time = self.get_clock().now()
            
            # Check if enough time has passed since last scan
            can_scan = True
            elapsed_time = 0.0
            if self.last_scan_time is not None:
                elapsed_time = (current_time - self.last_scan_time).nanoseconds / 1e9
                can_scan = elapsed_time >= self.config['scan_cooldown']
            
            # Start pillar scan if no obstacle and cooldown passed
            if self.distance > 30.0 and can_scan:  # No immediate obstacle
                self.last_scan_time = current_time
                self.get_logger().info(f'⏰ {elapsed_time:.1f}s since last scan - Starting new pillar scan')
                self.start_pillar_scan()
            # Obstacle avoidance if obstacle detected
            elif self.distance < self.config['detected_distance']:
                self.get_logger().warn(f'🛑 Obstacle detected within {self.config['detected_distance']} cm! Initiating avoidance...')
                # self.send_events(f'Obstacle detected within {self.config['detected_distance']} cm! Initiating avoidance...')
                self.send_twist_command(0.0, 0.0)  # Stop
                sleep(0.2)  # Brief pause to ensure reverse
                self.state = RobotState.OBSTACLE_DETECTED
                self.measure_left_start()
            # Continue forward if no obstacle and not scanning
            elif self.distance > self.config['detected_distance']:
                self.send_twist_command(self.config['forward_speed'], 0.0)  # Move forward

    def handle_normal_mode_logic(self):
        """
        Handle normal obstacle avoidance logic (Stages 1/2)
        """
        if self.state == RobotState.MEASURING_LEFT:
            self.left_distance = self.distance
            self.get_logger().info(f'✅ Left distance captured: {self.left_distance:.2f} cm')
            # self.send_events(f'Left distance captured: {self.left_distance:.2f} cm')
            self.turn_back_from_left()

        elif self.state == RobotState.MEASURING_RIGHT:
            self.right_distance = self.distance
            self.get_logger().info(f'✅ Right distance captured: {self.right_distance:.2f} cm')
            # self.send_events(f'Right distance captured: {self.right_distance:.2f} cm')
            self.decide_and_turn()

        elif self.state == RobotState.IDLE and self.distance < self.config['detected_distance']:
            self.get_logger().warn('🛑 Obstacle detected within 20 cm! Initiating avoidance...')
            self.send_twist_command(0.0, 0.0)  # Stop
            # self.send_events('Obstacle detected within 20 cm! Initiating avoidance...')
            sleep(0.2)  # Brief pause to ensure reverse
            self.state = RobotState.OBSTACLE_DETECTED
            self.measure_left_start()
        elif self.state == RobotState.IDLE and self.distance > self.config['detected_distance']:
            self.send_twist_command(self.config['forward_speed'], 0.0)  # Move forward

    def send_events(self, events: str):
        """
        Send an HTTP POST with json data in a non-blocking way.
        """
        def send_request():
            headers = {
                'Content-Type': 'application/json',
                'Accept': 'application/json',
                    'x-secret-key': self.config['secret_key']
                    }
            payload = {
                "robot_id": f"{self.config['robot_id']}",
                "tournament_round": f"{self.config['tournament_round']}",
                "events": f"{events}",
                "device_timestamp": f"{datetime.now().isoformat()}"
            }
        
            try:
                response = requests.post(self.config['events_url'], headers=headers, json=payload)
                if response.status_code == 200:
                    self.get_logger().info(f'HTTP POST to {self.config['events_url']} succeeded: {response.text}')
                else:
                    self.get_logger().error(f'HTTP POST to {self.config['events_url']} failed with status code {response.status_code}')
            except Exception as e:
                self.get_logger().error(f'HTTP POST to {self.config['events_url']} failed: {e}')
        
        # Create and start a thread to send the request
        thread = threading.Thread(target=send_request)
        thread.daemon = True  # Dies when main thread dies
        thread.start()

    def measure_left_start(self):
        """
        Start turning left to measure the left-side clearance.
        """
        self.get_logger().info('🔄 Turning left to measure side distance...')
        # self.send_events('Turning left to measure side distance')
        self.send_twist_command(0.0, self.config['turn_speed'])  # Turn left (positive angular z)
        self.state = RobotState.TURNING_LEFT_TO_MEASURE
        self.timer = self.create_timer(self.config['turn_90deg_duration'], self.on_left_turn_finished)  # Wait ~90 deg turn

    def on_left_turn_finished(self):
        """
        Called after timer expires — stop turning and prepare to capture left distance.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info("🛑 Stopped. Now capturing left distance...")
        # self.send_events("Stopped. Now capturing left distance")
        sleep(0.2)
        self.state = RobotState.MEASURING_LEFT  # Next distance read will be used as left

    def turn_back_from_left(self):
        """
        Return to forward-facing orientation after measuring left.
        """
        self.get_logger().info('↩️ Turning back to center (from left)...')
        # self.send_events('Turning back to center (from left)')
        self.send_twist_command(0.0, -self.config['turn_speed'])  # Turn right
        self.state = RobotState.TURNING_BACK_FROM_LEFT
        self.timer = self.create_timer(self.config['turn_90deg_duration'], self.on_turned_back_from_left)

    def on_turned_back_from_left(self):
        """
        After returning to center, begin measuring right side.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info("✅ Back to center. Preparing to measure right side.")
        self.measure_right_start()

    def measure_right_start(self):
        """
        Turn right to measure right-side clearance.
        """
        self.get_logger().info('🔄 Turning right to measure side distance...')
        # self.send_events('Turning right to measure side distance')
        self.send_twist_command(0.0, -self.config['turn_speed'])  # Turn right
        self.state = RobotState.TURNING_RIGHT_TO_MEASURE
        self.timer = self.create_timer(self.config['turn_90deg_duration'], self.on_right_turn_finished)

    def on_right_turn_finished(self):
        """
        After turning right, stop and prepare to capture right distance.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info("🛑 Stopped. Now capturing right distance...")
        # self.send_events("Stopped. Now capturing right distance")
        sleep(0.2)
        self.state = RobotState.MEASURING_RIGHT  # Next distance read is right

    def decide_and_turn(self):
        """
        Compare left and right distances and turn toward the clearer path.
        """
        self.get_logger().info(f"📊 Comparison: Left = {self.left_distance:.2f} cm, "
                               f"Right = {self.right_distance:.2f} cm")
        self.state = RobotState.DECIDING_TURN

        if self.left_distance > self.right_distance:
            self.get_logger().info('🟢 Turning left — more space available.')
            # self.send_events('Turning left — more space available.')
            self.send_twist_command(0.0, self.config['turn_speed'])  # Turn left
            self.timer = self.create_timer(self.config['turn_180deg_duration'], self.on_final_turn_done)
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
        self.send_twist_command(self.config['forward_speed'], 0.0)  # Move forward
        self.reset_state()

    def reset_state(self):
        """
        Reset internal state to allow next obstacle response or pillar scan.
        """
        self.state = RobotState.IDLE
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.pillar_detected = False
        self.pillar_distance = 0.0

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

    # ==================== PILLAR DETECTION METHODS ====================

    def start_pillar_scan(self):
        """
        Start 360° rotation to scan for pillar
        """
        if self.state != RobotState.IDLE:
            return
            
        self.get_logger().info('🔍 Starting 360° pillar scan...')
        # self.send_events('Starting 360° pillar scan...')
        self.state = RobotState.PILLAR_SCAN_START
        self.pillar_detected = False
        self.scan_start_time = self.get_clock().now()
        
        # Start rotating slowly
        self.send_twist_command(0.0, self.config['pillar_scan_turn_speed'])  # Slow rotation for better scanning
        self.state = RobotState.PILLAR_SCANNING

    def check_pillar_scan_complete(self):
        """
        Check if 360° scan is complete
        """
        if self.state == RobotState.PILLAR_SCANNING:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.scan_start_time).nanoseconds / 1e9
            
            if elapsed >= self.config['pillar_scan_duration']:
                self.stop_pillar_scan()
                
    def stop_pillar_scan(self):
        """
        Stop the pillar scanning rotation and resume forward movement
        """
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info('✅ Pillar scan complete')
        # self.send_events('Pillar scan complete')
        self.state = RobotState.PILLAR_SCAN_COMPLETE
        
        # If pillar found, move toward it
        if self.pillar_detected:
            self.get_logger().info(f'🎯 Pillar found at {self.pillar_distance:.2f} cm - moving toward it')
            self.send_events(f'Pillar found at {self.pillar_distance:.2f} cm - moving toward it')
            self.move_to_pillar()
        else:
            # No pillar found - continue forward movement
            self.get_logger().info('❌ No pillar found - continuing forward')
            # self.send_events('No pillar found - continuing forward')
            self.continue_forward_movement()

    def check_for_pillar_during_scan(self):
        """
        Check for pillar during scanning rotation
        """
        if (self.state == RobotState.PILLAR_SCANNING and 
            self.config['min_lidar_distance'] <= self.lidar_distance <= self.config['max_lidar_distance']):
            self.pillar_detected = True
            self.pillar_distance = self.lidar_distance
            self.get_logger().info(f'🟠 Pillar detected at {self.lidar_distance:.2f} cm during scan!')
            self.send_events(f'Pillar detected at {self.lidar_distance:.2f} cm during scan!')
            self.stop_pillar_scan()

    def move_to_pillar(self):
        """
        Move forward toward the detected pillar
        """
        self.get_logger().info(f'🚀 Moving toward pillar at {self.pillar_distance:.2f} cm')
        self.send_events(f'Moving toward pillar at {self.pillar_distance:.2f} cm')
        self.state = RobotState.MOVING_TO_PILLAR
        self.send_twist_command(self.config['forward_speed'], 0.0)  # Move forward
        
        # Stop after 3 seconds or when close to pillar
        self.create_timer(4.0, self.stop_moving_to_pillar)

    def stop_moving_to_pillar(self):
        """
        Stop moving toward pillar
        """
        self.send_twist_command(0.0, 0.0)
        self.get_logger().info('⏹️ Stopped moving toward pillar')
        self.send_events('Stopped moving toward pillar')
        self.destroy_timer_or_cancel()
        self.state = RobotState.STOPPED
        # self.reset_state()

    def perform_180_rotation(self):
        """
        Perform a 180-degree rotation when red color is detected.
        """
        if self.state == RobotState.TURNING_360:
            return  # Already performing rotation
            
        self.get_logger().info('🔄 Starting 180-degree rotation')
        # Cancel any existing timers to prevent conflicts
        self.destroy_timer_or_cancel()
        
        self.state = RobotState.TURNING_360
        
        # Send twist command to start rotating
        self.send_twist_command(0.0, self.config['turn_speed'])  # Rotate clockwise
        
        # Create timer to stop rotation after 180 degrees
        self.timer = self.create_timer(self.config['turn_180deg_duration'], self.on_180_rotation_complete)

    def on_180_rotation_complete(self):
        """
        Called after 180-degree rotation is complete.
        """
        self.destroy_timer_or_cancel()
        self.send_twist_command(0.0, 0.0)  # Stop rotation
        self.get_logger().info("✅ 180-degree rotation complete")
        # self.send_events("360-degree rotation complete")
        self.continue_forward_movement()  # Return to IDLE state

    def continue_forward_movement(self):
        """
        Resume forward movement after pillar scan
        """
        self.get_logger().info('🚀 Resuming forward movement')
        self.send_twist_command(self.config['forward_speed'], 0.0)  # Move forward
        self.reset_state()


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