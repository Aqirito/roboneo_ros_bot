# Roboneo Bot Test Script

This Python script tests both the subscriber and publisher functionality of the roboneo_bot by sending Twist commands and receiving distance data.

## Features

- Publishes `geometry_msgs/Twist` messages to the `/cmd_vel` topic to control the robot
- Subscribes to `std_msgs/Float32` messages on the `/ultrasonic/distance` topic to receive distance data
- Sends a sequence of test commands to verify robot movement
- Displays received distance measurements in real-time

## Prerequisites

1. ROS 2 Jazzy installed
2. Python 3 installed
3. Required Python packages:
   ```bash
   pip install rclpy geometry_msgs std_msgs
   ```

## Usage

1. **Start the micro-ROS agent**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/microros_ws/install/local_setup.bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

2. **Upload and run the roboneo_bot sketch**:
   - Ensure the `reboneo_bot.ino` is uploaded to your ESP32
   - Power on the ESP32 and verify it connects to WiFi and the micro-ROS agent

3. **Run the test script**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/microros_ws
   source install/setup.bash
   ros2 run roboneo_bot test_roboneo_bot
   ```

4. **Observe the results**:
   You should see:
   - Published Twist commands in the terminal
   - Received distance measurements in the terminal
   - Robot movement corresponding to the Twist commands

## Test Sequence

The test script sends the following sequence of commands:
1. Move forward (linear.x = 0.7, angular.z = 0.0) for 2 seconds
2. Turn left (linear.x = 0.0, angular.z = 0.7) for 2 seconds
3. Move backward (linear.x = -0.7, angular.z = 0.0) for 2 seconds
4. Turn right (linear.x = 0.0, angular.z = -0.7) for 2 seconds
5. Stop (linear.x = 0.0, angular.z = 0.0) for 1 second

After sending these commands, the script continues to run and display distance measurements.

## Expected Behavior

### Robot Movement
- The robot should move forward for 2 seconds
- The robot should rotate counter-clockwise for 2 seconds
- The robot should move backward for 2 seconds
- The robot should rotate clockwise for 2 seconds
- The robot should stop

### Distance Data
- Distance measurements displayed in the terminal for 10 times
- Values should change as objects move closer to or farther from the ultrasonic sensor

## Troubleshooting

### No Robot Movement
- Verify the ESP32 is properly connected and receiving Twist messages
- Check wiring connections between ESP32 and motor driver
- Ensure the motor driver is properly powered
- Confirm that the motor control pins in the ESP32 code match your wiring

### No Distance Data
- Verify the ESP32 is properly connected and publishing distance data
- Check wiring connections between ESP32 and HC-SR04 sensor
- Ensure the ultrasonic sensor is properly powered
- Confirm that the trigPin and echoPin in the ESP32 code match your wiring

### Communication Issues
- Ensure all devices are on the same network
- Verify the micro-ROS agent is running on the correct IP and port
- Check that firewall settings allow UDP traffic on port 8888
- Confirm that topic names match between publisher/subscriber and the ESP32 code

## Customization

### Modify Test Sequence
You can modify the test sequence by changing the commands in the main function:
```python
# Example: Add a new command
tester.send_twist_command(0.3, 0.3)  # Move forward while turning left
time.sleep(3)
```

### Change Command Duration
Modify the time.sleep() values to change how long each command runs:
```python
tester.send_twist_command(0.5, 0.0)
time.sleep(5)  # Change from 2 seconds to 5 seconds
```

### Add Additional Commands
You can add more commands to test different robot behaviors:
```python
# Example: Diagonal movement
tester.send_twist_command(0.5, 0.3)
time.sleep(2)
```

## Technical Details

### Node Implementation
The test script creates a ROS 2 node that:
1. Publishes to the `/cmd_vel` topic with `geometry_msgs/Twist` messages
2. Subscribes to the `/ultrasonic/distance` topic with `std_msgs/Float32` messages
3. Sends a predefined sequence of movement commands
4. Displays received distance data in real-time

### Message Handling
- Twist messages use standard ROS 2 geometry_msgs format
- Distance messages use standard ROS 2 std_msgs format
- All messages are published with a queue size of 10
- The subscriber processes messages as they arrive (asynchronous callback)

### Timing
- Each movement command runs for a specified duration
- The script uses time.sleep() to control command duration
- Distance data is received and displayed in real-time
