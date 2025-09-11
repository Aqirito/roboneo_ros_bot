# Ultrasonic Sensor Data Subscriber

This Python script subscribes to distance measurements from the ESP32 HC-SR04 ultrasonic sensor publisher and displays them in the terminal.

## Features

- Subscribes to ultrasonic distance data published by ESP32
- Displays distance measurements in centimeters with 2 decimal places
- Uses ROS 2 standard message types (`std_msgs/Float32`)
- Compatible with the micro-ROS ESP32 publisher

## Usage

1. **Build Package**

   - Navigate to your ROS 2 workspace:
   ```bash
   cd ~/microros_ws/src/roboneo_bot
   ```

   - Open setup.py and add the entry point for the subscriber node below the publisher’s entry point. The entry_points field should now look like this:
      ```python
      entry_points={
         'console_scripts': [
               'ultrasonic_sub = roboneo_bot.ultrasonic_sub:main',
         ],
      },
      ```
   - Install dependencies:
      ```bash
      cd ~/microros_ws
      rosdep install -i --from-path src --rosdistro jazzy -y
      ```

   - Then build your workspace:
      ```bash
      cd ~/microros_ws
      colcon build --packages-select roboneo_bot
      source install/setup.bash
      ```

2. **Start the micro-ROS agent**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/microros_ws/install/local_setup.bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

3. **Upload and run the ESP32 publisher**:
   - Ensure the `ultrasonic_publisher.ino` is uploaded to your ESP32
   - Power on the ESP32 and verify it connects to WiFi and the micro-ROS agent
   
4. **Run the Python subscriber**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/microros_ws
   source install/setup.bash
   ros2 run roboneo_bot ultrasonic_sub
   ```

5. **View the data**:
   You should see distance measurements displayed in the terminal:
   ```
   [INFO] [ultrasonic_subscriber]: Ultrasonic Distance: 15.34 cm
   [INFO] [ultrasonic_subscriber]: Ultrasonic Distance: 14.87 cm
   [INFO] [ultrasonic_subscriber]: Ultrasonic Distance: 16.21 cm

6. **You can also view the data using (optional)**:
   ```bash
   ros2 topic echo /ultrasonic/distance
   ```

## ROS 2 Interface

### Subscribed Topics
- `/ultrasonic/distance` (std_msgs/Float32): Distance measurements in centimeters

### Node Name
- `ultrasonic_subscriber`

### Message Format
```yaml
std_msgs/Float32:
  float32 data  # Distance in centimeters
```

## Troubleshooting

### No Data Received
- Verify the micro-ROS agent is running on port 8888
- Check that the ESP32 is properly connected and publishing data
- Confirm the topic name matches between publisher and subscriber
- Ensure all nodes are using the same ROS 2 domain ID

### Check Topic Communication
```bash
# List all topics
ros2 topic list

# View topic information
ros2 topic info /ultrasonic/distance

# Echo the topic to verify data is being published
ros2 topic echo /ultrasonic/distance
```

## Customization

### Change Output Format
Modify the callback function to change how data is displayed:
```python
def ultrasonic_callback(self, msg):
    # Example: Show distance in meters
    self.get_logger().info(f'Ultrasonic Distance: {msg.data/100:.3f} m')
```

### Add Data Processing
You can add filtering or processing of the distance data:
```python
def ultrasonic_callback(self, msg):
    distance = msg.data
    # Example: Simple moving average
    if not hasattr(self, 'distance_history'):
        self.distance_history = []
    self.distance_history.append(distance)
    if len(self.distance_history) > 5:
        self.distance_history.pop(0)
    avg_distance = sum(self.distance_history) / len(self.distance_history)
    self.get_logger().info(f'Ultrasonic Distance: {distance:.2f} cm (Avg: {avg_distance:.2f} cm)')
```

## Integration with Other ROS 2 Nodes

This subscriber can be used alongside other ROS 2 nodes. For example, you could:

1. Create a launch file that starts the micro-ROS agent, ESP32 publisher, and Python subscriber together
2. Add additional processing nodes that use the distance data
3. Integrate with visualization tools like RViz2

## Technical Details

### Node Implementation
The subscriber follows the standard ROS 2 node pattern:
1. Inherits from `rclpy.node.Node`
2. Creates a subscription to the `/ultrasonic/distance` topic
3. Implements a callback function to process incoming messages
4. Uses proper node lifecycle management

### Message Handling
- The subscriber uses a queue size of 10 for message buffering
- Messages are processed as they arrive (asynchronous callback)
- Float32 data is formatted to 2 decimal places for display
