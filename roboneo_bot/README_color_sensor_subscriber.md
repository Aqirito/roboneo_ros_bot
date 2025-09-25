# TCS3200 Color Sensor Subscriber

This Python script subscribes to color data from the ESP32 TCS3200 color sensor publisher and processes the detected colors and RGB values for robot control and data logging.

## Features

- Subscribes to color name and RGB data from ESP32 color sensor
- Processes color information for robot behavior control
- Compatible with micro-ROS ESP32 publisher
- Uses ROS 2 standard message types (`std_msgs/String`, `std_msgs/ColorRGBA`)
- Real-time color-based decision making

## Prerequisites

- ESP32 with TCS3200 color sensor running the publisher code (see `ESP32/README_color_sensor.md`)
- ROS 2 Jazzy installation
- micro-ROS agent running
- Python 3 with rclpy

## Usage

### 1. Build Package

Navigate to your ROS 2 workspace:

```bash
cd ~/microros_ws/src/roboneo_bot
```

Open setup.py and add the entry point for the color subscriber node:

```python
entry_points={
    'console_scripts': [
        'color_sub = roboneo_bot.color_subscriber:main',
    ],
},
```

Install dependencies:

```bash
cd ~/microros_ws
rosdep install -i --from-path src --rosdistro jazzy -y
```

Build your workspace:

```bash
cd ~/microros_ws
colcon build --packages-select roboneo_bot
source install/setup.bash
```

### 2. Start the micro-ROS agent

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 3. Upload and run the ESP32 publisher

- Ensure the `esp32_tcs3200_color_sense_pub.ino` is uploaded to your ESP32
- Power on the ESP32 and verify it connects to WiFi and the micro-ROS agent

### 4. Run the Python subscriber

```bash
source /opt/ros/jazzy/setup.bash
cd ~/microros_ws
source install/setup.bash
ros2 run roboneo_bot color_sub
```

### 5. View the data

You should see color detection messages in the terminal:

```
[INFO] [color_subscriber]: Detected color: Red
[INFO] [color_subscriber]: RGB values: (255, 0, 0)
[INFO] [color_subscriber]: Detected color: Blue  
[INFO] [color_subscriber]: RGB values: (0, 0, 255)
```

### 6. Monitor topics directly (optional)

```bash
# View color names
ros2 topic echo /color_sensor/color_name

# View RGB values
ros2 topic echo /color_sensor/rgb

# List all topics
ros2 topic list

# Check topic info
ros2 topic info /color_sensor/color_name
ros2 topic info /color_sensor/rgb
```

## ROS 2 Interface

### Subscribed Topics

#### `/color_sensor/color_name` (std_msgs/String)

- **Description**: Detected color name from ESP32 sensor
- **Rate**: ~5 Hz (every 200ms)
- **Example values**: "Red", "Green", "Blue", "White", "Black", "Yellow", "Cyan", "Magenta", "Orange", "Unknown"

#### `/color_sensor/rgb` (std_msgs/ColorRGBA)

- **Description**: Normalized RGB values from ESP32 sensor
- **Rate**: ~5 Hz (every 200ms)
- **Fields**:
  - `r`: Red component (0.0-1.0)
  - `g`: Green component (0.0-1.0)  
  - `b`: Blue component (0.0-1.0)
  - `a`: Alpha/opacity (always 1.0)

### Node Name

- `color_subscriber`

## Example Subscriber Implementations

### Basic Color Monitor

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA

class ColorMonitor(Node):
    def __init__(self):
        super().__init__('color_monitor')
        
        self.color_name_sub = self.create_subscription(
            String, '/color_sensor/color_name', 
            self.color_callback, 10)
        
        self.rgb_sub = self.create_subscription(
            ColorRGBA, '/color_sensor/rgb',
            self.rgb_callback, 10)
    
    def color_callback(self, msg):
        self.get_logger().info(f'Color detected: {msg.data}')
    
    def rgb_callback(self, msg):
        r = int(msg.r * 255)
        g = int(msg.g * 255) 
        b = int(msg.b * 255)
        self.get_logger().info(f'RGB: ({r}, {g}, {b})')
```

### Color-Based Robot Control

```python
from geometry_msgs.msg import Twist

class ColorController(Node):
    def __init__(self):
        super().__init__('color_controller')
        
        self.color_sub = self.create_subscription(
            String, '/color_sensor/color_name',
            self.color_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def color_callback(self, msg):
        twist = Twist()
        
        if msg.data == "Red":
            # Stop on red
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif msg.data == "Green":
            # Go forward on green
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        elif msg.data == "Blue":
            # Turn left on blue
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            
        self.cmd_pub.publish(twist)
```

## Troubleshooting

### No Color Data Received

- Verify the micro-ROS agent is running on port 8888
- Check that the ESP32 color sensor publisher is running and connected
- Confirm the topic names match between publisher and subscriber
- Ensure all nodes are using the same ROS 2 domain ID

### Check Topic Communication

```bash
# List all topics
ros2 topic list

# View topic information
ros2 topic info /color_sensor/color_name
ros2 topic info /color_sensor/rgb

# Echo topics to verify data is being published
ros2 topic echo /color_sensor/color_name
ros2 topic echo /color_sensor/rgb

# Check node information
ros2 node list
ros2 node info /color_subscriber
```

### Debugging Connection Issues

```bash
# Check if micro-ROS agent is running
ps aux | grep micro_ros_agent

# Monitor ROS 2 network traffic
ros2 topic hz /color_sensor/color_name
ros2 topic bw /color_sensor/rgb

# Test with simple echo
ros2 topic echo /color_sensor/color_name --once
```

## Integration Examples

### Data Logging

Create a CSV logger for color detection data:

```python
import csv
from datetime import datetime

class ColorLogger(Node):
    def __init__(self):
        super().__init__('color_logger')
        self.csv_file = open('color_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['timestamp', 'color', 'r', 'g', 'b'])
        
        # Subscribe to both topics
        self.color_sub = self.create_subscription(
            String, '/color_sensor/color_name', self.log_data, 10)
```

### Multi-Sensor Fusion

Combine color data with other sensors:

```python
class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor')
        
        # Subscribe to multiple sensors
        self.color_sub = self.create_subscription(
            String, '/color_sensor/color_name', self.color_cb, 10)
        self.distance_sub = self.create_subscription(
            Float32, '/ultrasonic/distance', self.distance_cb, 10)
        
        self.current_color = "Unknown"
        self.current_distance = 0.0
    
    def make_decision(self):
        # Combine color and distance for intelligent behavior
        if self.current_color == "Red" and self.current_distance < 20:
            # Emergency stop
            pass
        elif self.current_color == "Green":
            # Safe to proceed
            pass
```

## Advanced Features

### Color History Tracking

```python
from collections import deque

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.color_history = deque(maxlen=10)  # Last 10 colors
        
    def analyze_pattern(self):
        # Detect color patterns or sequences
        if len(self.color_history) >= 3:
            recent = list(self.color_history)[-3:]
            if recent == ["Red", "Green", "Blue"]:
                self.get_logger().info("RGB sequence detected!")
```

### Parameter-Based Configuration

```python
class ConfigurableColorNode(Node):
    def __init__(self):
        super().__init__('configurable_color')
        
        # Declare parameters
        self.declare_parameter('target_color', 'Blue')
        self.declare_parameter('response_speed', 0.5)
        
        # Get parameter values
        self.target = self.get_parameter('target_color').value
        self.speed = self.get_parameter('response_speed').value
```

Use parameters:

```bash
ros2 run roboneo_bot configurable_color --ros-args -p target_color:="Red" -p response_speed:=0.8
```
