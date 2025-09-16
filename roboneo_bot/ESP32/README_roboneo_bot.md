# Roboneo Bot - Combined Publisher and Subscriber

This Arduino sketch combines both a subscriber for `geometry_msgs/Twist` messages and a publisher for ultrasonic sensor data. It enables the robot to receive motion commands while simultaneously publishing distance measurements.

## Features

- Subscribes to `geometry_msgs/Twist` messages on the `/cmd_vel` topic for motion control
- Publishes ultrasonic distance measurements on the `/ultrasonic/distance` topic
- Implements differential drive kinematics for motor control
- Uses WiFi for UDP communication with micro-ROS agent
- Connection status indicated by built-in LED
- Compatible with Arduino IDE

## Hardware Setup

### Components Required
- ESP32 development board
- HC-SR04 ultrasonic sensor
- Motor driver (e.g., L298N, TB6612FNG)
- DC motors (2 for differential drive)
- Power supply for motors
- Jumper wires
- Breadboard (optional)

### Wiring Diagram
```
ESP32         HC-SR04        Motor Driver     Motors
----------------------------------------------------
GPIO 25   ---> Trig          IN1 (Left)  ---> Left Motor
GPIO 26   ---> Echo          IN2 (Left)
GPIO 16  --->               IN3 (Right) ---> Right Motor
GPIO 17  --->               IN4 (Right)
GPIO 2   ---> LED           5V          ---> 5V (Logic)
5V       ---> VCC           GND         ---> GND
GND      ---> GND
```

Note: The motor driver power connections are not shown. You'll need to connect:
- Motor driver power supply to VM or VCC
- Motor driver grounds connected together
- Motors to the motor driver output terminals

### Pin Configuration
You can modify the pin assignments in the code if needed:
```cpp
// HC-SR04 Ultrasonic Sensor pins
const int trigPin = 25;
const int echoPin = 26;

// Motor control pins
#define LEFT_MOTOR_PIN1 16
#define LEFT_MOTOR_PIN2 17
#define RIGHT_MOTOR_PIN1 18
#define RIGHT_MOTOR_PIN2 19
```

## Software Setup

### Prerequisites
1. Arduino IDE installed
2. ESP32 board support installed in Arduino IDE
3. micro-ROS Arduino library installed (Jazzy version)

### Installation Steps

1. **Install ESP32 Board Support**:
   - Open Arduino IDE
   - Go to `File → Preferences`
   - Add this URL to Additional Board Manager URLs:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to `Tools → Board → Boards Manager`
   - Search for "ESP32" and install

2. **Install micro-ROS Arduino Library**:
   - Download the micro-ROS Arduino library for Jazzy:
     [micro_ros_arduino Jazzy](https://github.com/micro-ROS/micro_ros_arduino/archive/refs/heads/jazzy.zip)
   - In Arduino IDE, go to `Sketch → Include Library → Add .ZIP Library...`
   - Select the downloaded ZIP file

### Configuration

Before uploading the code, modify these parameters in the sketch:

1. **WiFi Credentials**:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

2. **Host IP Address**:
   ```cpp
   const char* host_ip = "192.168.1.100";  // Change to your PC's IP
   ```

## Usage

1. **Start micro-ROS Agent**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/microros_ws/install/local_setup.bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

2. **Upload Sketch to ESP32**:
   - Open `reboneo_bot.ino` in Arduino IDE
   - Select your ESP32 board and port
   - Upload the sketch

3. **Monitor Serial Output**:
   - Open Serial Monitor (115200 baud rate)
   - You should see connection status, received Twist messages, and published distance values

4. **Publish Twist Messages**:
   You can publish Twist messages using various methods:

   **Using ROS 2 command line**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
   ```

5. **Monitor Ultrasonic Data**:
   ```bash
   ros2 topic echo /ultrasonic/distance
   ```

## ROS 2 Interface

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot motion

### Published Topics
- `/ultrasonic/distance` (std_msgs/Float32): Distance measurements in centimeters

### Node Name
- `roboneo_bot`

### Message Formats

**geometry_msgs/Twist**:
```yaml
geometry_msgs/Twist:
  geometry_msgs/Vector3 linear
    float64 x  # Linear velocity in x direction (forward/backward)
    float64 y  # Linear velocity in y direction (left/right)
    float64 z  # Linear velocity in z direction (up/down)
  geometry_msgs/Vector3 angular
    float64 x  # Angular velocity around x axis (roll)
    float64 y  # Angular velocity around y axis (pitch)
    float64 z  # Angular velocity around z axis (yaw)
```

**std_msgs/Float32**:
```yaml
std_msgs/Float32:
  float32 data  # Distance in centimeters
```

## Motor Control Implementation

The subscriber implements a simple differential drive kinematics model:

1. **Velocity Conversion**:
   - Left wheel speed = linear.x - angular.z
   - Right wheel speed = linear.x + angular.z

2. **Speed Normalization**:
   - Speeds are normalized to the -1.0 to 1.0 range
   - If either speed exceeds 1.0, both are scaled down proportionally

3. **Motor Control**:
   - PWM is used for speed control
   - Digital output pins control motor direction

## Ultrasonic Sensor Implementation

The publisher measures distance using the HC-SR04 ultrasonic sensor:

1. **Measurement Process**:
   - Send a 10μs trigger pulse
   - Measure the echo pulse duration
   - Calculate distance using the speed of sound

2. **Distance Calculation**:
   - Distance (cm) = Duration (μs) × 0.01715

3. **Publishing Rate**:
   - Distance is published every 500ms (2Hz)

## Troubleshooting

### No Twist Messages Received
- Check that the topic name matches between publisher and subscriber (`/cmd_vel`)
- Verify the micro-ROS agent is running on the specified IP and port
- Ensure ESP32 and PC are on the same network
- Confirm firewall allows UDP traffic on port 8888

### No Distance Data Published
- Check wiring connections between ESP32 and HC-SR04
- Verify power supply to the ultrasonic sensor
- Ensure trigPin and echoPin match your wiring

### Motors Not Responding
- Check wiring connections between ESP32 and motor driver
- Verify power supply to the motor driver and motors
- Ensure motor driver enable pins are properly configured
- Check that the motor control pins in the code match your wiring

### LED Indicators
- LED OFF: Not connected to micro-ROS agent
- LED ON: Connected to micro-ROS agent

## Customization

### Motor Control Pins
Modify the pin definitions to match your hardware:
```cpp
#define LEFT_MOTOR_PIN1 16
#define LEFT_MOTOR_PIN2 17
#define RIGHT_MOTOR_PIN1 18
#define RIGHT_MOTOR_PIN2 19
```

### Ultrasonic Sensor Pins
Modify the pin definitions for the HC-SR04:
```cpp
const int trigPin = 3;
const int echoPin = 1;
```

### Topic Names
Change the topic names in the subscriber and publisher initialization:
```cpp
// For subscriber
rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel") // Change this string

// For publisher
rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ultrasonic/distance") // Change this string
```

### Publishing Rate
Modify the timer timeout value to change publishing frequency:
```cpp
const unsigned int timer_timeout = 1000; // 1000ms = 4Hz
```

## Technical Details

### Connection Management
The code implements a state machine for robust connection handling:
1. WAITING_AGENT: Ping micro-ROS agent
2. AGENT_AVAILABLE: Create ROS entities
3. AGENT_CONNECTED: Process incoming messages and publish data
4. AGENT_DISCONNECTED: Clean up and reconnect

This ensures reliable communication even if the agent restarts or network connectivity is temporarily lost.

### Executor Configuration
The rclc executor is configured to handle both the subscriber and timer:
```cpp
rclc_executor_init(&executor, &support.context, 2, &allocator);
```

This allows the ESP32 to simultaneously process incoming Twist messages and publish distance data at regular intervals.

### Differential Drive Kinematics
The implementation assumes a differential drive robot where:
- Positive `linear.x` moves the robot forward
- Positive `angular.z` rotates the robot counter-clockwise
- The wheel speeds are calculated using the standard differential drive equations
