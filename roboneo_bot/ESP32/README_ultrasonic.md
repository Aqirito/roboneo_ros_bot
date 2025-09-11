# ESP32 HC-SR04 Ultrasonic Sensor Publisher

This Arduino sketch publishes distance measurements from an HC-SR04 ultrasonic sensor to a ROS 2 topic via UDP using micro-ROS.

## Features

- Reads distance measurements from HC-SR04 ultrasonic sensor
- Publishes distance data as `std_msgs/Float32` messages
- Uses WiFi for UDP communication with micro-ROS agent
- Connection status indicated by built-in LED
- Compatible with Arduino IDE

## Hardware Setup

### Components Required
- ESP32 development board
- HC-SR04 ultrasonic sensor
- Jumper wires
- Breadboard (optional)

### Wiring Diagram
```
ESP32         HC-SR04
---------------------
5V     ---->  VCC
GND    ---->  GND
GPIO 3 ---->  Trig
GPIO 1---->  Echo
```

Note: You can modify the pin assignments in the code if needed:
```cpp
const int trigPin = 3;
const int echoPin = 1;
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
   - Open `ultrasonic_publisher.ino` in Arduino IDE
   - Select your ESP32 board and port
   - Upload the sketch

3. **Monitor Serial Output**:
   - Open Serial Monitor (115200 baud rate)
   - You should see connection status and published distance values

4. **Verify ROS 2 Communication**:
   ```bash
   # List topics
   ros2 topic list
   
   # Echo the distance topic
   ros2 topic echo /ultrasonic/distance
   
   # View topic info
   ros2 topic info /ultrasonic/distance
   ```

## ROS 2 Interface

### Published Topics
- `/ultrasonic/distance` (std_msgs/Float32): Distance measurements in centimeters

### Node Name
- `esp32_ultrasonic_publisher`

### Message Format
```yaml
std_msgs/Float32:
  float32 data  # Distance in centimeters
```

## Troubleshooting

### No Distance Readings
- Check wiring connections between ESP32 and HC-SR04
- Verify power supply to the ultrasonic sensor (some need stable 5V)
- Ensure trigPin and echoPin match your wiring

### Connection Issues
- Verify WiFi credentials are correct
- Ensure ESP32 and PC are on the same network
- Check that micro-ROS agent is running on the specified IP and port
- Confirm firewall allows UDP traffic on port 8888

### LED Indicators
- LED OFF: Not connected to micro-ROS agent
- LED ON: Connected to micro-ROS agent

## Customization

### Publishing Rate
Modify the timer timeout value to change publishing frequency:
```cpp
const unsigned int timer_timeout = 500; // 500ms = 2Hz
```

### Topic Name
Change the topic name in the publisher initialization:
```cpp
rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ultrasonic/distance") // Change this string
```

## Technical Details

### Distance Calculation
The HC-SR04 measures distance using the speed of sound:
```
Distance = (Time × Speed of Sound) / 2
Distance (cm) = Duration (μs) × 0.01715
```

### Connection Management
The code implements a state machine for robust connection handling:
1. WAITING_AGENT: Ping micro-ROS agent
2. AGENT_AVAILABLE: Create ROS entities
3. AGENT_CONNECTED: Publish messages
4. AGENT_DISCONNECTED: Clean up and reconnect

This ensures reliable communication even if the agent restarts or network connectivity is temporarily lost.
