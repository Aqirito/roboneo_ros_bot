# ROS 2 Jazzy with ESP32 via UDP Network Communication

## Prerequisites
- **Operating System**: Ubuntu 24.04 LTS
- **ROS 2 Distribution**: Jazzy Jalisco (LTS, released May 2024)
- **Microcontroller**: ESP32 (with WiFi capability)
- **Development Environment**: Arduino IDE (latest version)
- **Additional Hardware**:
  - HC-SR04 Ultrasonic Sensor
  - DC Motors with motor driver (L298N or similar)
  - Wheels and chassis for robot platform
- **Network**: Both ESP32 and Ubuntu machine should be on the same local network (WiFi or Ethernet)
- **Python 3**: Ensure Python 3 is installed on your Ubuntu system
- **Basic Knowledge**: Familiarity with ROS 2 concepts, Arduino programming, and terminal commands

## Compatibility Overview

**ROS 2 Jazzy Jalisco** is fully compatible with ESP32 microcontrollers through **micro-ROS**. Jazzy is the latest Long Term Support (LTS) release launched in May 2024, with support extending until May 2029. The micro-ROS project has been actively updated to support Jazzy, with all core repositories and standalone build systems receiving Jazzy branch updates.[^1_1][^1_2][^1_3]

**Key Compatibility Points:**

- **Supported Platforms**: Ubuntu 24.04 LTS (your setup) is a Tier 1 platform for ROS 2 Jazzy[^1_2][^1_1]
- **ESP32 Support**: ESP32 has been officially supported by micro-ROS since 2020, with native WiFi and UDP transport capabilities[^1_4][^1_5]
- **Network Communication**: UDP port 8888 is the default transport method for micro-ROS agents[^1_6][^1_7]


## Step-by-Step Setup Guide

### 1. Install micro-ROS Agent on Ubuntu 24.04

First, set up the micro-ROS agent on your host system:

```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Create workspace for micro-ROS
mkdir ~/microros_ws
cd ~/microros_ws

# Clone micro-ROS setup tools
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Clone roboneo_bot package
git clone https://github.com/Aqirito/roboneo_ros_bot.git src/roboneo_bot

# Optional: Clone teleop_twist_keyboard for manual control
git clone -b dashing https://github.com/ros2/teleop_twist_keyboard.git src/teleop_twist_keyboard

# Install dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools
colcon build
source install/local_setup.bash

# Create and build micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

Add this to your `.bashrc` for convenience:

```bash
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```


### 2. Install micro-ROS Arduino Library

1. **Download and install the micro-ROS Arduino library**:
Download as ZIP: [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/archive/refs/heads/jazzy.zip)
    - Go to `Sketch → Include Library → Add .ZIP Library...` in Arduino IDE
    - Select the downloaded ZIP file
    - Check `Sketch → Include Library` find the installed libraries probably named `micro_ros_arduino` to confirm installation

### 3. Configure ESP32 in Arduino IDE

1. **Install ESP32 Board Support**:
    - Open Arduino IDE
    - Go to `File → Preferences`
    - Add this URL to Additional Board Manager URLs:
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    - Go to `Tools → Board → Boards Manager`
    - Search for "ESP32" and install version 2.0.2 for safe compatibility with micro-ROS
2. **Select Board and Port**:
    - `Tools → Board → ESP32 Arduino → ESP32 Dev Module`
    - `Tools → Port → [Your ESP32 Port]` (usually `/dev/ttyUSB0` or `/dev/ttyACM0`)

### 4. Hardware Setup

#### HC-SR04 Ultrasonic Sensor Connections:
```
ESP32         HC-SR04
---------------------
5V     ---->  VCC
GND    ---->  GND
GPIO 25 ---->  Trig
GPIO 26 ---->  Echo
```

#### Motor Driver (L298N) Connections:
```
ESP32         Motor Driver     Motors
-------------------------------------
GPIO 16  ---> IN1 (Left)      Left Motor
GPIO 17  ---> IN2 (Left)      
GPIO 18  ---> IN3 (Right)     Right Motor
GPIO 19  ---> IN4 (Right)

5V       ---> VCC (Logic)
GND      ---> GND
```

### 5. ESP32 Arduino Sketches

The project includes multiple Arduino sketches for different functionalities:

#### A. Basic String Publisher
ESP32 code that publishes a "hello" string message via WiFi UDP: [esp32_hello_publisher.ino](roboneo_bot/ESP32/esp32_hello_publisher.ino)

#### B. Ultrasonic Sensor Publisher
ESP32 code that publishes distance measurements from HC-SR04 sensor: [ultrasonic_publisher.ino](roboneo_bot/ESP32/ultrasonic_publisher.ino)

#### C. Motor Control Subscriber
ESP32 code that subscribes to Twist messages for motor control: [twist_subscriber.ino](roboneo_bot/ESP32/twist_subscriber.ino)

#### D. Complete Robot Node
Combined functionality with both ultrasonic sensor publishing and motor control: [roboneo_bot.ino](roboneo_bot/ESP32/roboneo_bot.ino)

**Key Configuration Points for all sketches**:

- Update `YOUR_WIFI_SSID` and `YOUR_WIFI_PASSWORD` with your network credentials
- Change `host_ip` to your Ubuntu PC's IP address (find with `ip addr show`)
- Built-in LED (pin 2) indicates micro-ROS agent connection status
- All sketches use UDP port 8888 for communication

### 6. ROS 2 Python Scripts

Create ROS 2 nodes for communication with ESP32:

#### A. Basic String Subscriber
Receives messages from ESP32 hello publisher: [hello_sub.py](roboneo_bot/hello_sub.py)

#### B. LED Control Publisher
Publishes messages to control ESP32 LED: [led_state_pub.py](roboneo_bot/led_state_pub.py)

#### C. Ultrasonic Distance Subscriber
Receives distance measurements from ESP32: [ultrasonic_sub.py](roboneo_bot/ultrasonic_sub.py)

#### D. Robot Test Script
Comprehensive test script for the complete robot system: [test_roboneo_bot.py](roboneo_bot/test_roboneo_bot.py)

### 7. Build and Source Your ROS 2 Workspace

You likely already have the rclpy, std_msgs, and geometry_msgs packages installed as part of your ROS 2 system. It's good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

Then build your workspace:

```bash
cd ~/microros_ws
colcon build --packages-select roboneo_bot
source install/setup.bash
```

### 8. Running the Complete System

#### For Basic Hello Publisher/Subscriber:
**Terminal 1 - Start micro-ROS Agent:**

```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

You should see:

```
[INFO] [UDPv4AgentLinux.cpp] init | running... | port: 8888
```

**Terminal 2 - Run ROS 2 Subscriber:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run roboneo_bot hello_sub
```

**Terminal 3 - Monitor Topics (Optional):**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /esp32/hello
```

#### For Complete Roboneo Bot:
**Terminal 1 - Start micro-ROS Agent:**

```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 - Run Ultrasonic Distance Subscriber:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run roboneo_bot ultrasonic_sub
```

**Terminal 3 - Run Robot Test Script:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run roboneo_bot test_roboneo_bot
```

### 9. Flash and Test ESP32

1. **Upload Code**: Flash the desired Arduino sketch to your ESP32
2. **Monitor Connection**: Watch the serial monitor - you should see WiFi connection and micro-ROS agent discovery
3. **Verify Communication**: The ESP32's LED should turn on when connected to the agent
4. **Check Messages**: Your Python subscribers should receive messages from the ESP32

### 10. Advanced Usage

#### Running Individual Components:
To run individual components of the roboneo_bot system:

```bash
# Publish distance measurements
ros2 topic pub /ultrasonic_distance std_msgs/msg/Float32 "data: 0.0" --once

# Control motors with Twist messages
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" --once
```

#### Testing the Complete Robot:
Use the test script to verify all functionality:

```bash
ros2 run roboneo_bot test_roboneo_bot
```

This will:
1. Move the robot forward
2. Rotate the robot
3. Stop the robot
4. Display distance measurements

#### Controlling the Robot with teleop_twist_keyboard:
You can manually control the roboneo_bot using the teleop_twist_keyboard package:

**Terminal 1 - Start micro-ROS Agent:**
```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 - Run teleop_twist_keyboard:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the on-screen instructions to control the robot:
- `i`/`k` - Move forward/backward
- `j`/`l` - Turn left/right
- `q`/`z` - Increase/decrease linear speed
- `w`/`x` - Increase/decrease angular speed
- `space` - Stop the robot

**Terminal 3 - Monitor distance sensor (optional):**
```bash
ros2 topic echo /ultrasonic_distance
```

## Troubleshooting Tips

**Common Issues:**

- **No topics visible**: Ensure both devices are on the same network and firewall allows UDP traffic on port 8888[^1_8][^1_9]
- **Connection fails**: Verify IP address is correct and micro-ROS agent is running[^1_8]
- **WiFi connection issues**: Double-check SSID and password in the ESP32 code[^1_10]
- **Sensor readings inaccurate**: Check HC-SR04 wiring and power supply
- **Motor not responding**: Verify motor driver connections and power supply

**Network Verification:**

```bash
# Check if ESP32 can reach your PC
ping [ESP32_IP]

# Verify micro-ROS agent is listening
netstat -ulnp | grep 8888
```

**Debugging Connection Issues:**
1. Check that the micro-ROS agent is running on port 8888
2. Verify that both ESP32 and PC are on the same network
3. Confirm that the IP address in the ESP32 code matches your PC's IP
4. Use the serial monitor to check ESP32 connection status

This setup provides a robust foundation for ESP32-ROS 2 Jazzy communication over your local network via UDP, enabling real-time bidirectional communication between your microcontroller and ROS 2 system for robotics applications.[^1_11][^1_12]
<span style="display:none">[^1_13][^1_14][^1_15][^1_16][^1_17][^1_18][^1_19][^1_20][^1_21][^1_22][^1_23][^1_24][^1_25][^1_26][^1_27][^1_28][^1_29][^1_30][^1_31][^1_32][^1_33][^1_34][^1_35][^1_36][^1_37][^1_38][^1_39][^1_40][^1_41][^1_42][^1_43][^1_44][^1_45][^1_46][^1_47][^1_48][^1_49][^1_50][^1_51][^1_52][^1_53][^1_54][^1_55][^1_56][^1_57][^1_58][^1_59][^1_60][^1_61][^1_62]</span>

<div style="text-align: center">⁂</div>

[^1_1]: https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released

[^1_2]: https://discourse.openrobotics.org/t/ros-2-jazzy-jalisco-released/37862

[^1_3]: https://github.com/micro-ROS/micro_ros_setup/discussions/694

[^1_4]: https://discourse.ros.org/t/micro-ros-porting-to-esp32/16101

[^1_5]: https://micro.ros.org/blog/2020/08/27/esp32/

[^1_6]: https://snapcraft.io/install/micro-ros-agent/arch

[^1_7]: https://snapcraft.io/install/micro-ros-agent/ubuntu

[^1_8]: https://github.com/micro-ROS/micro_ros_setup/issues/532

[^1_9]: https://github.com/micro-ROS/micro_ros_platformio/issues/139

[^1_10]: https://technologiehub.at/project-posts/micro-ros-on-esp32-tutorial/

[^1_11]: https://github.com/97hackbrian/uRos2_esp32_uPython

[^1_12]: https://husarnet.com/blog/esp32-microros

[^1_13]: https://www.youtube.com/watch?v=0R8VUPEkYhg

[^1_14]: https://www.reddit.com/r/ROS/comments/15jey91/dynamic_communication_between_esp32_and_ros2/

[^1_15]: https://crossbell.io/notes/57370-2

[^1_16]: https://dev.to/lonebots/seamless-robotics-integration-setting-up-micro-ros-on-esp32-2ad2

[^1_17]: https://xiangyu.xlog.app/Integrating-ESP32-with-ROS-over-WiFi?locale=en

[^1_18]: https://www.youtube.com/watch?v=48SxC6LBVs8

[^1_19]: https://www.youtube.com/watch?v=WncOr9l4KQ8

[^1_20]: https://www.youtube.com/watch?v=qtVFsgTG3AA

[^1_21]: https://www.youtube.com/watch?v=HcbNKhWhL8o

[^1_22]: https://ardupilot.org/dev/docs/ros2-over-ethernet.html

[^1_23]: https://www.youtube.com/watch?v=dP9fVZKwBTk

[^1_24]: https://www.reddit.com/r/ROS/comments/175k1hl/ros2_esp32_sos/

[^1_25]: https://github.com/albertrichard080/ros2_esp32-microros

[^1_26]: https://docs.ros.org/en/jazzy/p/network_bridge/

[^1_27]: https://xiangyu-fu.github.io/2023/08/07/misc/esp32_ros_wifi/

[^1_28]: https://github.com/yossefhady/Self-Balance

[^1_29]: https://ros2-tutorial.readthedocs.io/en/latest/publishers_and_subscribers.html

[^1_30]: https://blog.hadabot.com/set-up-esp32-microcontroller-for-ros2-robotics.html

[^1_31]: https://www.theconstruct.ai/how-to-write-a-ros-publisher-and-subscriber-with-a-custom-message/

[^1_32]: https://deepwiki.com/micro-ROS/micro-ROS-Agent/3.2-running-the-agent\&rut=8410700258b7e59a9cf9de81d68c668823c76557b9b72b971bac5b6970e81766

[^1_33]: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

[^1_34]: https://docs.ros.org/en/jazzy/index.html

[^1_35]: https://www.youtube.com/watch?v=8407qTyBRe0

[^1_36]: https://www.youtube.com/watch?v=tL7swYonY8w

[^1_37]: https://snapcraft.io/micro-ros-agent

[^1_38]: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

[^1_39]: https://www.reddit.com/r/ROS/comments/1cyvjfg/ros_2_jazzy_jalisco_has_been_released_details/

[^1_40]: https://adityakamath.github.io/2022-07-02-even-more-microros-examples/

[^1_41]: https://www.mathworks.com/help/ros/gs/ros2-topics.html

[^1_42]: https://www.youtube.com/watch?v=z2HTDllOKRw

[^1_43]: https://adityakamath.github.io/2022-06-26-more-microros-examples/

[^1_44]: https://answers.ros.org/question/379267/

[^1_45]: https://github.com/alvaro0308/micro-ROS-Teleop-ESP32

[^1_46]: https://github.com/mgonzs13/esp32_ros

[^1_47]: https://www.youtube.com/watch?v=z4redsBi-Uo

[^1_48]: https://discourse.ros.org/t/ros-2-jazzy-jalisco-released/37862

[^1_49]: https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca

[^1_50]: https://qiita.com/ousagi_sama/items/b4eb3d9c6b337cbe1b05

[^1_51]: https://www.youtube.com/watch?v=kQWRPGLg6OI

[^1_52]: https://docs.ros.org/en/humble/Releases/Release-Jazzy-Jalisco.html

[^1_53]: https://gist.github.com/rasheeddo/5a6dd95b206233ad58bda8304ae2f30d

[^1_54]: https://automaticaddison.com/how-to-create-a-ros-2-python-subscriber-iron/

[^1_55]: https://www.reddit.com/r/ROS/comments/vgycwx/microros_node_on_esp32_that_both_publishes_and/

[^1_56]: https://github.com/micro-ROS/micro_ros_setup/issues/234

[^1_57]: https://www.youtube.com/watch?v=od3JwOeyEXc

[^1_58]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

[^1_59]: https://www.theconstruct.ai/ros2-programming-basics-using-python/

[^1_60]: https://github.com/micro-ROS/micro_ros_espidf_component

[^1_61]: https://ppl-ai-code-interpreter-files.s3.amazonaws.com/web/direct-files/8d9aff20712c0dc808dee3a1918c6910/b500273d-9cec-4b5c-99bc-498499d85735/422142d4.ino

[^1_62]: https://ppl-ai-code-interpreter-files.s3.amazonaws.com/web/direct-files/8d9aff20712c0dc808dee3a1918c6910/3808b7d3-c2a7-4dd1-ab48-8ed0e178d6fd/dc0c08fa.py
