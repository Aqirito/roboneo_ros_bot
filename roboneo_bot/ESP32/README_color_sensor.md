# TCS3200 Color Sensor Publisher

This ESP32 Arduino sketch reads color data from a TCS3200 color sensor and publishes it to ROS 2 topics using micro-ROS. The sensor detects RGB values and identifies the closest matching color from a predefined palette.

## Features

- Reads RGB values from TCS3200 color sensor
- Maps raw sensor readings to 0-255 RGB values using calibration
- Identifies closest matching color from predefined palette (Red, Green, Blue, White, Black, Yellow, Cyan, Magenta, Orange)
- Publishes both color name and RGB values to separate ROS 2 topics
- WiFi connectivity with automatic reconnection
- Non-blocking sensor reading and ROS communication
- LED status indicator for connection state

## Hardware Requirements

### TCS3200 Color Sensor Module

- **S0, S1**: Frequency scaling pins
- **S2, S3**: Color filter selection pins  
- **OUT**: Frequency output pin
- **VCC**: 3.3V or 5V power
- **GND**: Ground

### ESP32 Pin Connections

```
TCS3200 Pin  →  ESP32 Pin
S0           →  GPIO 25
S1           →  GPIO 26  
S2           →  GPIO 19
S3           →  GPIO 18
OUT          →  GPIO 34 (input only)
VCC          →  3.3V
GND          →  GND
LED          →  GPIO 27 (status indicator)
```

## Software Setup

### 1. Arduino IDE Configuration

Install required libraries:

- **micro_ros_arduino**: For ROS 2 communication
- **WiFi**: ESP32 WiFi library (built-in)

### 2. WiFi and Network Configuration

Update these settings in the code:

```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* host_ip = "192.168.1.100";  // Your PC's IP address
const int host_port = 8888;
```

### 3. Color Sensor Calibration

The sensor requires calibration for accurate color detection:

1. **Gather calibration data** by pointing sensor at pure red, green, and blue objects
2. **Update calibration ranges** in the code:

```cpp
long R_min = 503, R_max = 1268;  // Red channel range
long G_min = 521, G_max = 1336;  // Green channel range  
long B_min = 443, B_max = 1151;  // Blue channel range
```

### 4. Upload and Run

1. Connect ESP32 to computer via USB
2. Select correct board and port in Arduino IDE
3. Upload the sketch
4. Open Serial Monitor (115200 baud) to view debug output

## ROS 2 Interface

### Published Topics

#### `/color_sensor/color_name` (std_msgs/String)

- **Description**: Detected color name from predefined palette
- **Rate**: ~5 Hz (every 200ms)
- **Example values**: "Red", "Green", "Blue", "White", "Black", "Yellow", "Cyan", "Magenta", "Orange", "Unknown"

#### `/color_sensor/rgb` (std_msgs/ColorRGBA)

- **Description**: Normalized RGB values (0.0-1.0 range)
- **Rate**: ~5 Hz (every 200ms)
- **Fields**:
  - `r`: Red component (0.0-1.0)
  - `g`: Green component (0.0-1.0)  
  - `b`: Blue component (0.0-1.0)
  - `a`: Alpha/opacity (always 1.0)

### Node Information

- **Node name**: `color_sensor_node`
- **Namespace**: Default (`/`)

## Usage Instructions

### 1. Start micro-ROS Agent

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. Power ESP32

- Connect power to ESP32
- Wait for WiFi connection (LED will stop blinking)
- Wait for ROS agent connection (LED will stay solid)

### 3. Monitor Color Data

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

### 4. Test Color Detection

Point the sensor at different colored objects:

- **Red objects**: Should detect "Red"
- **Green objects**: Should detect "Green"  
- **Blue objects**: Should detect "Blue"
- **White paper**: Should detect "White"
- **Black objects**: Should detect "Black"

## Calibration Process

### 1. Raw Value Collection

1. Point sensor at pure red object, note raw values in Serial Monitor
2. Repeat for pure green and blue objects
3. Record minimum and maximum values for each color channel

### 2. Update Calibration Constants

```cpp
// Example calibration values (update with your measurements)
long R_min = 500, R_max = 1300;   // Red channel
long G_min = 520, G_max = 1350;   // Green channel
long B_min = 440, B_max = 1200;   // Blue channel
```

### 3. Verify Calibration

- Upload updated code
- Test with known colored objects
- RGB values should map to 0-255 range appropriately

## Troubleshooting

### No ROS Connection

- Verify micro-ROS agent is running on correct port (8888)
- Check ESP32 and PC are on same WiFi network
- Confirm PC IP address in ESP32 code
- Check firewall settings on PC

### Inaccurate Color Detection

- **Recalibrate sensor** with better lighting conditions
- **Adjust sensor distance** (optimal: 1-3cm from object)
- **Update color palette** if needed for your specific colors
- **Check wiring connections**

### WiFi Connection Issues

- Verify SSID and password in code
- Check WiFi signal strength
- Try different WiFi network
- Monitor Serial output for connection status

### Inconsistent Readings

- **Improve lighting**: Use consistent, bright lighting
- **Stable positioning**: Mount sensor securely
- **Clean sensor**: Ensure sensor surface is clean
- **Shield from ambient light**: Use housing if needed

## Advanced Configuration

### Custom Color Palette

Add or modify colors in the palette array:

```cpp
Color palette[] = {
  {"Red",     255,   0,   0},
  {"Green",     0, 255,   0},
  {"Blue",      0,   0, 255},
  {"Purple",  128,   0, 128},  // Add custom color
  // ... more colors
};
```

### Adjust Publishing Rate

Change timer interval in `create_entities()`:

```cpp
const unsigned int timer_timeout = 100;  // 100ms = 10Hz
```

### Modify Frequency Scaling

Change S0/S1 pins for different sensor sensitivity:

```cpp
// Current: 20% frequency scaling
digitalWrite(S0_PIN, HIGH); 
digitalWrite(S1_PIN, LOW);

// Alternative: 2% scaling (more sensitive)
// digitalWrite(S0_PIN, LOW); 
// digitalWrite(S1_PIN, HIGH);
```

## Technical Details

### Sensor Operation

- **Light source**: Built-in white LEDs illuminate target
- **Photodiodes**: Detect reflected light through color filters
- **Output**: Square wave frequency proportional to light intensity
- **Measurement**: `pulseIn()` measures pulse duration

### Color Matching Algorithm

1. Read raw RGB frequencies from sensor
2. Map to 0-255 range using calibration values
3. Calculate Euclidean distance to each palette color
4. Return closest matching color name

### Performance Characteristics

- **Reading rate**: ~5 Hz
- **Detection range**: 1-10cm (optimal: 2-3cm)
- **Color accuracy**: Depends on calibration and lighting
- **Response time**: ~200ms per reading
