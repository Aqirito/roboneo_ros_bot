#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/color_rgba.h>
#include <Wire.h>        // instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library


// Sensor availability flags
// 0 = disabled, 1 = enabled
#define USE_TFLUNA 0 
#define USE_ULTRASONIC 0
#define USE_COLOR_SENSOR 0

// TF-Luna LiDAR
TFLI2C tflI2C;
int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // use this default I2C address or set to your own value

// HC-SR04 Ultrasonic Sensor pins
const int trigPin = 25;
const int echoPin = 26;

// ESP32 Pins for TCS3200 Color Sensor
#define S0_PIN 16    // (I2C SCL)
#define S1_PIN 17    // (I2C SDA)
#define S2_PIN 39
#define S3_PIN 32
#define OUT_PIN 33

// LED pin to indicate connection status
#define LED_PIN 2

// Calibration ranges for color sensor (update after measuring raw values)
long R_min = 330, R_max = 1000;
long G_min = 330, G_max = 1000;
long B_min = 330, B_max = 1000;
// WiFi credentials
const char* ssid = "smartspacekk";
const char* password = "smartspace09";

// Host PC IP address (where micro-ROS agent is running)
const char* host_ip = "192.168.0.108";  // Change to your PC's IP
const int host_port = 8888;

// Motor control pins (example pins, adjust as needed for your hardware)
#define LEFT_MOTOR_PIN1 12
#define LEFT_MOTOR_PIN2 13
#define RIGHT_MOTOR_PIN1 14
#define RIGHT_MOTOR_PIN2 27

// PWM channels for motor speed control
#define LEFT_MOTOR_CHANNEL 0
#define RIGHT_MOTOR_CHANNEL 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// Color palette (0-255 RGB)
struct Color {
  const char* name;
  int r, g, b;
};

Color palette[] = {
  {"Red",     255,   0,   0},
  {"Green",     0, 255,   0},
  {"Blue",      0,   0, 255},
  {"White",   255, 255, 255},
  {"Black",     0,   0,   0},
  {"Yellow",  255, 150,   100},
  {"Tiles",  160, 80,   80},
//  {"Cyan",      0, 255, 255},
//  {"Magenta", 255,   0, 255},
//  {"Orange",  255, 165,   0},
};

const int PALETTE_SIZE = sizeof(palette)/sizeof(palette[0]);

// ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t lidar_publisher;
rcl_publisher_t ultrasonic_publisher;
rcl_publisher_t color_name_publisher;
rcl_publisher_t color_rgb_publisher;
rcl_publisher_t raw_rgb_publisher;  // Raw RGB publisher
rcl_subscription_t subscriber;
rcl_timer_t lidar_timer;
rcl_timer_t ultrasonic_timer;
rcl_timer_t color_timer;
rclc_executor_t executor;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 lidar_distance_msg;
std_msgs__msg__Float32 ultrasonic_distance_msg;
std_msgs__msg__String color_name_msg;
std_msgs__msg__ColorRGBA color_rgb_msg;
std_msgs__msg__String raw_rgb_msg;  // Raw RGB message

// Connection state management
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Global variables for non-blocking operation
volatile float last_lidar_distance = 0.0;
volatile float last_ultrasonic_distance = 0.0;
volatile int last_r = 0, last_g = 0, last_b = 0;
String last_color_name = "Unknown";
volatile bool new_twist_received = false;
volatile unsigned long last_motor_update = 0;

// Raw RGB readings
long r_raw = 0;
long g_raw = 0;
long b_raw = 0;

// Helper macro for periodic execution
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Function to read distance from TF Luna via I2C
float readLidarDistance() {
  if (tflI2C.getData(tfDist, tfAddr)) {
    return (float)tfDist; // Distance is already in cm
  }
  return -1.0f; // Return -1 if no valid data
}

// Function to read distance from HC-SR04 sensor (non-blocking)
float readUltrasonicDistance() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Non-blocking echo reading with timeout
  unsigned long start_time = micros();
  while (digitalRead(echoPin) == LOW && (micros() - start_time) < 50000UL); // Wait for HIGH or timeout (50ms)

  start_time = micros();
  while (digitalRead(echoPin) == HIGH && (micros() - start_time) < 50000UL); // Wait for LOW or timeout

  long duration = micros() - start_time;
  
  // Calculate distance in centimeters
  if (duration > 0 && duration < 50000) {
    return duration * 0.01715f;
  } else {
    return -1.0f; // Timeout or invalid reading
  }
}

// Function to read color from TCS3200 sensor
long readColor(byte s2, byte s3) {
  digitalWrite(S2_PIN, s2);
  digitalWrite(S3_PIN, s3);
  delay(10); // stabilize output
  long val = pulseIn(OUT_PIN, LOW, 2000000UL); // raw pulse in microseconds
  return val > 0 ? val : 1; // avoid zero
}

// Function to find closest color in palette
String closestColor(int r, int g, int b) {
  float bestDist = 1e6;
  const char* best = "Unknown";

  for(int i = 0; i < PALETTE_SIZE; i++) {
    float dr = r - palette[i].r;
    float dg = g - palette[i].g;
    float db = b - palette[i].b;
    float dist = sqrt(dr*dr + dg*dg + db*db);

    if(dist < bestDist) {
      bestDist = dist;
      best = palette[i].name;
    }
  }
  return String(best);
}

// Function to read color sensor data
void readColorSensor() {
  // Raw readings
  r_raw = readColor(LOW, LOW);
  g_raw = readColor(HIGH, HIGH);
  b_raw = readColor(LOW, HIGH);

  // Map raw readings to 0-255
  int r = constrain(map(r_raw, R_min, R_max, 255, 0), 0, 255);
  int g = constrain(map(g_raw, G_min, G_max, 255, 0), 0, 255);
  int b = constrain(map(b_raw, B_min, B_max, 255, 0), 0, 255);

  // Update global variables
  last_r = r;
  last_g = g;
  last_b = b;
  last_color_name = closestColor(r, g, b);

  // Print to serial for debugging
  Serial.printf("Raw -> R:%6ld G:%6ld B:%6ld\tMapped RGB(%3d,%3d,%3d) -> %s\n",
                r_raw, g_raw, b_raw, r, g, b, last_color_name.c_str());
}

// Function to control motors based on Twist message
void controlMotors(float linear_x, float angular_z) {
  // Simple differential drive kinematics
  // Convert linear and angular velocities to left and right wheel speeds
  float left_speed = linear_x - angular_z;
  float right_speed = linear_x + angular_z;
  
  // Normalize speeds to -1.0 to 1.0 range
  float max_speed = fmax(fabs(left_speed), fabs(right_speed));
  if (max_speed > 1.0) {
    left_speed /= max_speed;
    right_speed /= max_speed;
  }
  
  // Convert to PWM values (0-255)
  int left_pwm = (int)(left_speed * 127);
  int right_pwm = (int)(right_speed * 127);
  
  // Apply motor control with PWM
  if (left_pwm > 0) {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    ledcWrite(LEFT_MOTOR_CHANNEL, left_pwm);
  } else if (left_pwm < 0) {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    ledcWrite(LEFT_MOTOR_CHANNEL, -left_pwm);
  } else {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    ledcWrite(LEFT_MOTOR_CHANNEL, 0);
  }
  
  if (right_pwm > 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    ledcWrite(RIGHT_MOTOR_CHANNEL, right_pwm);
  } else if (right_pwm < 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    ledcWrite(RIGHT_MOTOR_CHANNEL, -right_pwm);
  } else {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    ledcWrite(RIGHT_MOTOR_CHANNEL, 0);
  }
  
  Serial.print("Left motor: ");
  Serial.print(left_pwm);
  Serial.print(", Right motor: ");
  Serial.println(right_pwm);
}

// Timer callback - publishes LiDAR distance measurement
void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // Publish the last known LiDAR distance
    lidar_distance_msg.data = last_lidar_distance;
    
    // Publish message
    rcl_ret_t ret = rcl_publish(&lidar_publisher, &lidar_distance_msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Failed to publish LiDAR distance message");
    } else {
      Serial.print("Published LiDAR distance: ");
      Serial.print(last_lidar_distance);
      Serial.println(" cm");
    }
  }
}

// Timer callback - publishes ultrasonic distance measurement
void ultrasonic_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // Publish the last known ultrasonic distance
    ultrasonic_distance_msg.data = last_ultrasonic_distance;
    
    // Publish message
    rcl_ret_t ret = rcl_publish(&ultrasonic_publisher, &ultrasonic_distance_msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Failed to publish ultrasonic distance message");
    } else {
      Serial.print("Published ultrasonic distance: ");
      Serial.print(last_ultrasonic_distance);
      Serial.println(" cm");
    }
  }
}

// Timer callback - publishes color data
void color_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // Prepare color name message
    color_name_msg.data.data = (char*)last_color_name.c_str();
    color_name_msg.data.size = last_color_name.length();
    color_name_msg.data.capacity = last_color_name.length() + 1;

    // Prepare RGB color message
    color_rgb_msg.r = last_r;
    color_rgb_msg.g = last_g;
    color_rgb_msg.b = last_b;
    color_rgb_msg.a = 255.0;  // Alpha (opacity)

    // Prepare raw RGB as a string: "r,g,b"
    char raw_buffer[64];
    snprintf(raw_buffer, sizeof(raw_buffer), "%ld,%ld,%ld", r_raw, g_raw, b_raw);
    raw_rgb_msg.data.data = raw_buffer;
    raw_rgb_msg.data.size = strlen(raw_buffer);
    raw_rgb_msg.data.capacity = sizeof(raw_buffer);

    // Publish color name
    rcl_ret_t ret1 = rcl_publish(&color_name_publisher, &color_name_msg, NULL);
    if (ret1 != RCL_RET_OK) {
      Serial.println("Failed to publish color name message");
    }

    // Publish RGB values
    rcl_ret_t ret2 = rcl_publish(&color_rgb_publisher, &color_rgb_msg, NULL);
    if (ret2 != RCL_RET_OK) {
      Serial.println("Failed to publish color RGB message");
    }

    // Publish raw RGB values
    rcl_ret_t ret3 = rcl_publish(&raw_rgb_publisher, &raw_rgb_msg, NULL);
    if (ret3 != RCL_RET_OK) {
      Serial.println("Failed to publish raw RGB message");
    }

    if (ret1 == RCL_RET_OK && ret2 == RCL_RET_OK && ret3 == RCL_RET_OK) {
      Serial.printf("Published: %s RGB(%.2f,%.2f,%.2f) Raw(%ld,%ld,%ld)\n",
                    last_color_name.c_str(),
                    color_rgb_msg.r, color_rgb_msg.g, color_rgb_msg.b,
                    r_raw, g_raw, b_raw);
    }
  }
}

// Subscriber callback
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Copy the received values
  twist_msg.linear.x = msg->linear.x;
  twist_msg.angular.z = msg->angular.z;
  
  new_twist_received = true;
  last_motor_update = millis();
  
  // Control motors immediately when new command received
  controlMotors(twist_msg.linear.x, twist_msg.angular.z);
  
  // Print received values
  Serial.print("Received Twist - Linear X: ");
  Serial.print(twist_msg.linear.x);
  Serial.print(", Angular Z: ");
  Serial.println(twist_msg.angular.z);
}

// Create ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Initialize support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "roboneo_bot", "", &support) != RCL_RET_OK) {
    return false;
  }

  // Create subscriber for Twist messages
  if (rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel") != RCL_RET_OK) {
    return false;
  }

  // Create publisher for LiDAR distance data
  if (rclc_publisher_init_default(
      &lidar_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/tfluna/distance") != RCL_RET_OK) {
    return false;
  }

  // Create publisher for ultrasonic distance data
  if (rclc_publisher_init_default(
      &ultrasonic_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/ultrasonic/distance") != RCL_RET_OK) {
    return false;
  }

  // Create publisher for color name
  if (rclc_publisher_init_default(
      &color_name_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/color_sensor/color_name") != RCL_RET_OK) {
    return false;
  }

  // Create publisher for RGB values
  if (rclc_publisher_init_default(
      &color_rgb_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
      "/color_sensor/rgb") != RCL_RET_OK) {
    return false;
  }

  // Create publisher for raw RGB (as string)
  if (rclc_publisher_init_default(
      &raw_rgb_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/color_sensor/raw_rgb") != RCL_RET_OK) {
    return false;
  }

  // Create timer for LiDAR distance data (publish every 100 milliseconds)
  const unsigned int lidar_timer_timeout = 100;
  if (rclc_timer_init_default2(
      &lidar_timer,
      &support,
      RCL_MS_TO_NS(lidar_timer_timeout),
      lidar_timer_callback,
      true) != RCL_RET_OK) {
    return false;
  }

  // Create timer for ultrasonic distance data (publish every 100 milliseconds)
  const unsigned int ultrasonic_timer_timeout = 100;
  if (rclc_timer_init_default2(
      &ultrasonic_timer,
      &support,
      RCL_MS_TO_NS(ultrasonic_timer_timeout),
      ultrasonic_timer_callback,
      true) != RCL_RET_OK) {
    return false;
  }

  // Create timer for color data (publish every 200 milliseconds)
  const unsigned int color_timer_timeout = 200;
  if (rclc_timer_init_default2(
      &color_timer,
      &support,
      RCL_MS_TO_NS(color_timer_timeout),
      color_timer_callback,
      true) != RCL_RET_OK) {
    return false;
  }

  // Create executor (increased to handle subscriber and all timers)
  if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Add subscriber to executor
  if (rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  // Add LiDAR timer to executor
  if (rclc_executor_add_timer(&executor, &lidar_timer) != RCL_RET_OK) {
    return false;
  }

  // Add ultrasonic timer to executor
  if (rclc_executor_add_timer(&executor, &ultrasonic_timer) != RCL_RET_OK) {
    return false;
  }

  // Add color timer to executor
  if (rclc_executor_add_timer(&executor, &color_timer) != RCL_RET_OK) {
    return false;
  }

  return true;
}

// Destroy ROS entities
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void) rcl_subscription_fini(&subscriber, &node);
  (void) rcl_publisher_fini(&lidar_publisher, &node);
  (void) rcl_publisher_fini(&ultrasonic_publisher, &node);
  (void) rcl_publisher_fini(&color_name_publisher, &node);
  (void) rcl_publisher_fini(&color_rgb_publisher, &node);
  (void) rcl_publisher_fini(&raw_rgb_publisher, &node);
  (void) rcl_timer_fini(&lidar_timer);
  (void) rcl_timer_fini(&ultrasonic_timer);
  (void) rcl_timer_fini(&color_timer);
  rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  rclc_support_fini(&support);
  
  // Small delay to ensure proper cleanup
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor to connect
  Serial.println("Starting roboneo_bot with color sensor...");
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  
  // Initialize color sensor pins
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(OUT_PIN, INPUT);
  
  // Frequency scaling 20% for color sensor
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);
  
  // Initialize PWM channels
  ledcSetup(LEFT_MOTOR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_PIN1, LEFT_MOTOR_CHANNEL);
  ledcSetup(RIGHT_MOTOR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_CHANNEL);

  // Initialize I2C for TF Luna
  Wire.begin(22, 21);  // SDA=22, SCL=21 (you can change these pins if needed)

  // Set static IP for better WiFi stability (optional)
  // IPAddress local_IP(192, 168, 0, 188);
  // IPAddress gateway(192, 168, 0, 1);
  // IPAddress subnet(255, 255, 255, 0);
  // WiFi.config(local_IP, gateway, subnet);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED while connecting
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());

  // Set micro-ROS WiFi transport (cast const → char*)
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)host_ip, host_port);

  state = WAITING_AGENT;
  
  // Small delay to ensure transport is properly initialized
  delay(1000);
  
  // Initialize twist message
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
        Serial.println("Connected to agent and created entities");
      } else {
        state = WAITING_AGENT;
        destroy_entities();
        Serial.println("Failed to create entities, retrying...");
      }
      break;

    case AGENT_CONNECTED:
      // Read TF Luna LiDAR sensor via I2C periodically (non-blocking)
      if (USE_TFLUNA) {
        EXECUTE_EVERY_N_MS(100, {
          float d = readLidarDistance();
          if (d >= 0) {
            last_lidar_distance = d;
          }
        });
      }
      // Read ultrasonic sensor periodically (non-blocking)
      if (USE_ULTRASONIC) {
        EXECUTE_EVERY_N_MS(100, {
          float d = readUltrasonicDistance();
          if (d >= 0) {
            last_ultrasonic_distance = d;
          }
        });
      }

      // Read color sensor periodically (non-blocking)
      if (USE_COLOR_SENSOR) {
        EXECUTE_EVERY_N_MS(200, {
          readColorSensor();
        });
      }

      // Check agent connection more frequently
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );

      // Spin the executor for ROS communication
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      
      // Ensure motors are updated regularly (every 50ms)
      if (millis() - last_motor_update > 50) {
        controlMotors(twist_msg.linear.x, twist_msg.angular.z);
        last_motor_update = millis();
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      Serial.println("Disconnected from agent, retrying...");
      state = WAITING_AGENT;
      break;

    default:
      break;
  }

  // LED indicates connection status
  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);

  delay(10);
}