#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/color_rgba.h>

// ESP32 Pins
#define S0_PIN 16    // (I2C SCL) – actually used for TCS3200 frequency
#define S1_PIN 17    // (I2C SDA) – actually used for TCS3200 frequency
#define S2_PIN 39
#define S3_PIN 32
#define OUT_PIN 33

// LED pin to indicate connection status
#define LED_PIN 35

// 353,319,318 
// 680,628,626
// Calibration ranges (update after measuring raw values)
long R_min = 353, R_max = 680;
long G_min = 319, G_max = 628;
long B_min = 318, B_max = 626;

// WiFi credentials
const char* ssid = "smartspacekk";
const char* password = "smartspace09";

// Host PC IP address (where micro-ROS agent is running)
const char* host_ip = "192.168.0.108";  // Change to your PC's IP
const int host_port = 8888;

// Color palette (0-255 RGB)
struct Color {
  const char* name;
  int r, g, b;
};

Color palette[] = {
  {"Red",     255,   0,   0},
  {"Black",     0,   0,   0},
  // {"Green",     0, 255,   0},
  // {"Blue",      0,   0, 255},
  {"White",   255, 255, 255},
  // {"Yellow",  255, 255,   0},
};

const int PALETTE_SIZE = sizeof(palette)/sizeof(palette[0]);

// ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t color_name_publisher;
rcl_publisher_t color_rgb_publisher;
rcl_publisher_t raw_rgb_publisher;
rcl_timer_t timer;
rclc_executor_t executor;
std_msgs__msg__String color_name_msg;
std_msgs__msg__String raw_rgb_msg;
std_msgs__msg__ColorRGBA color_rgb_msg;

// Connection state management
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Global variables for non-blocking operation
volatile int last_r = 0, last_g = 0, last_b = 0;
String last_color_name = "Unknown";

// Raw readings
long r_raw = 0;
long g_raw = 0;
long b_raw = 0;

// Helper macro for periodic execution
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

long readColor(byte s2, byte s3) {
  digitalWrite(S2_PIN, s2);
  digitalWrite(S3_PIN, s3);
  delay(10); // stabilize output
  long val = pulseIn(OUT_PIN, LOW, 2000000UL); // raw pulse in microseconds
  return val > 0 ? val : 1; // avoid zero
}

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

// Timer callback - publishes color data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // Prepare color name message
    color_name_msg.data.data = (char*)last_color_name.c_str();
    color_name_msg.data.size = last_color_name.length();
    color_name_msg.data.capacity = last_color_name.length() + 1;

    // Prepare RGB color message (normalized to 0.0–1.0)
    color_rgb_msg.r = last_r / 255.0f;
    color_rgb_msg.g = last_g / 255.0f;
    color_rgb_msg.b = last_b / 255.0f;
    color_rgb_msg.a = 1.0f;

    // Prepare raw RGB as a string: "r,g,b"
    char raw_buffer[64];
    snprintf(raw_buffer, sizeof(raw_buffer), "%ld,%ld,%ld", r_raw, g_raw, b_raw);
    raw_rgb_msg.data.data = raw_buffer;
    raw_rgb_msg.data.size = strlen(raw_buffer);
    raw_rgb_msg.data.capacity = sizeof(raw_buffer);

    // Publish messages
    rcl_ret_t ret_raw = rcl_publish(&raw_rgb_publisher, &raw_rgb_msg, NULL);
    rcl_ret_t ret_name = rcl_publish(&color_name_publisher, &color_name_msg, NULL);
    rcl_ret_t ret_rgb = rcl_publish(&color_rgb_publisher, &color_rgb_msg, NULL);

    if (ret_raw != RCL_RET_OK) {
      Serial.println("Failed to publish raw rgb message");
    }
    if (ret_name != RCL_RET_OK) {
      Serial.println("Failed to publish color name message");
    }
    if (ret_rgb != RCL_RET_OK) {
      Serial.println("Failed to publish color RGB message");
    }

    if (ret_raw == RCL_RET_OK && ret_name == RCL_RET_OK && ret_rgb == RCL_RET_OK) {
      Serial.printf("Published: %s RGB(%.2f,%.2f,%.2f) Raw(%ld,%ld,%ld)\n",
                    last_color_name.c_str(),
                    color_rgb_msg.r, color_rgb_msg.g, color_rgb_msg.b,
                    r_raw, g_raw, b_raw);
    }
  }
}

// Create ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Initialize support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "color_sensor_node", "", &support) != RCL_RET_OK) {
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

  // Create publisher for raw RGB (as string)
  if (rclc_publisher_init_default(
      &raw_rgb_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/color_sensor/raw_rgb") != RCL_RET_OK) {
    return false;
  }

  // Create publisher for normalized RGB (ColorRGBA)
  if (rclc_publisher_init_default(
      &color_rgb_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
      "/color_sensor/rgb") != RCL_RET_OK) {
    return false;
  }

  // Create timer (publish every 200 ms)
  const unsigned int timer_timeout = 200;
  if (rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback) != RCL_RET_OK) {
    return false;
  }

  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Add timer to executor
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
    return false;
  }

  return true;
}

// Destroy ROS entities
void destroy_entities() {
  rcl_publisher_fini(&raw_rgb_publisher, &node);
  rcl_publisher_fini(&color_name_publisher, &node);
  rcl_publisher_fini(&color_rgb_publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting roboneo_bot color sensor...");

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(OUT_PIN, INPUT);

  // Set TCS3200 frequency scaling to 20%
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());

  // Set micro-ROS transport
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)host_ip, host_port);

  state = WAITING_AGENT;
  delay(1000);

  Serial.println("ESP32 TCS3200 Color Sensor - ROS2 Publisher Ready");
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      });
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
        Serial.println("Connected to ROS2 agent and created entities");
      } else {
        state = WAITING_AGENT;
        destroy_entities();
        Serial.println("Failed to create entities, retrying...");
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, {
        readColorSensor();
      });

      EXECUTE_EVERY_N_MS(500, {
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      });

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      Serial.println("Disconnected from ROS2 agent, retrying...");
      state = WAITING_AGENT;
      break;

    default:
      break;
  }

  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);
  delay(10);
}