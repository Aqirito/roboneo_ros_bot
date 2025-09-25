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
#define S0_PIN 25    // (I2C SCL)
#define S1_PIN 26    // (I2C SDA)  
#define S2_PIN 19
#define S3_PIN 18   
#define OUT_PIN 34   

// LED pin to indicate connection status
#define LED_PIN 27   

// Calibration ranges (update after measuring raw values)
long R_min = 503, R_max = 1268;
long G_min = 521, G_max = 1336;
long B_min = 443, B_max = 1151;

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
  {"Green",     0, 255,   0},
  {"Blue",      0,   0, 255},
  {"White",   255, 255, 255},
  {"Black",     0,   0,   0},
  {"Yellow",  255, 255,   0},
  {"Cyan",      0, 255, 255},
  {"Magenta", 255,   0, 255},
  {"Orange",  255, 165,   0},
};

const int PALETTE_SIZE = sizeof(palette)/sizeof(palette[0]);

// ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t color_name_publisher;
rcl_publisher_t color_rgb_publisher;
rcl_timer_t timer;
rclc_executor_t executor;
std_msgs__msg__String color_name_msg;
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
volatile String last_color_name = "Unknown";

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
  long r_raw = readColor(LOW, LOW);
  long g_raw = readColor(HIGH, HIGH);
  long b_raw = readColor(LOW, HIGH);

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

    // Prepare RGB color message
    color_rgb_msg.r = last_r / 255.0f;  // Normalize to 0-1
    color_rgb_msg.g = last_g / 255.0f;
    color_rgb_msg.b = last_b / 255.0f;
    color_rgb_msg.a = 1.0f;  // Alpha (opacity)

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

    if (ret1 == RCL_RET_OK && ret2 == RCL_RET_OK) {
      Serial.printf("Published: %s RGB(%.2f,%.2f,%.2f)\n", 
                    last_color_name.c_str(), 
                    color_rgb_msg.r, color_rgb_msg.g, color_rgb_msg.b);
    }
  }
}// 
Create ROS entities
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

  // Create publisher for RGB values
  if (rclc_publisher_init_default(
      &color_rgb_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
      "/color_sensor/rgb") != RCL_RET_OK) {
    return false;
  }

  // Create timer for color data (publish every 200 milliseconds)
  const unsigned int timer_timeout = 200;
  if (rclc_timer_init_default2(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback,
      true) != RCL_RET_OK) {
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
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  (void) rcl_publisher_fini(&color_name_publisher, &node);
  (void) rcl_publisher_fini(&color_rgb_publisher, &node);
  (void) rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  rclc_support_fini(&support);
  
  // Small delay to ensure proper cleanup
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor to connect
  Serial.println("Starting roboneo_bot color sensor...");
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(S0_PIN, OUTPUT); 
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT); 
  pinMode(S3_PIN, OUTPUT);
  pinMode(OUT_PIN, INPUT);

  // Frequency scaling 20%
  digitalWrite(S0_PIN, HIGH); 
  digitalWrite(S1_PIN, LOW);

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

  // Set micro-ROS WiFi transport
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)host_ip, host_port);

  state = WAITING_AGENT;
  
  // Small delay to ensure transport is properly initialized
  delay(1000);
  
  Serial.println("ESP32 TCS3200 Color Sensor - ROS2 Publisher Ready");
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
        Serial.println("Connected to ROS2 agent and created entities");
      } else {
        state = WAITING_AGENT;
        destroy_entities();
        Serial.println("Failed to create entities, retrying...");
      }
      break;

    case AGENT_CONNECTED:
      // Read color sensor periodically (non-blocking)
      EXECUTE_EVERY_N_MS(200, {
        readColorSensor();
      });

      // Check agent connection
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );

      // Spin the executor for ROS communication
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

  // LED indicates connection status
  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);

  delay(10);
}