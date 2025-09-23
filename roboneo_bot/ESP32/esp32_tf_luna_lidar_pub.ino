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
#include <Wire.h>        // instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library

TFLI2C tflI2C;

int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // use this default I2C address or set to your own value

// WiFi credentials
const char* ssid = "smartspacekk";
const char* password = "smartspace09";

// Host PC IP address (where micro-ROS agent is running)
const char* host_ip = "192.168.0.108";  // Change to your PC's IP
const int host_port = 8888;

// LED pin to indicate connection status
#define LED_PIN 2

// ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
std_msgs__msg__Float32 distance_msg;

// Connection state management
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Global variables for non-blocking operation
volatile float last_distance = 0.0;

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

// Timer callback - publishes distance measurement
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // Publish the last known distance
    distance_msg.data = last_distance;
    
    // Publish message
    rcl_ret_t ret = rcl_publish(&publisher, &distance_msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Failed to publish distance message");
    } else {
      Serial.print("Published distance: ");
      Serial.print(last_distance);
      Serial.println(" cm");
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
  if (rclc_node_init_default(&node, "roboneo_bot", "", &support) != RCL_RET_OK) {
    return false;
  }

  // Create publisher for distance data
  if (rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/tfluna/distance") != RCL_RET_OK) {
    return false;
  }

  // Create timer for distance data (publish every 100 milliseconds)
  const unsigned int timer_timeout = 100;
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
  (void) rcl_publisher_fini(&publisher, &node);
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
  Serial.println("Starting roboneo_bot...");
  pinMode(LED_PIN, OUTPUT);

  // Initialize I2C for TF Luna
  Wire.begin(22, 21);  // SDA=22, SCL=21 (you can change these pins if needed)
  // Wire.begin();     // Use this instead if you want to use default I2C pins

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
      EXECUTE_EVERY_N_MS(100, {
        float d = readLidarDistance();
        if (d >= 0) {
          last_distance = d;
        }
      });

      // Check agent connection more frequently
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );

      // Spin the executor for ROS communication
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
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