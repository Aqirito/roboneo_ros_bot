#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Host PC IP address (where micro-ROS agent is running)
const char* host_ip = "192.168.1.100";  // Change to your PC's IP
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
std_msgs__msg__String hello_msg;

// Connection state management
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Helper macro for periodic execution
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Timer callback - publishes "hello" message
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    static int counter = 0;
    sprintf(hello_msg.data.data, "Hello from ESP32! Count: %d", counter++);
    hello_msg.data.size = strlen(hello_msg.data.data);

    rcl_ret_t ret = rcl_publish(&publisher, &hello_msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Failed to publish message");
    }
  }
}

// Create ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Initialize support structure
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "esp32_hello_publisher", "", &support) != RCL_RET_OK) {
    return false;
  }

  // Create publisher
  if (rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/esp32/hello") != RCL_RET_OK) {
    return false;
  }

  // Create timer (publish every 1 second)
  const unsigned int timer_timeout = 1000;
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
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());

  // Set micro-ROS WiFi transport (cast const → char*)
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)host_ip, host_port);

  // Initialize hello message
  hello_msg.data.data = (char*)malloc(100);
  hello_msg.data.capacity = 100;
  hello_msg.data.size = 0;

  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }

  // LED indicates connection status
  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);

  delay(10);
}
