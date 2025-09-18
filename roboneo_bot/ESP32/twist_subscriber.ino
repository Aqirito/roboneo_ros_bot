#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Host PC IP address (where micro-ROS agent is running)
const char* host_ip = "192.168.1.100";  // Change to your PC's IP
const int host_port = 8888;

// LED pin to indicate connection status
#define LED_PIN 2

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

// ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Twist twist_msg;

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
  int left_pwm = (int)(left_speed * 255);
  int right_pwm = (int)(right_speed * 255);
  
  // Apply motor control (simplified for example)
  if (left_pwm > 0) {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    ledcWrite(LEFT_MOTOR_CHANNEL, left_pwm); // Uncomment for PWM control
  } else if (left_pwm < 0) {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    ledcWrite(LEFT_MOTOR_CHANNEL, -left_pwm); // Uncomment for PWM control
  } else {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    ledcWrite(LEFT_MOTOR_CHANNEL, 0); // Uncomment for PWM control
  }
  
  if (right_pwm > 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    ledcWrite(RIGHT_MOTOR_CHANNEL, right_pwm); // Uncomment for PWM control
  } else if (right_pwm < 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    ledcWrite(RIGHT_MOTOR_CHANNEL, -right_pwm); // Uncomment for PWM control
  } else {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    ledcWrite(RIGHT_MOTOR_CHANNEL, 0); // Uncomment for PWM control
  }
  
  Serial.print("Left motor: ");
  Serial.print(left_pwm);
  Serial.print(", Right motor: ");
  Serial.println(right_pwm);
}

// Subscriber callback
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Extract linear and angular components
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  // Control motors based on Twist message
  controlMotors(linear_x, angular_z);
  
  // Print received values
  Serial.print("Received Twist - Linear X: ");
  Serial.print(linear_x);
  Serial.print(", Angular Z: ");
  Serial.println(angular_z);
}

// Create ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Initialize support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "esp32_twist_subscriber", "", &support) != RCL_RET_OK) {
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

  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Add subscriber to executor
  if (rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  return true;
}

// Destroy ROS entities
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void) rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  
  // Initialize PWM channels (uncomment for PWM control)
  ledcSetup(LEFT_MOTOR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_PIN1, LEFT_MOTOR_CHANNEL);
  ledcSetup(RIGHT_MOTOR_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_CHANNEL);

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
