#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <CANSAME5x.h>

// CAN Configuration
CANSAME5x CAN;

// CAN IDs and Enums
enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

enum control_mode {
  Duty_Cycle_Set = 0x2050080,
  Speed_Set = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set = 0x2050C80,
  Voltage_Set = 0x2051080,
  Current_Set = 0x20510C0,
  Smart_Motion_Set = 0x2051480
};

// Control Frame
const uint8_t CONTROL_SIZE = 8;
const uint8_t STATUS_SIZE = 8;
const uint8_t DRIVE_MOTOR_COUNT = 6;

// Micro-ROS Variables
rcl_publisher_t status_publisher;
rcl_subscription_t velocity_subscriber;
rcl_subscription_t idle_mode_subscriber;
std_msgs__msg__Float32MultiArray status_msg;
std_msgs__msg__Float32MultiArray velocity_msg;
std_msgs__msg__Bool idle_mode_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Status layout: 6 motors * 6 values (Velocity, Position, Current, Voltage, Temp, Faults)
const uint8_t VALUES_PER_MOTOR = 6;
float status_data[DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR]; 
float target_velocities[DRIVE_MOTOR_COUNT];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Helper to convert float to bytes
void floatToBytes(float val, uint8_t* bytes_array) {
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  u.float_variable = val;
  memcpy(bytes_array, u.temp_array, 4);
}

void send_control_frame(const uint32_t device_id, const control_mode mode, const float setpoint) {
  uint32_t control_id = mode + device_id;
  uint8_t control_data[CONTROL_SIZE];
  
  // Clear buffer
  memset(control_data, 0, CONTROL_SIZE);
  
  // REV format for Set Point: 
  // Bytes 0-3: Float 32 (Little Endian)
  // Bytes 4-7: Aux (0)
  memcpy(control_data, &setpoint, sizeof(setpoint));

  CAN.beginExtendedPacket(control_id);
  CAN.write(control_data, CONTROL_SIZE);
  CAN.endPacket();
}

void send_config_parameter(const uint32_t device_id, uint8_t param_id, float value) {
  // Use Set Config Parameter (0x2051C00)
  uint32_t config_id = 0x2051C00 + device_id;
  uint8_t data[8] = {0};
  
  data[0] = param_id;
  data[1] = 0;
  // REV Param Value: Float 32 Little Endian
  memcpy(&data[2], &value, 4);
  data[6] = 0; // Type 0
  
  CAN.beginExtendedPacket(config_id);
  CAN.write(data, 8);
  CAN.endPacket();
}

void idle_mode_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  // 1.0 = Brake, 0.0 = Coast
  float mode_val = msg->data ? 1.0f : 0.0f; 
  const uint8_t kIdleMode = 10;
  
  for(int i=1; i<=DRIVE_MOTOR_COUNT; i++) {
     send_config_parameter(i, kIdleMode, mode_val);
     // Small delay to prevent flooding CAN buffer
     delay(2); 
  }
}

// Subscription Callback
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  if (msg->data.size >= DRIVE_MOTOR_COUNT) {
    for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
        target_velocities[i] = msg->data.data[i];
        send_control_frame(i + 1, Smart_Velocity_Set, target_velocities[i]);
    }
  }
}

// Timer Callback (Publish Status)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    status_msg.data.data = status_data;
    status_msg.data.size = DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR;
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
  }
}

void check_for_can_packets() {
    // Try to read all available packets
    while (CAN.parsePacket()) {
        uint32_t packet_id = CAN.packetId();
        
        // Identify motor index and packet type
        int motor_index = -1;
        
        // Check for Status 0, 1, 2
        // Status 0: 0x2051800 + ID
        // Status 1: 0x2051840 + ID
        // Status 2: 0x2051880 + ID
        
        if (packet_id > status_0 && packet_id <= status_0 + DRIVE_MOTOR_COUNT) {
            motor_index = packet_id - status_0 - 1;
            
            if (CAN.available() >= 8) {
               uint8_t buffer[8];
               for (int i=0; i<8; i++) buffer[i] = CAN.read();
               
               // Status 0: [AppliedOut(2), Faults(2), Sticky(2), ...]
               // Faults at bytes 2-3 (Little Endian)
               uint16_t faults = buffer[2] | (buffer[3] << 8);
               
               int base_idx = motor_index * VALUES_PER_MOTOR;
               status_data[base_idx + 5] = (float)faults;
            }
        } 
        else if (packet_id > status_1 && packet_id <= status_1 + DRIVE_MOTOR_COUNT) {
            motor_index = packet_id - status_1 - 1;
            
            if (CAN.available() >= 8) {
               uint8_t buffer[8];
               for (int i=0; i<8; i++) buffer[i] = CAN.read(); 
               
               // Status 1
               // Bytes 0-3: Velocity (Float)
               float velocity = 0.0f;
               memcpy(&velocity, buffer, 4);
               
               // Byte 4: Temp (uint8)
               uint8_t temp = buffer[4];
               
               // Byte 5-6: Voltage (12-bit)
               // Byte 5: LSB. Byte 6 (0-3): MSB.
               uint16_t v_raw = buffer[5] | ((buffer[6] & 0x0F) << 8);
               float voltage = v_raw / 128.0f; 
               
               // Byte 6-7: Current (12-bit)
               // Byte 6 (4-7): LSB. Byte 7: MSB.
               uint16_t c_raw = (buffer[6] >> 4) | (buffer[7] << 4);
               float current = c_raw / 128.0f;
               
               int base_idx = motor_index * VALUES_PER_MOTOR;
               status_data[base_idx + 0] = velocity;
               status_data[base_idx + 2] = current;
               status_data[base_idx + 3] = voltage;
               status_data[base_idx + 4] = (float)temp;
            }
        }
        else if (packet_id > status_2 && packet_id <= status_2 + DRIVE_MOTOR_COUNT) {
            motor_index = packet_id - status_2 - 1;
            
            if (CAN.available() >= 8) {
               uint8_t buffer[8];
               for (int i=0; i<8; i++) buffer[i] = CAN.read(); 
               
               // Status 2
               // Bytes 0-3: Position (Float32)
               float position = 0.0f;
               memcpy(&position, buffer, 4);
               
               int base_idx = motor_index * VALUES_PER_MOTOR;
               status_data[base_idx + 1] = position;
            }
        }
        else {
          // Flush unknown
          while(CAN.available()) CAN.read();
        }
    }
}

void setup() {
  // Serial setup for micro-ROS
  Serial.begin(115200);
  set_microros_transports();
  
  // CAN Setup
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); 
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);
  
  if (!CAN.begin(1000000)) {
    // If CAN fails, we can't do much. 
    // Maybe blink LED rapidly?
    // We'll proceed so ROS might connect and report something.
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize Support with retry logic
  // Loop until the agent is answering
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  while (rc != RCL_RET_OK) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink while waiting
      delay(250);
      rc = rclc_support_init(&support, 0, NULL, &allocator);
  }
  
  // Turn LED solid to indicate connection
  digitalWrite(LED_BUILTIN, HIGH); 

  // Initialize Node
  RCCHECK(rclc_node_init_default(&node, "rover_drive_node", "", &support));

  // Initialize Publisher
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_status"));

  // Initialize Subscriber
  RCCHECK(rclc_subscription_init_default(
    &velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_velocities_cmd"));

  // Initialize Idle Mode Subscriber
  RCCHECK(rclc_subscription_init_default(
    &idle_mode_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_idle_mode"));

  // Initialize Timer (e.g. 50ms -> 20Hz)
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); // 3 handles: sub_vel + sub_idle + timer
  RCCHECK(rclc_executor_add_subscription(&executor, &velocity_subscriber, &velocity_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &idle_mode_subscriber, &idle_mode_msg, &idle_mode_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Allocate memory for messages
  // Velocity msg (Subscriber) - Pre-allocate to max 6
  static float velocity_memory[DRIVE_MOTOR_COUNT];
  velocity_msg.data.capacity = DRIVE_MOTOR_COUNT;
  velocity_msg.data.data = velocity_memory;
  velocity_msg.data.size = 0;

  // Status msg (Publisher)
  status_msg.data.capacity = DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR;
  status_msg.data.data = status_data;
  status_msg.data.size = DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR;
  
  // Initialize status headers (optional, relying on layout convention)
  status_msg.layout.dim.capacity = 0;
  status_msg.layout.dim.size = 0;
}

void loop() {
  // Read CAN non-blockingly
  check_for_can_packets();
  
  // Handle ROS Communication
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  // Heartbeat to motors?
  // Original code sent heartbeat frame 0x2052C80 every loop.
  // We should do that periodically. Or on every timer callback?
  // Let's do it in the loop with a simple non-blocking timer.
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 50) {
     CAN.beginExtendedPacket(0x2052C80); // HEARTBEAT_ID
     uint8_t hb_data[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
     CAN.write(hb_data, 8);
     CAN.endPacket();
     last_heartbeat = millis();
  }
}
