#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
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
std_msgs__msg__Float32MultiArray status_msg;
std_msgs__msg__Float32MultiArray velocity_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Data Storage
// Status layout: 6 motors * 4 values (Velocity, Temp, Voltage, Current)
float status_data[DRIVE_MOTOR_COUNT * 4]; 
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
  
  // Clear buffer (optional, but good practice)
  memset(control_data, 0, CONTROL_SIZE);
  
  // The original code used memcpy(&setpoint, control_data, sizeof(setpoint)), which writes 4 bytes.
  // The rest of the 8 bytes are 0?
  // "memcpy(control_data, &setpoint, sizeof(setpoint));"
  // This copies the float to the first 4 bytes of control_data.
  memcpy(control_data, &setpoint, sizeof(setpoint));

  CAN.beginExtendedPacket(control_id);
  CAN.write(control_data, CONTROL_SIZE);
  CAN.endPacket();
}

// Subscription Callback
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  if (msg->data.size >= DRIVE_MOTOR_COUNT) {
    for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
        target_velocities[i] = msg->data.data[i];
        // Send CAN message immediately upon receipt, or update state and send in loop?
        // Usually sending 6 CAN frames in interrupt/callback is okay if brief.
        // Or we can just store them and send in a timer loop.
        // Let's send immediately to minimize latency, assuming executor spin is frequent.
        send_control_frame(i + 1, Smart_Velocity_Set, target_velocities[i]);
    }
  }
}

// Timer Callback (Publish Status)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Fill status message
    // status_data is updated in the loop() via checkForCanPackets()
    
    // Assign data pointer (already assigned in setup but safe to ensure)
    status_msg.data.data = status_data;
    status_msg.data.size = DRIVE_MOTOR_COUNT * 4;
    
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
  }
}

void check_for_can_packets() {
    // Try to read all available packets
    while (CAN.parsePacket()) {
        uint32_t packet_id = CAN.packetId();
        
        // Identify which motor this is for
        // IDs are base + device_id (1-6)
        int motor_index = -1;
        uint32_t base_id = 0;
        
        // Check for Status 1 (Velocity, Temp, Volt, Curr)
        // 0x2051840 + ID
        if (packet_id > status_1 && packet_id <= status_1 + DRIVE_MOTOR_COUNT) {
            motor_index = packet_id - status_1 - 1; // 0-indexed
            base_id = status_1;
        }

        if (motor_index >= 0 && motor_index < DRIVE_MOTOR_COUNT) {
            uint8_t buffer[8];
            if (CAN.available() >= 8) {
               for (int i=0; i<8; i++) buffer[i] = CAN.read(); 
            } else {
                continue; // Malformed
            }
            
            // Unpack Status 1
            // Bytes 0-3: Velocity (Float 32 Little Endian)
            // Bytes 4-7: Packed Temp/Volt/Curr
            
            // Unpack Velocity
            float velocity = 0.0f;
            memcpy(&velocity, buffer, 4);
            
            // Unpack Other
            // Byte 4: Temp (int8)
            int8_t temp = (int8_t)buffer[4];
            
            // Byte 5-6: Voltage (12-bit)
            // V11..V4 in Byte 5
            // V3..V0 in Byte 6 [7..4]
            uint16_t v_fixed = ((uint16_t)buffer[5] << 4) | (buffer[6] >> 4);
            float voltage = v_fixed * 12.0f / 4095.0f;
            
            // Byte 6-7: Current (12-bit)
            // C11..C8 in Byte 6 [3..0]
            // C7..C0 in Byte 7
            uint16_t c_fixed = ((uint16_t)(buffer[6] & 0x0F) << 8) | buffer[7];
            float current = c_fixed * 3.0f / 4095.0f;
            
            // Store in status_data array
            // Format: [M1_Vel, M1_Temp, M1_Volt, M1_Curr, M2_Vel, ...]
            int base_idx = motor_index * 4;
            status_data[base_idx + 0] = velocity;
            status_data[base_idx + 1] = (float)temp;
            status_data[base_idx + 2] = voltage;
            status_data[base_idx + 3] = current;
        } else {
          // Flush unknown packet
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

  // Initialize Timer (e.g. 50ms -> 20Hz)
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 2 handles: sub + timer
  RCCHECK(rclc_executor_add_subscription(&executor, &velocity_subscriber, &velocity_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Allocate memory for messages
  // Velocity msg (Subscriber) - Pre-allocate to max 6
  static float velocity_memory[DRIVE_MOTOR_COUNT];
  velocity_msg.data.capacity = DRIVE_MOTOR_COUNT;
  velocity_msg.data.data = velocity_memory;
  velocity_msg.data.size = 0;

  // Status msg (Publisher)
  status_msg.data.capacity = DRIVE_MOTOR_COUNT * 4;
  status_msg.data.data = status_data;
  status_msg.data.size = DRIVE_MOTOR_COUNT * 4;
  
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
