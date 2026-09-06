#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <builtin_interfaces/msg/time.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <CANSAME5x.h>
#include <Wire.h>
#include <algorithm> // For std::min and std::max

// Fallback CAN Transceiver Pin Definitions
#ifndef PIN_CAN_STANDBY
  #define PIN_CAN_STANDBY 4
#endif
#ifndef PIN_CAN_BOOSTEN
  #define PIN_CAN_BOOSTEN 2
#endif

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

// Max wheel RPM safety threshold
const float MAX_WHEEL_RPM = 300.0f; 

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

// Static Memory Buffers for Micro-ROS Messages
static float velocity_memory[8];

// IMU (MPU6050) Publisher
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
float imu_data[6];

bool time_synced = false;
const uint8_t MPU_ADDR = 0x68;

// Status layout: 6 motors * 6 values
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
  
  memset(control_data, 0, CONTROL_SIZE);
  memcpy(control_data, &setpoint, sizeof(setpoint));

  CAN.beginExtendedPacket(control_id);
  CAN.write(control_data, CONTROL_SIZE);
  CAN.endPacket();
}

void send_config_parameter(const uint32_t device_id, uint8_t param_id, float value) {
  uint32_t config_id = 0x2051C00 + device_id;
  uint8_t data[8] = {0};
  
  data[0] = param_id;
  data[1] = 0;
  memcpy(&data[2], &value, 4);
  data[6] = 0;
  
  CAN.beginExtendedPacket(config_id);
  CAN.write(data, 8);
  CAN.endPacket();
}

void idle_mode_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  float mode_val = msg->data ? 1.0f : 0.0f; 
  const uint8_t kIdleMode = 10;
  
  for(int i=1; i<=DRIVE_MOTOR_COUNT; i++) {
     send_config_parameter(i, kIdleMode, mode_val);
     delay(2); 
  }
}

// Subscription Callback (Updated for Closed-Loop Speed Mode)
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  uint8_t count = (msg->data.size < DRIVE_MOTOR_COUNT) ? msg->data.size : DRIVE_MOTOR_COUNT;

  for (int i = 0; i < count; i++) {
      target_velocities[i] = msg->data.data[i];

      // Clamp target velocity to max speed safety limits (-300 to +300 RPM)
      float target_rpm = std::max(-MAX_WHEEL_RPM, std::min(MAX_WHEEL_RPM, target_velocities[i]));

      // Send RPM setpoint directly using Speed_Set (Closed-Loop Velocity Mode)
      send_control_frame(i + 1, Speed_Set, target_rpm);
  }
}

static inline builtin_interfaces__msg__Time ros_time_now()
{
  builtin_interfaces__msg__Time t;
  int64_t ns = rmw_uros_epoch_nanos();
  t.sec = (int32_t)(ns / 1000000000LL);
  t.nanosec = (uint32_t)(ns % 1000000000LL);
  return t;
}

// Timer Callback (Publish Status)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    status_msg.data.data = status_data;
    status_msg.data.size = DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR;
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));

    read_mpu6050();
    imu_msg.linear_acceleration.x = (double)imu_data[0];
    imu_msg.linear_acceleration.y = (double)imu_data[1];
    imu_msg.linear_acceleration.z = (double)imu_data[2];

    imu_msg.angular_velocity.x = (double)imu_data[3];
    imu_msg.angular_velocity.y = (double)imu_data[4];
    imu_msg.angular_velocity.z = (double)imu_data[5];

    imu_msg.header.stamp = ros_time_now();
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void write_mpu_register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void read_mpu6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  if (Wire.available() >= 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t t  = (Wire.read() << 8) | Wire.read();
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    imu_data[0] = (float)ax / 16384.0f;
    imu_data[1] = (float)ay / 16384.0f;
    imu_data[2] = (float)az / 16384.0f;
    imu_data[3] = (float)gx / 131.0f;
    imu_data[4] = (float)gy / 131.0f;
    imu_data[5] = (float)gz / 131.0f;
  }
}

void check_for_can_packets() {
    while (CAN.parsePacket()) {
        uint32_t packet_id = CAN.packetId();
        int motor_index = -1;
        
        if (packet_id > status_0 && packet_id <= status_0 + DRIVE_MOTOR_COUNT) {
            motor_index = packet_id - status_0 - 1;
            if (CAN.available() >= 8) {
               uint8_t buffer[8];
               for (int i=0; i<8; i++) buffer[i] = CAN.read();
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
               float velocity = 0.0f;
               memcpy(&velocity, buffer, 4);
               uint8_t temp = buffer[4];
               uint16_t v_raw = buffer[5] | ((buffer[6] & 0x0F) << 8);
               float voltage = v_raw / 128.0f; 
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
               float position = 0.0f;
               memcpy(&position, buffer, 4);
               int base_idx = motor_index * VALUES_PER_MOTOR;
               status_data[base_idx + 1] = position;
            }
        }
        else {
          while(CAN.available()) CAN.read();
        }
    }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); 
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);
  
  CAN.begin(1000000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  while (rc != RCL_RET_OK) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(250);
      rc = rclc_support_init(&support, 0, NULL, &allocator);
  }
  
  digitalWrite(LED_BUILTIN, HIGH); 

  RCCHECK(rclc_node_init_default(&node, "rover_drive_node", "", &support));

  // Initialize Publishers
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_status"));

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_raw"));

  // Initialize Subscribers
  RCCHECK(rclc_subscription_init_default(
    &velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_velocities_cmd"));

  RCCHECK(rclc_subscription_init_default(
    &idle_mode_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_idle_mode"));

  // Initialize Timer
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Allocate message buffers before attaching to executor
  velocity_msg.data.capacity = 8;
  velocity_msg.data.data = velocity_memory;
  velocity_msg.data.size = 0;

  status_msg.data.capacity = DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR;
  status_msg.data.data = status_data;
  status_msg.data.size = DRIVE_MOTOR_COUNT * VALUES_PER_MOTOR;
  status_msg.layout.dim.capacity = 0;
  status_msg.layout.dim.size = 0;

  // Initialize Executor AFTER Memory Allocation
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &velocity_subscriber, &velocity_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &idle_mode_subscriber, &idle_mode_msg, &idle_mode_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  time_synced = (rmw_uros_sync_session(2000) == RMW_RET_OK);

  Wire.begin();
  delay(10);
  write_mpu_register(0x6B, 0x00);
  delay(10);

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = -1.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
}

void loop() {
  check_for_can_packets();
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 50) {
     CAN.beginExtendedPacket(0x2052C80);
     uint8_t hb_data[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
     CAN.write(hb_data, 8);
     CAN.endPacket();
     last_heartbeat = millis();
  }
}