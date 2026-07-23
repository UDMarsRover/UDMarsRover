#include <micro_ros_arduino.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include the Float64 message type for accelerometer data
#include <std_msgs/msg/float64.h>

// ROS 2 Publishers and Messages
rcl_publisher_t p_accel_x;
rcl_publisher_t p_accel_y;
rcl_publisher_t p_accel_z;

std_msgs__msg__Float64 msg_accel_x;
std_msgs__msg__Float64 msg_accel_y;
std_msgs__msg__Float64 msg_accel_z;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// IMU Object and Pin Definitions
BNO08x myIMU;
#define BNO08X_INT  A4
#define BNO08X_RST  A5
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout defaults to 0x4B

#define LED_PIN 13

// Global variables to hold the most recent IMU readings (Changed to double for ROS 2 compatibility)
double current_accel_x = 0.0;
double current_accel_y = 0.0;
double current_accel_z = 0.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Function to enable IMU sensors
void setReports(void) {
  // Explicitly request 50ms update rate
  if (myIMU.enableAccelerometer(50) == false) {
    // If it fails, it will attempt to reset later in the loop
  }
}

// Timer callback to publish the latest data to ROS 2
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    // Assign latest sensor readings to the ROS message structs
    msg_accel_x.data = current_accel_x;
    msg_accel_y.data = current_accel_y;
    msg_accel_z.data = current_accel_z;

    // Publish the messages
    RCSOFTCHECK(rcl_publish(&p_accel_x, &msg_accel_x, NULL));
    RCSOFTCHECK(rcl_publish(&p_accel_y, &msg_accel_y, NULL));
    RCSOFTCHECK(rcl_publish(&p_accel_z, &msg_accel_z, NULL));
  }
}

// --- MICRO-ROS CUSTOM SERIAL TRANSPORT FUNCTIONS FOR DUE NATIVE PORT ---
bool my_custom_transport_open(struct uxrCustomTransport * transport) {
  SerialUSB.begin(115200);
  return true;
}

bool my_custom_transport_close(struct uxrCustomTransport * transport) {
  SerialUSB.end();
  return true;
}

size_t my_custom_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
  return SerialUSB.write(buf, len);
}

size_t my_custom_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
  SerialUSB.setTimeout(timeout);
  return SerialUSB.readBytes((char *)buf, len);
}
// ------------------------------------------------------------------------

void setup() {
  rmw_uros_set_custom_transport(
    true, NULL,
    my_custom_transport_open,
    my_custom_transport_close,
    my_custom_transport_write,
    my_custom_transport_read
  );
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Wire.begin();
  
  while(!SerialUSB) { delay(10); }

  // Initialize the IMU
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    error_loop(); 
  }
  
  // Wait a half second for the IMU processor to fully boot before sending commands
  delay(500);
  
  setReports(); // Enable accelerometer

  // Wait for micro-ROS agent connection
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
  
  digitalWrite(LED_PIN, HIGH); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Initialize Node
  RCCHECK(rclc_node_init_default(&node, "imu_arduino_due", "", &support));

  // Initialize Publishers
  RCCHECK(rclc_publisher_init_default(
    &p_accel_x, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "imu/accel/x"));
    
  RCCHECK(rclc_publisher_init_default(
    &p_accel_y, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "imu/accel/y"));

  RCCHECK(rclc_publisher_init_default(
    &p_accel_z, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "imu/accel/z"));

  // Set Timer to 10Hz (100 ms). 
  const unsigned int timer_timeout = 100; 
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Initialize Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // Give the I2C bus a tiny pause so the BNO08x doesn't lock up
  delay(5); 

  // If the sensor was reset, re-enable the desired reports
  if (myIMU.wasReset()) {
    delay(100); // Give it a moment after reset
    setReports();
  }

  // Continuously check for new hardware data
  if (myIMU.getSensorEvent() == true) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
      // Update the global variables (casting float to double)
      current_accel_x = (double)myIMU.getAccelX();
      current_accel_y = (double)myIMU.getAccelY();
      current_accel_z = (double)myIMU.getAccelZ();
    }
  }

  // Service the micro-ROS executor to handle publishing
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
