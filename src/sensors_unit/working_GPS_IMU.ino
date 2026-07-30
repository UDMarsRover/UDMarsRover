#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
// #include <TinyGPSPlus.h> // Commented out for indoor spoofing
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 Message Types
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/float64.h>

// --- IMU Definitions ---
BNO08x myIMU;
#define BNO08X_INT  A4
#define BNO08X_RST  A5
#define BNO08X_ADDR 0x4B  

// Global variables to hold the latest Quaternions
double current_quat_i = 0.0;
double current_quat_j = 0.0;
double current_quat_k = 0.0;
double current_quat_real = 1.0; 

// --- GPS Definitions (Spoofed) ---
// TinyGPSPlus gps; 
// #define gpsPort Serial1
// static const uint32_t GPSBaud = 9600; 

#define LED_PIN 13

// --- ROS 2 Publishers and Messages ---
// IMU
rcl_publisher_t p_imu;
sensor_msgs__msg__Imu msg_imu;

// GPS
rcl_publisher_t p_navsat;
rcl_publisher_t p_mph;
rcl_publisher_t p_deg;

sensor_msgs__msg__NavSatFix msg_navsat;
std_msgs__msg__Float64 msg_mph; 
std_msgs__msg__Float64 msg_deg; 

// ROS Core Variables
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_imu; 
rcl_timer_t timer_gps;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Enable Rotation Vector for accurate Quaternions (IMU)
void setReports(void) {
  if (myIMU.enableRotationVector(50) == false) { // 50ms = 20Hz update rate
    // Will attempt to reset later if failed
  }
}

// Timer callback to publish IMU data (runs every 50ms)
void timer_callback_imu(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 1. Sync timestamp with ROS 2
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg_imu.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    msg_imu.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

    // 2. Load the latest Quaternions into the message
    msg_imu.orientation.x = current_quat_i;
    msg_imu.orientation.y = current_quat_j;
    msg_imu.orientation.z = current_quat_k;
    msg_imu.orientation.w = current_quat_real;

    // 3. Publish
    RCSOFTCHECK(rcl_publish(&p_imu, &msg_imu, NULL));
  }
}

// Timer callback to publish SPOOFED GPS data (runs every 1000ms)
void timer_callback_gps(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 1. Sync timestamp with the ROS 2 Agent
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg_navsat.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    msg_navsat.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

    // 2. Spoof a perfect GPS lock in Dayton, OH
    msg_navsat.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
    msg_navsat.latitude = 39.75894;  // Simulated Latitude
    msg_navsat.longitude = -84.19161; // Simulated Longitude
    msg_navsat.altitude = 225.0;      // Simulated Altitude in meters (approx 740 ft)

    // Simulate sitting still
    msg_mph.data = 0.0;
    msg_deg.data = 0.0;

    // 3. Publish the complete messages
    RCSOFTCHECK(rcl_publish(&p_navsat, &msg_navsat, NULL));
    RCSOFTCHECK(rcl_publish(&p_mph, &msg_mph, NULL));
    RCSOFTCHECK(rcl_publish(&p_deg, &msg_deg, NULL));
  }
}

// --- MICRO-ROS CUSTOM SERIAL TRANSPORT FOR DUE NATIVE PORT ---
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
  
  // Hardware Serial initialization
  Wire.begin();
  // gpsPort.begin(GPSBaud); // Commented out for indoor spoofing
  
  while(!SerialUSB) { delay(10); }

  // Initialize the BNO08x
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    error_loop(); 
  }
  delay(500);
  setReports(); 

  // Wait for micro-ROS agent connection
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
  
  // Sync time with the agent for valid timestamps
  rmw_uros_sync_session(1000);
  
  digitalWrite(LED_PIN, HIGH); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Initialize Unified Node
  RCCHECK(rclc_node_init_default(&node, "sensors_arduino_due", "", &support));

  // --- Initialize Publishers ---
  // IMU Publisher
  RCCHECK(rclc_publisher_init_default(
    &p_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));

  // GPS Publishers
  RCCHECK(rclc_publisher_init_default(
    &p_navsat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gps/fix"));
  RCCHECK(rclc_publisher_init_default(
    &p_mph, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/speed/mph"));
  RCCHECK(rclc_publisher_init_default(
    &p_deg, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/course/deg"));

  // --- Pre-fill static fields ---
  // IMU Static Fields
  msg_imu.header.frame_id.data = (char * ) "imu_link";
  msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);
  msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;
  msg_imu.angular_velocity_covariance[0] = -1.0;
  msg_imu.linear_acceleration_covariance[0] = -1.0;

  // GPS Static Fields
  msg_navsat.header.frame_id.data = (char * ) "imu_link";
  msg_navsat.header.frame_id.size = strlen(msg_navsat.header.frame_id.data);
  msg_navsat.header.frame_id.capacity = msg_navsat.header.frame_id.size + 1;
  msg_navsat.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  msg_navsat.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  // --- Initialize Timers ---
  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(50), timer_callback_imu));
  RCCHECK(rclc_timer_init_default(&timer_gps, &support, RCL_MS_TO_NS(1000), timer_callback_gps));

  // --- Initialize Executor ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_gps));
}

void loop() {
  // 1. Process all available NMEA characters from GPS (Bypassed for spoofing)
  // while (gpsPort.available() > 0) {
  //   gps.encode(gpsPort.read());
  // }

  // 2. Process hardware updates for IMU (Still active)
  delay(5); 
  
  if (myIMU.wasReset()) {
    delay(100); 
    setReports();
  }

  if (myIMU.getSensorEvent() == true) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      current_quat_i = (double)myIMU.getQuatI();
      current_quat_j = (double)myIMU.getQuatJ();
      current_quat_k = (double)myIMU.getQuatK();
      current_quat_real = (double)myIMU.getQuatReal();
    }
  }

  // 3. Service the micro-ROS executor to handle publishing both timers
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
