#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <TinyGPSPlus.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 Message Types
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <geometry_msgs/msg/vector3.h> 
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32.h>

// --- IMU Definitions ---
BNO08x myIMU;
#define BNO08X_INT  A4
#define BNO08X_RST  A5
#define BNO08X_ADDR 0x4B  

// Global variables to hold the latest IMU Data
double current_quat_i = 0.0, current_quat_j = 0.0, current_quat_k = 0.0, current_quat_real = 1.0;
double current_accel_x = 0.0, current_accel_y = 0.0, current_accel_z = 0.0;
double current_gyro_x = 0.0, current_gyro_y = 0.0, current_gyro_z = 0.0;
double current_mag_x = 0.0, current_mag_y = 0.0, current_mag_z = 0.0;
double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;

// --- GPS Definitions ---
TinyGPSPlus gps;
#define gpsPort Serial1
static const uint32_t GPSBaud = 9600; 

#define LED_PIN 13

// --- ROS 2 Publishers and Messages ---
// IMU Publishers
rcl_publisher_t p_imu;
rcl_publisher_t p_mag;
rcl_publisher_t p_euler;

sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;
geometry_msgs__msg__Vector3 msg_euler;

// GPS Publishers
rcl_publisher_t p_navsat;
rcl_publisher_t p_mph;
rcl_publisher_t p_deg;
rcl_publisher_t p_sats;
rcl_publisher_t p_hdop;

sensor_msgs__msg__NavSatFix msg_navsat;
std_msgs__msg__Float64 msg_mph;
std_msgs__msg__Float64 msg_deg;
std_msgs__msg__Int32 msg_sats;
std_msgs__msg__Float64 msg_hdop;

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

// Enable all desired IMU sensors
void setReports(void) {
  myIMU.enableRotationVector(50); 
  myIMU.enableAccelerometer(50);
  myIMU.enableGyro(50);
  myIMU.enableMagnetometer(50);
}

// Timer callback to publish IMU & Mag data (runs every 50ms)
void timer_callback_imu(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int64_t time_ns = rmw_uros_epoch_nanos();
    int32_t stamp_sec = (int32_t)(time_ns / 1000000000);
    uint32_t stamp_nanosec = (uint32_t)(time_ns % 1000000000);

    msg_imu.header.stamp.sec = stamp_sec;
    msg_imu.header.stamp.nanosec = stamp_nanosec;
    msg_mag.header.stamp.sec = stamp_sec;
    msg_mag.header.stamp.nanosec = stamp_nanosec;

    msg_imu.orientation.x = current_quat_i;
    msg_imu.orientation.y = current_quat_j;
    msg_imu.orientation.z = current_quat_k;
    msg_imu.orientation.w = current_quat_real;

    msg_imu.linear_acceleration.x = current_accel_x;
    msg_imu.linear_acceleration.y = current_accel_y;
    msg_imu.linear_acceleration.z = current_accel_z;

    msg_imu.angular_velocity.x = current_gyro_x;
    msg_imu.angular_velocity.y = current_gyro_y;
    msg_imu.angular_velocity.z = current_gyro_z;

    msg_mag.magnetic_field.x = current_mag_x;
    msg_mag.magnetic_field.y = current_mag_y;
    msg_mag.magnetic_field.z = current_mag_z;

    msg_euler.x = current_roll;
    msg_euler.y = current_pitch;
    msg_euler.z = current_yaw;

    RCSOFTCHECK(rcl_publish(&p_imu, &msg_imu, NULL));
    RCSOFTCHECK(rcl_publish(&p_mag, &msg_mag, NULL));
    RCSOFTCHECK(rcl_publish(&p_euler, &msg_euler, NULL));
  }
}

// Timer callback to publish real GPS data (runs every 1000ms)
void timer_callback_gps(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg_navsat.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    msg_navsat.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

    if (gps.location.isValid()) {
      msg_navsat.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
      msg_navsat.latitude = gps.location.lat();
      msg_navsat.longitude = gps.location.lng();
      msg_navsat.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    } else {
      msg_navsat.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
      msg_navsat.latitude = 0.0;
      msg_navsat.longitude = 0.0;
      msg_navsat.altitude = 0.0;
    }

    // Populate Extra GPS Data
    msg_mph.data = gps.speed.isValid() ? gps.speed.mph() : 0.0;
    msg_deg.data = gps.course.isValid() ? gps.course.deg() : 0.0;
    msg_sats.data = gps.satellites.isValid() ? gps.satellites.value() : 0;
    msg_hdop.data = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;

    RCSOFTCHECK(rcl_publish(&p_navsat, &msg_navsat, NULL));
    RCSOFTCHECK(rcl_publish(&p_mph, &msg_mph, NULL));
    RCSOFTCHECK(rcl_publish(&p_deg, &msg_deg, NULL));
    RCSOFTCHECK(rcl_publish(&p_sats, &msg_sats, NULL));
    RCSOFTCHECK(rcl_publish(&p_hdop, &msg_hdop, NULL));
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
  gpsPort.begin(GPSBaud);
  
  while(!SerialUSB) { delay(10); }

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    error_loop(); 
  }
  delay(500);
  setReports(); 

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
  
  rmw_uros_sync_session(1000);
  
  digitalWrite(LED_PIN, HIGH); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  RCCHECK(rclc_node_init_default(&node, "sensors_arduino_due", "", &support));

  // --- Initialize Publishers ---
  RCCHECK(rclc_publisher_init_default(
    &p_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  RCCHECK(rclc_publisher_init_default(
    &p_mag, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "imu/mag"));
  RCCHECK(rclc_publisher_init_default(
    &p_euler, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "imu/euler"));

  RCCHECK(rclc_publisher_init_default(
    &p_navsat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gps/fix"));
  RCCHECK(rclc_publisher_init_default(
    &p_mph, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/speed/mph"));
  RCCHECK(rclc_publisher_init_default(
    &p_deg, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/course/deg"));
  RCCHECK(rclc_publisher_init_default(
    &p_sats, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "gps/satellites"));
  RCCHECK(rclc_publisher_init_default(
    &p_hdop, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/hdop"));

  // --- Pre-fill static fields ---
  msg_imu.header.frame_id.data = (char * ) "imu_link";
  msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);
  msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;
  msg_imu.angular_velocity_covariance[0] = -1.0;     
  msg_imu.linear_acceleration_covariance[0] = -1.0; 

  msg_mag.header.frame_id.data = (char * ) "imu_link";
  msg_mag.header.frame_id.size = strlen(msg_mag.header.frame_id.data);
  msg_mag.header.frame_id.capacity = msg_mag.header.frame_id.size + 1;
  msg_mag.magnetic_field_covariance[0] = -1.0; 

  msg_navsat.header.frame_id.data = (char * ) "gps_link";
  msg_navsat.header.frame_id.size = strlen(msg_navsat.header.frame_id.data);
  msg_navsat.header.frame_id.capacity = msg_navsat.header.frame_id.size + 1;
  msg_navsat.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  msg_navsat.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  // --- Initialize Timers & Executor ---
  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(50), timer_callback_imu));
  RCCHECK(rclc_timer_init_default(&timer_gps, &support, RCL_MS_TO_NS(1000), timer_callback_gps));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_gps));
}

void loop() {
  while (gpsPort.available() > 0) {
    gps.encode(gpsPort.read());
  }

  delay(5); 
  
  if (myIMU.wasReset()) {
    delay(100); 
    setReports();
  }

  if (myIMU.getSensorEvent() == true) {
    uint8_t eventID = myIMU.getSensorEventID();

    if (eventID == SENSOR_REPORTID_ROTATION_VECTOR) {
      current_quat_i = (double)myIMU.getQuatI();
      current_quat_j = (double)myIMU.getQuatJ();
      current_quat_k = (double)myIMU.getQuatK();
      current_quat_real = (double)myIMU.getQuatReal();

      current_roll = (double)myIMU.getRoll() * (180.0 / PI);
      current_pitch = (double)myIMU.getPitch() * (180.0 / PI);
      current_yaw = (double)myIMU.getYaw() * (180.0 / PI);
    } 
    else if (eventID == SENSOR_REPORTID_ACCELEROMETER) {
      current_accel_x = (double)myIMU.getAccelX();
      current_accel_y = (double)myIMU.getAccelY();
      current_accel_z = (double)myIMU.getAccelZ();
    }
    else if (eventID == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      current_gyro_x = (double)myIMU.getGyroX();
      current_gyro_y = (double)myIMU.getGyroY();
      current_gyro_z = (double)myIMU.getGyroZ();
    }
    else if (eventID == SENSOR_REPORTID_MAGNETIC_FIELD) {
      current_mag_x = (double)myIMU.getMagX();
      current_mag_y = (double)myIMU.getMagY();
      current_mag_z = (double)myIMU.getMagZ();
    }
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
