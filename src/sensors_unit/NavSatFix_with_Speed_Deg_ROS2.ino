#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <TinyGPSPlus.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include the standard NavSatFix message type
#include <sensor_msgs/msg/nav_sat_fix.h>

// Include the Float64 message type for speed and direction
#include <std_msgs/msg/float64.h>

// Publisher and Message definitions
rcl_publisher_t p_navsat;
sensor_msgs__msg__NavSatFix msg_navsat;
double msg_mph;
double msg_deg;

//Speed and Direction
rcl_publisher_t p_mph;
rcl_publisher_t p_deg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// GPS Object
TinyGPSPlus gps;
#define gpsPort Serial1
static const uint32_t GPSBaud = 9600; 

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    // 1. Sync timestamp with the ROS 2 Agent
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg_navsat.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    msg_navsat.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

    // 2. Check for a valid GPS lock and populate data
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

    msg_mph = gps.speed.mph();
    msg_deg = gps.course.deg();

    // 3. Publish the complete message
    RCSOFTCHECK(rcl_publish(&p_navsat, &msg_navsat, NULL));
    RCSOFTCHECK(rcl_publish(&p_mph, &msg_mph, NULL));
    RCSOFTCHECK(rcl_publish(&p_deg, &msg_deg, NULL));
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
  
  while(!SerialUSB) { delay(10); }

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
  
  // Sync time with the agent so our ROS headers have valid timestamps
  rmw_uros_sync_session(1000);
  
  digitalWrite(LED_PIN, HIGH); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "sensors_arduino_due", "", &support));

  // Initialize NavSatFix Publisher
  RCCHECK(rclc_publisher_init_default(
    &p_navsat, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), 
    "gps/fix"));

  //Publisher for speed
  RCCHECK(rclc_publisher_init_default(
    &p_mph, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/speed/mph"));
    
  //Publisher for direction
  RCCHECK(rclc_publisher_init_default(
    &p_deg, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "gps/course/deg"));


  // Pre-fill static fields in the NavSatFix message
  msg_navsat.header.frame_id.data = (char * ) "gps_link";
  msg_navsat.header.frame_id.size = strlen(msg_navsat.header.frame_id.data);
  msg_navsat.header.frame_id.capacity = msg_navsat.header.frame_id.size + 1;
  msg_navsat.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  msg_navsat.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  const unsigned int timer_timeout = 1000; // Publish at 1Hz
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  gpsPort.begin(GPSBaud);
}

void loop() {
  // Process all available NMEA characters 
  while (gpsPort.available() > 0) {
    gps.encode(gpsPort.read());
  }
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
