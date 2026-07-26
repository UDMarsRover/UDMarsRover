#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h> // Added core header for custom transports

#include <TinyGPSPlus.h>


#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

//GPS Publishers
//Date
rcl_publisher_t p_year;
rcl_publisher_t p_month;
rcl_publisher_t p_day;
rcl_publisher_t p_value;
rcl_publisher_t p_date_age;

//Time

//Location
rcl_publisher_t p_lat;
rcl_publisher_t p_lng;
rcl_publisher_t p_alt_m;
rcl_publisher_t p_alt_ft;

//Location




//GPS Variables
uint16_t year;
uint8_t month;
uint8_t day;
uint32_t value;
uint32_t date_age;

double lat;
double lng;
double alt_m;
double alt_ft;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


//GPS Object
TinyGPSPlus gps;
// Use Hardware Serial1 on the Arduino Due
#define gpsPort Serial1
//GPS Baud
static const uint32_t GPSBaud = 9600; // Update to 9600 if your GPS requires it


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

//Date
  
    RCSOFTCHECK(rcl_publish(&p_year, &year, NULL));
    year = gps.date.year();

    RCSOFTCHECK(rcl_publish(&p_month, &month, NULL));
    month = gps.date.month();

    RCSOFTCHECK(rcl_publish(&p_day, &day, NULL));
    day = gps.date.day();

    RCSOFTCHECK(rcl_publish(&p_value, &value, NULL));
    value = gps.date.value();

    RCSOFTCHECK(rcl_publish(&p_date_age, &date_age, NULL));
    date_age = gps.date.age();

  //Location

    RCSOFTCHECK(rcl_publish(&p_year, &year, NULL));
    lat = gps.location.lat();

    RCSOFTCHECK(rcl_publish(&p_month, &month, NULL));
    lng = gps.location.lng();

    RCSOFTCHECK(rcl_publish(&p_day, &day, NULL));
    alt_m = gps.altitude.meters();

    RCSOFTCHECK(rcl_publish(&p_value, &value, NULL));
    alt_ft = gps.altitude.feet();

    
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
  // Call the core micro-ROS transport setter instead of the Arduino wrapper
  rmw_uros_set_custom_transport(
    true,
    NULL,
    my_custom_transport_open,
    my_custom_transport_close,
    my_custom_transport_write,
    my_custom_transport_read
  );
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off to indicate waiting
  
  // Wait for the Host PC to physically mount the Native USB device
  while(!SerialUSB) {
    delay(10);
  }

  // NEW: Wait for the micro-ROS agent to actually answer!
  // This prevents the board from crashing into error_loop() on a reset.
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    // Optionally blink the LED quickly while waiting for the agent
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
  
  digitalWrite(LED_PIN, HIGH); // Solid LED means agent is connected!

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "sensors_arduino_due", "", &support));

  // create publisher - gps year
  RCCHECK(rclc_publisher_init_default(
    &p_year,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/date/year"));

  // create publisher - gps month
  RCCHECK(rclc_publisher_init_default(
    &p_month,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/date/month"));

  // create publisher - gps day
  RCCHECK(rclc_publisher_init_default(
    &p_day,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/date/day"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &p_value,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/date/value"));

  // create publisher - date age
  RCCHECK(rclc_publisher_init_default(
    &p_date_age,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/date/age"));



  // create publisher - gps lat
  RCCHECK(rclc_publisher_init_default(
    &p_lat,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/location/lat"));

  // create publisher - gps lng
  RCCHECK(rclc_publisher_init_default(
    &p_lng,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/location/lon"));

  // create publisher - gps alt m
  RCCHECK(rclc_publisher_init_default(
    &p_alt_m,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/altitude/meters"));

  // create publisher - gps alt ft
  RCCHECK(rclc_publisher_init_default(
    &p_alt_ft,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps/altitude/feet"));


  

  // create timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  year = 0;

//GPS Setup code
  gpsPort.begin(GPSBaud);





}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

//GPS encoding
gps.encode(gpsPort.read());


}
