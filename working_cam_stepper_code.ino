#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <AccelStepper.h>

// --- Motor Settings ---
const int optoPin = 21; 
const int pulPin  = 18; 
const int dirPin  = 5;  
const int enaPin  = 19; 

AccelStepper stepper(1, pulPin, dirPin); 

const long limitMin = -200;
const long limitMax = 200;

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// --- Loop Timer Variables ---
unsigned long lastRosSpin = 0;
const int rosSpinInterval = 10; // Check ROS 2 every 10 milliseconds (100 Hz)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  long targetPosition = msg->data;
  
  if (targetPosition > limitMax) targetPosition = limitMax;
  if (targetPosition < limitMin) targetPosition = limitMin;
  
  stepper.moveTo(targetPosition);
}

void setup() {
  set_microros_transports();
  
  pinMode(optoPin, OUTPUT);
  digitalWrite(optoPin, HIGH);
  pinMode(enaPin, OUTPUT);
  digitalWrite(enaPin, HIGH); 

  stepper.setPinsInverted(false, true, false); 
  stepper.setMinPulseWidth(100); 
  
  // --- INCREASED SPEED SETTINGS ---
  stepper.setMaxSpeed(2000.0);      
  stepper.setAcceleration(1000.0);  

  delay(2000); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "stepper_motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "target_position"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // --- NON-BLOCKING ROS 2 CHECK ---
  // This allows stepper.run() to fire thousands of times per second undisturbed
  unsigned long currentMillis = millis();
  if (currentMillis - lastRosSpin >= rosSpinInterval) {
    // Timeout is set to 0 so it returns immediately if no message is waiting
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)); 
    lastRosSpin = currentMillis;
  }
  
  // The motor is now the absolute priority of the ESP32 processor
  stepper.run(); 
}
