#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// --- Stepper Motor Settings ---
const int optoPin = 21; 
const int pulPin  = 18; 
const int dirPin  = 5;  
AccelStepper stepper(1, pulPin, dirPin); 
const long limitMin = -200;
const long limitMax = 200;

// --- Servo Motor Settings ---
Servo myServo;
const int servoPin  = 19; 
const int tiltMin = 70;  // Minimum tilt angle
const int tiltMax = 135; // Maximum tilt angle


// --- micro-ROS Objects ---
rcl_subscription_t stepper_sub;
rcl_subscription_t servo_sub;
std_msgs__msg__Int32 msg_stepper;
std_msgs__msg__Int32 msg_servo;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// --- Loop Timer Variables ---
unsigned long lastRosSpin = 0;
const int rosSpinInterval = 10; // Check ROS 2 every 10 ms (100 Hz)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

// --- Stepper Callback ---
void stepper_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  long targetPosition = msg->data;
  
  if (targetPosition > limitMax) targetPosition = limitMax;
  if (targetPosition < limitMin) targetPosition = limitMin;
  
  stepper.moveTo(targetPosition);
}

// --- Servo Callback ---
void servo_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int targetAngle = msg->data;

  // Constrain the target angle to your new physical limits
  if (targetAngle > tiltMax) targetAngle = tiltMax;
  if (targetAngle < tiltMin) targetAngle = tiltMin;

  // The .write() command updates a hardware timer instantly and doesn't block the loop
  myServo.write(targetAngle);
}


void setup() {
  set_microros_transports();
  
  // Initialize Stepper Hardware
  pinMode(optoPin, OUTPUT);
  digitalWrite(optoPin, HIGH);
  //pinMode(enaPin, OUTPUT);
  //digitalWrite(enaPin, HIGH); 

  stepper.setPinsInverted(false, true, false); 
  stepper.setMinPulseWidth(100); 
  stepper.setMaxSpeed(350.0);      
  stepper.setAcceleration(350.0);  

  // Initialize Servo Hardware
  // RDS3218 often uses pulse widths between 500us and 2500us for full range
  myServo.setPeriodHertz(50); // Standard 50Hz servo frequency
  myServo.attach(servoPin, 500, 2500); 

  delay(2000); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_motors_node", "", &support));

  // 1. Create Stepper Subscriber
  RCCHECK(rclc_subscription_init_default(
    &stepper_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "camera_pan"));

  // 2. Create Servo Subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "camera_tilt"));

  // 3. Initialize Executor with '2' handles (one for each subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  // 4. Add both subscribers to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &stepper_sub, &msg_stepper, &stepper_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &msg_servo, &servo_callback, ON_NEW_DATA));
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check for new ROS 2 messages without blocking
  if (currentMillis - lastRosSpin >= rosSpinInterval) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)); 
    lastRosSpin = currentMillis;
  }
  
  // Keep the stepper pulsing at maximum speed
  stepper.run(); 
}
