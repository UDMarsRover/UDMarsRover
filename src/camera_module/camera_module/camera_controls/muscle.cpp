#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h> // LIGHTWEIGHT

// Hardware Pins
const int SERVOPIN = 18;
const int PULPIN = 14;
const int DIRPIN = 12;
const int ENAPIN = 27;

Servo myServo;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// micro-ROS Objects
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_subscription_t servo_sub;
rcl_subscription_t stepper_sub;
std_msgs__msg__Int32 servo_msg;
std_msgs__msg__Int32 stepper_msg;

// Callback: Just write the angle directly
void servo_callback(const void * msin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msin;
  myServo.write(msg->data);
}

// Callback: Set speed and direction based on integer
void stepper_callback(const void * msin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msin;
  int speed = msg->data;

  if (abs(speed) < 100) {
    stepper->stopMove();
  } else {
    stepper->setSpeedInHz(abs(speed));
    if (speed > 0) stepper->runForward();
    else stepper->runBackward();
  }
}

void setup() {
  // Start Hardware
  engine.init();
  stepper = engine.stepperConnectToPin(PULPIN);
  if (stepper) {
    stepper->setDirectionPin(DIRPIN);
    stepper->setEnablePin(ENAPIN, false ); // LOW = Enabled
    stepper->setAutoEnable(true);
    stepper->setAcceleration(2000);
  }
  
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVOPIN, 500, 2500);

  // Start micro-ROS
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  // WAIT FOR AGENT (Crucial)
  while (RCL_RET_OK != rmw_uros_ping_agent(100, 1)) { delay(100); }

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rover_muscle_node", "", &support);

  // Subscriptions
  rclc_subscription_init_default(&servo_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_angle");
  rclc_subscription_init_default(&stepper_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "stepper_speed");

  // Executor (2 subscriptions)
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &stepper_sub, &stepper_msg, &stepper_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}