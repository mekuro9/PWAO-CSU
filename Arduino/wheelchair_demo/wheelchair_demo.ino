#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <vector>
#include <math.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "PIDController.hpp"
#include <ctime>

#include "imu_util.h"
/* =========== STEP SIZE ==============*/

int rotationStepSize = 10;

/*========= MICRO ROS =================*/

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Subscriber
rcl_subscription_t state_subscriber;
std_msgs__msg__Int32 state_subscriber_msg;
rclc_executor_t state_subscriber_executor;

rcl_subscription_t angleSetpoint_subscriber;
std_msgs__msg__Int32 angleSetpoint_msg;
rclc_executor_t angleExecutor;

// setup publishers
rcl_publisher_t currentAngle_publisher;
std_msgs__msg__Float32 currentAngle_msg;
rclc_executor_t current_angle_msg_executor;

rcl_publisher_t angleError_publisher;
std_msgs__msg__Float32 angle_error_msg;
rclc_executor_t angle_error_msg_executor;

rcl_publisher_t relative_angle_publisher;
std_msgs__msg__Float32 relative_angle_msg;
rclc_executor_t relative_angle_executor;

rcl_publisher_t current_velocity_publisher;
std_msgs__msg__Float32 current_velocity_msg;
rclc_executor_t current_velocity_msg_executor;

rcl_timer_t timer;
// subscriber to Lidar obstacle angle and distance
// TO DO
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
/* ========= STATE of the Car =========*/
enum class WheelchairState{
    Stopped,
    MovingForward,
    Turning
};

WheelchairState desired_motion_state ;
/*============= IMU ===================*/

bool zero = true;
bool calibrate = true;
double radToDeg = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(11,0x28,&Wire2);

// Global variable gyro and angle for angular velocity and rotation angle 
imu::Vector<3> gyro;
imu::Vector<3> angle;

/* =========== JOYSTICK ===============*/

// For analog reading 
int linearPin = A0; // forward and Backward motion
int turningPin = A1;   // Turning motion
int centerReferencePin = A2; // Reference when the joystick is at natural position (not used)
int minJoy = 300; 
int maxJoy = 470;

/*=======DIGITAL POTENTIOMETER ========*/

int midlinear = 168;
int midturn = 165;
const int shutDownPin = 2;
int action = midlinear; //(middle value) decide to increase vel or decrease vel (forward or backward)


/*========== PID CONTROLLER ===========*/

unsigned long startTime;
unsigned long currentTime;

bool justonce = true;

float rotateby = 0; // amount of degree you want to rotate;

float desiredAngle = 0.0; // Desired turn angle in degrees
float currentAngle = 0.0; // Current angle from IMU
float motorSpeed = 0.0; // Speed correction for motors

float max_speed = 60; // 60 in case of wheelchair (profile 1 or 4)
// PID controller setup (Cascaded PID where motor speed (pwm signal) is the output of the OuterController(Angle controller))
float angleSetpoint, angleMeasure;
int ControllerOutput;

//controller type 
pid_controller_t controllertype = typePID;
// PID OuterLoop
float kp_angle = 5;
float ki_angle = 0;
float kd_angle = 1;

// PID Controller for outerLoop, angleSetpoint is the desired Angle which in deg. The measurement is orientation in deg and the output is desired motor Controller Speed
PIDController<float,int> anglePID(&angleSetpoint, &angleMeasure, &ControllerOutput,kp_angle,ki_angle,kd_angle,controllertype);


/*========= MICRO ROS FUNCTIONS ======*/

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
// 
// Forward declarations of functions
void stop();
void goforward(int speed);
void turn(int speed);
void angle_callback(const void * msgin);
void state_callback(const void * msgin);
double wrapAngle(double angle);
void updateState();
//


/*=============  SETUP ===============*/
void setup() {
    
  // Initialize the motor control pins as outputs
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Wire.begin();
  
  //IMU
  if(!bno.begin()){
    while (1); 
  }

  if(calibrate) {
    long bnoID;
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;
    bool foundCalib = imu_calibrate_check(&sensor, &bno, &bnoID);
    if(!foundCalib){
      //Send sensor status as not calibrated TO DO ************* HOW ??
      delay(500);
    }
    else{   
      // retrieve calibration data
      imu_retrieve_offsets(&calibrationData, &bno);
    }
    delay(1000);
    //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);
    sensors_event_t event;
    if(foundCalib){
      // set sensor status as calibrated
      while(!bno.isFullyCalibrated()){ imu_calibrate(&bno, &event); }
    }else{
      while(!bno.isFullyCalibrated()){ imu_calibrate(&bno, &event);}
    }
    adafruit_bno055_offsets_t newCalib;
    imu_store_offsets(&bno, &newCalib, &sensor, &bnoID);
  } else { 
      bno.setExtCrystalUse(true); 
  }

  if(zero) {
      // set imu status to zeroing
      delay(1000);
  }
    
  bno.setMode(OPERATION_MODE_NDOF);
  delay(500); // end imu setup

  // Mirco ROS
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  delay(2000);
  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_teensy", "", &support));
  //create subscriber for webinput/sequence controller
  RCCHECK(rclc_subscription_init_default(
    &state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "setpoint/state"));

  RCCHECK(rclc_subscription_init_default(
    &angleSetpoint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "setpoint/angle"));

  RCCHECK(rclc_publisher_init_default(
    &currentAngle_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/currentAngle"));

  RCCHECK(rclc_publisher_init_default(
    &angleError_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/angleError"));

  RCCHECK(rclc_publisher_init_default(
    &current_velocity_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/currentSpeed"));

  RCCHECK(rclc_publisher_init_default(
    &relative_angle_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/relativeAngle"));
  
 // create executor
  RCCHECK(rclc_executor_init(&current_angle_msg_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&current_velocity_msg_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&relative_angle_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&angle_error_msg_executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_init(&angleExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&angleExecutor, &angleSetpoint_subscriber, &angleSetpoint_msg, &angle_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&state_subscriber_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&state_subscriber_executor, &state_subscriber, &state_subscriber_msg, &state_callback, ON_NEW_DATA));

  /* ==================== CONTROLLER  ========================== */

  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  angle = bno.getQuat().toEuler();
  // Initialize the PID controller

  bool check = anglePID.setup();

  anglePID.setControllerType(controllertype);
  
  // Set the sample time (in milliseconds)
  anglePID.setSampleTime(10);
  // Set output limits for turning speed
  anglePID.setOutputLimits(-max_speed, max_speed);
  
  // Initialize the motor control pins as outputs
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Wire.begin();

  currentAngle_msg.data = 0;
  current_velocity_msg.data = 0;

  stop();

  startTime = millis();
  
  
}//end setup


/* END SETUP */

/* =========== LOOP VARIABLES ========*/
// car speed
int wheelchair_speed = 0;
// previous angle
float prevAngle = 0;
// error of anglePID (angle control)
float error_angle = 0;
//time difference
float tau = 0;

bool state = true;
int loop_ctr = 0;
int min_speed = 160;
int pwm_ctr = min_speed;
int mapped_input = 160;

int motorpwm1 = 0;
int motorpwm2 = 0;

float initial_angle;
/* ============ LOOP =================*/
void loop() {

  RCCHECK(rclc_executor_spin_some(&angleExecutor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&state_subscriber_executor, RCL_MS_TO_NS(10)));
 
  // Loop spin rate
  currentTime = millis();
  tau = (currentTime - startTime);
  if(tau > 10){
    // Loop spin rate
    startTime = millis();

    // Update current orientation (from IMU)
    angle = bno.getQuat().toEuler();
    currentAngle = angle.x()*radToDeg;

    if(justonce == true){
      
      stop();
      initial_angle = currentAngle;
      angleMeasure = currentAngle;
      angleSetpoint = currentAngle;
      // Compute the PID controller output
      anglePID.compute();

      justonce = false;
    }
    // Update motion state (based on WebGUI)
    updateState();

    switch (desired_motion_state)
    {
      case WheelchairState::Stopped:
        stop();
        motorpwm1 = midlinear;
        motorpwm2 = midturn;
        angleSetpoint = currentAngle;
        break;

      case WheelchairState::MovingForward:
        angleSetpoint = currentAngle;
        motorpwm1 = max_speed;
        motorpwm2 = midturn;
        goforward(motorpwm1);
        break;

      case WheelchairState::Turning:
         // this is to publish for moving forward only
        angleSetpoint = initial_angle + angleSetpoint_msg.data;
        angleMeasure = currentAngle;
        anglePID.compute();
        error_angle = anglePID.geterror();
        if(abs(error_angle) > 1){
          // Turn right
          //map(value, fromLow, fromHigh, toLow, toHigh)
          motorpwm1 = midlinear;
          motorpwm2 = ControllerOutput;
          turn(motorpwm2);
        }else{
            motorpwm1 = midlinear;
            motorpwm2 = midturn;
            stop();
        }
        break;

      default:
        motorpwm1 = midlinear;
        motorpwm2 = midturn;
        stop();
        break;
    }
    
  }

  relative_angle_msg.data = currentAngle - initial_angle;
  angle_error_msg.data = error_angle;
  current_velocity_msg.data = motorpwm1;
  currentAngle_msg.data = currentAngle;

  // Publish data
  RCSOFTCHECK(rcl_publish(&currentAngle_publisher, &currentAngle_msg, NULL));
  RCSOFTCHECK(rcl_publish(&current_velocity_publisher, &current_velocity_msg, NULL));
  RCSOFTCHECK(rcl_publish(&relative_angle_publisher, &relative_angle_msg, NULL));
  RCSOFTCHECK(rcl_publish(&angleError_publisher, &angle_error_msg, NULL));

  delay(BNO055_SAMPLERATE_DELAY_MS);


}// end loop

/*====== UTILITY FUNCTIONS ============*/

/* LIMIT VALUES FUNCTION */

int limitValue(int value){
  if (value > maxJoy){
    value = maxJoy;
  }

  else if (value < minJoy){
    value = minJoy; 
  }
  return value;
}

double wrapAngle(double angle){
  // Wrap the angle rotation by +- 180 deg
  if(angle > 180){
    angle -= 360;
  }
  else if(angle < -180){
    angle += 360 ;
  }
  else{
    angle += 0 ;
  }  
 return angle;
}
/* MOTOR CONTROL FUNCTION */
void stop(){

  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn);
  Wire.endTransmission();


}

void goforward(int speed){
  
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear + speed);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn);
  Wire.endTransmission();

}

void turn(int speed){
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn-speed);
  Wire.endTransmission();
}

void updateState(){
  if(state_subscriber_msg.data == 0){
    desired_motion_state = WheelchairState::Stopped;
  }else if(state_subscriber_msg.data == 1){
    desired_motion_state = WheelchairState::MovingForward;
  }else if(state_subscriber_msg.data == 2){
    desired_motion_state = WheelchairState::Turning;
  }else{
    desired_motion_state = WheelchairState::Stopped;
  }

}

// Callback functions if needed
void angle_callback(const void * msgin){
  
}

void state_callback(const void * msgin){
  

}