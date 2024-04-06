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
int velocityStepSize = 40; // also max Velocity

/*========= MICRO ROS =================*/

rclc_support_t support;
rclc_allocator_t allocator;
rcl_node_t node;

// Subscriber to speed from sequence controller
rcl_subscription_t speed_subscriber;
std_msgs__msg__Bool speedSetpoint;
rclc_executor_t speedExecutor;

// subscriber to angle from sequence controller
rcl_subscription_t angleDirection_subscriber;
std_msgs__msg__Int32 angleDirection;
rclc_executor_t angleExecutor;

// setup publishers
rcl_publisher_t currentAngle_publisher;
std_msgs__msg__Float32 currentAngle_msg;

rcl_publisher_t angleError_publisher;
std_msgs__msg__Float32 angle_error_msg;

rcl_publisher_t current_velocity_publisher;
std_msgs__msg__Int32 current_velocity_msg;

rcl_publisher_t velocity_error_publisher;
std_msgs__msg__Int32 velocity_error_msg;

// subscriber to Lidar obstacle angle and distance
// TO DO
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

int midlinear = 175;
int midturn = 165;
const int shutDownPin = 2;
int action = midturn; //(middle value) decide to increase vel or decrease vel (forward or backward)


// Motor control pins for two motors

const int u1F = 5; // u1 forward
const int u1B = 6; // u1 backward
const int u2F = 3; //u2 forward
const int u2B = 4;  // u2 backward

/*========== PID CONTROLLER ===========*/

unsigned long startTime;
unsigned long currentTime;

// PID controller setup (Cascaded PID where gyroSetpoint is the output of the OuterController)
float gyroSetpoint, gyroMeasure;
float angleSetpoint, angleMeasure;
float OuterControllerOutput;
int speedControllerOutput;
bool justonce = true;

float rotateby = 0; // amount of degree you want to rotate;

float desiredAngle = 0.0; // Desired turn angle in degrees
float currentAngle = 0.0; // Current angle from IMU
float motorSpeed = 0.0; // Speed correction for motors

float maxSpeed = 200;
float maxAcceleration = 100;

//controller type 
pid_controller_t Ocontroller = typePID;
pid_controller_t Icontroller = typePID;
// PID gains - these need tuning for your specific application
float ikp = 5;
float iki = 0.00005;
float ikd = 1;
// PID OuterLoop
float Okp = 5;
float Oki = 0;
float Okd = 1;

// PID Controller for motorPID, gyroSetpoint is the OuterControllerOutput which is rad/s. The measurement is angular velocity and the output is int pwm
PIDController<float, int> motorPID(&gyroSetpoint, &gyroMeasure, &speedControllerOutput, ikp, iki, ikd, Icontroller);
// PID Controller for outerLoop, angleSetpoint is the desired Angle which in deg. The measurement is orientation in deg and the output is desired angular velocity in deg/s
PIDController<float,float> OuterLoop(&angleSetpoint, &angleMeasure, &OuterControllerOutput,Okp,Oki,Okd,Ocontroller);

// PID Controller for forward motion

int forwardSpeedSetpoint, forwardSpeedMeasure;
int forwardSpeedOutput;
float fkp = 5;
float fki = 0;
float fkd = 1;

PIDController<int,int> forwardPID(&forwardSpeedSetpoint, &forwardSpeedMeasure, &forwardSpeedOutput, fkp, fki, fkd, Icontroller);

int pwm_array[4] = {0,0,0,0}; // u1f, u2f, u1b, u2b


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
void goforward(int speed1, int speed2);
void turnleft(int speed1, int speed2);
void turnright(int speed1, int speed2);
void angle_callback();
void speed_callback();
double wrapAngle(double angle);
//
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
  }else { 
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
    &speed_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "setpoint/speed"))

  RCCHECK(rclc_subscription_init_default(
    &angleDirection_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "setpoint/speed"))

  RCCHECK(rclc_publisher_init(
    &currentAngle_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/currentAngle"))

  RCCHECK(rclc_publisher_init(
    &angleError_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/angleError"))

  RCCHECK(rclc_publisher_init(
    &current_velocity_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "loop/currentSpeed")
  )
  RCCHECK(rclc_publisher_init(
    &velocity_error_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "loop/speedError")
  )
 // create executor
  RCCHECK(rclc_executor_init(&angleExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&angleExecutor, &angleDirection_subscriber, &angleDirection, &angle_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&speedExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&speedExecutor, &speed_subscriber, &speedSetpoint, &speed_callback, ON_NEW_DATA));

  /* ==================== CONTROLLER  ========================== */

  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  angle = bno.getQuat().toEuler();
  // Initialize the PID controller
  bool check = motorPID.setup();

  check = OuterLoop.setup();

  motorPID.setControllerType(Icontroller);
  OuterLoop.setControllerType(Ocontroller);
  
  check = forwardPID.setup();
  // Set the sample time (in milliseconds)
  motorPID.setSampleTime(10);
  OuterLoop.setSampleTime(10);
  forwardPID.setSampleTime(10);
  // Set output limits for turning speed
  motorPID.setOutputLimits(-maxSpeed, maxSpeed); // speed backward to speed forward
  
  OuterLoop.setOutputLimits(-maxAcceleration, maxAcceleration); // angular acceleration to reach the desired limits deg/s
  
  forwardPID.setOutputLimits(0,maxSpeed);
  // Initialize the motor control pins as outputs
  pinMode(u1F, OUTPUT);
  pinMode(u1B, OUTPUT);
  pinMode(u2F, OUTPUT);
  pinMode(u2B, OUTPUT);

  startTime = millis();
  stop();
  
}//end setup


/* END SETUP */

/* =========== LOOP VARIABLES ========*/
// previous angle
float prevAngle = 0;
// error of inner loop (velocity control)
float error = 0;
// error of outerloop (angle control)
float errorO = 0;
//time difference
float tau = 0;
int errorVel = 0;
bool state = true;
int loop_ctr = 0;
int max_speed = 255;
int min_speed = 160;
int pwm_ctr = min_speed;
int mapped_input = 160;

int prevSpeedData = 0;


/* ============ LOOP =================*/
void loop() {

  RCCHECK(rclc_executor_spin_some(&angleExecutor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&speedExecutor, RCL_MS_TO_NS(10)));
 
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  angle = bno.getQuat().toEuler;
  currentAngle = angle.x()*radToDeg;

  if(justonce == true){
        rotateby = wrapAngle((float)currentAngle);
        desiredAngle = wrapAngle((float)currentAngle);
        angleMeasure = wrapAngle((float)currentAngle);
        angleSetpoint = desiredAngle;
        gyroMeasure = 0.0;//(float)gyro.x();
        // Compute the PID controller output
        forwardSpeedMeasure = 0;
        forwardSpeedSetpoint = 0;
        forwardPID.compute();
        OuterLoop.compute();
        gyroSetpoint = (float)OuterControllerOutput; //setting gyro setpoint in deg/s
        motorPID.compute();

        errorO = OuterLoop.geterror();
        error = motorPID.geterror();
        errorVel = forwardPID.geterror();

        justonce = false;
      }

  currentTime = millis();
  tau = (currentTime - startTime);
  if(tau > 10){
    startTime = millis();

    if(speedSetpoint.data){

      forwardSpeedMeasure = prevSpeedData;
      forwardSpeedSetpoint = velocityStepSize;
      forwardPID.compute();
      errorVel = forwardPID.geterror();


    }
    angleDirection.data


        
  }

  current_angle_msg.data = angleMeasure;
  angle_error_msg.data = errorO;
  current_velocity_msg.data = forwardSpeedMeasure;
  velocity_error_msg.data = errorVel;
  
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
  analogWrite(3,0);
  pwm_array[0] = 0;
  analogWrite(4,0);
  pwm_array[1] = 0;
  analogWrite(5,0);
  pwm_array[2] = 0;
  analogWrite(6,0);
  pwm_array[3] = 0;


}

void goforward(int speed1, int speed2){

  analogWrite(u1F,speed1);
  pwm_array[0] = speed1;
  analogWrite(u2F,speed2);
  pwm_array[1] = speed2;
  analogWrite(u1B,0);
  pwm_array[2] = 0;
  analogWrite(u2B,0);
  pwm_array[3] = 0;

}

void turnleft(int speed1, int speed2){
  analogWrite(u1F,0);
  pwm_array[0] = 0;
  analogWrite(u2F,speed2);
  pwm_array[1] = speed2;
  analogWrite(u1B,speed1);
  pwm_array[2] = speed1;
  analogWrite(u2B,0);
  pwm_array[3] = 0;
}

void turnright(int speed1, int speed2){
  analogWrite(u1F,speed1);
  pwm_array[0] = speed1;
  analogWrite(u2F,0);
  pwm_array[1] = 0;
  analogWrite(u1B,0);
  pwm_array[2] = 0;
  analogWrite(u2B,speed2);
  pwm_array[3] = speed2;
}

// Callback functions if needed
void angle_callback(){

}

void speed_callback(){

}