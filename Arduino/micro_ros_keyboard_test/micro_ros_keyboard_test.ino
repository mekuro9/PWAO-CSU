#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>
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

/* ================== IMU =================== */

bool zero = true;
bool calibrate = true;
int zeroTime = 50;
double degToRad = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(11,0x28,&Wire2);

// Global variable gyro and angle for angular velocity and rotation angle 
imu::Vector<3> gyro;
imu::Vector<3> angle;

/* ================= USER INPUT ================ */

// For analog reading 
int linearPin = A0; // forward and Backward motion
int turningPin = A1;   // Turning motion
int centerReferencePin = A2; // Reference when the joystick is at natural position (not used)
int minJoy = 300; 
int maxJoy = 470;
// Digital Potentiometer
int midlinear = 175;
int midturn = 165;
const int shutDownPin = 2;

int action = midturn; //(middle value) decide to increase vel or decrease vel (forward or backward)
int step = 30;

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

float maxSpeed = 40;
float maxAcceleration = 40;

//controller type 
pid_controller_t Ocontroller = typePID;
pid_controller_t Icontroller = typePID;
// PID gains - these need tuning for your specific application
float ikp = 2;
float iki = 0.00005;
float ikd = 10;
// PID OuterLoop
float Okp = 2;
float Oki = 0.005;
float Okd = 1;

// PID Controller for motorPID, gyroSetpoint is the OuterControllerOutput which is rad/s. The measurement is angular velocity and the output is int pwm
PIDController<float, int> motorPID(&gyroSetpoint, &gyroMeasure, &speedControllerOutput, ikp, iki, ikd, Icontroller);
// PID Controller for outerLoop, angleSetpoint is the desired Angle which in deg. The measurement is orientation in deg and the output is desired angular velocity in deg/s
PIDController<float,float> OuterLoop(&angleSetpoint, &angleMeasure, &OuterControllerOutput,Okp,Oki,Okd,Ocontroller);


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

/* =========== UTILITY FOR MOTOR CONTROL ==========================*/

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

/* ================= Micro-ros =============== */

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t teensy_keyboard_; // subscriber for keyboard input 
std_msgs__msg__Int32 keyboard_data; // message type for keyboard data
rclc_executor_t executor_keyboard;

bool user_input = false;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void keyboard_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * keyboard_data = (const std_msgs__msg__Int32 *)msgin;
  int user = keyboard_data->data;  // S = 83, W= 87, A = 65, D = 68

  (user==83) ? (action -= step) : (action += 0);
  (user==87) ? (action += step) : (action += 0);
  (user==65) ? (rotateby -= 30) : (rotateby += 0);
  (user==68) ? (rotateby += 30) : (rotateby += 0);

  user_input = true;
  
}

void setup() {
    
    /* =========================== Input ================================ */
  // Initialize the motor control pins as outputs
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Wire.begin();
  
    /* =========================== IMU ================================ */

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
    }
    else
    {
      while(!bno.isFullyCalibrated()){ imu_calibrate(&bno, &event);}
      
    }

    adafruit_bno055_offsets_t newCalib;
    imu_store_offsets(&bno, &newCalib, &sensor, &bnoID);
  }
  else { 
      bno.setExtCrystalUse(true); 
    }

  if(zero) {
      // set imu status to zeroing
      delay(1000);
    }
    
  bno.setMode(OPERATION_MODE_NDOF);

  delay(500);


    /* ========================= Micro-ros =============================== */


  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //create subscriber for keyboard input
  RCCHECK(rclc_subscription_init_default(
    &teensy_keyboard_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "userinput"))

 // create executor
  
  RCCHECK(rclc_executor_init(&executor_keyboard, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_keyboard, &teensy_keyboard_, &keyboard_data, &keyboard_callback, ON_NEW_DATA));

  
  /* ==================== CONTROLLER  ========================== */

  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  angle = bno.getQuat().toEuler();
  // Initialize the PID controller
  bool check = motorPID.setup();
  Serial.println(check);
  Serial.println("");
  check = OuterLoop.setup();
  Serial.println(check);
  Serial.println("");
  motorPID.setControllerType(Icontroller);
  OuterLoop.setControllerType(Ocontroller);
  // Set the sample time (in milliseconds)
  motorPID.setSampleTime(10);
  OuterLoop.setSampleTime(10);

  // Set output limits for turning speed
  motorPID.setOutputLimits(-maxSpeed, maxSpeed); // speed backward to speed forward
  
  OuterLoop.setOutputLimits(-maxAcceleration, maxAcceleration); // angular acceleration to reach the desired limits deg/s
  

  startTime = millis();
  stop();
  
}//end setup

// previous angle
float prevAngle = 0;
// error of inner loop (velocity control)
float error = 0;
// error of outerloop (angle control)
float errorO = 0;
//time difference
float tau = 0;

void loop() {

    RCCHECK(rclc_executor_spin_some(&executor_keyboard, RCL_MS_TO_NS(10)));

    currentTime = millis();
    tau = (currentTime - startTime);
    if(tau > 10){
        // check if tau is greater than 10ms
        startTime = millis();
        // get the current value of angle (orientation)
        angle = bno.getQuat().toEuler();
        // z axis is the yaw ( x is forward in wheelchair frame and y can be derived using right hand rule)
        float z = angle.x() * degToRad;
        // get the current gyroscope reading
        gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        // Check for serial data (keyboard input for rotation)
        if(user_input == true) {
        
        desiredAngle = wrapAngle(rotateby); //function to wrap the angle
        user_input = false; 
      }

      if(justonce == true){
        int pot1 = midlinear;//129;// 5.924
        int pot2 = midturn;//128;// 5.920

        Wire.beginTransmission(0x2C);
        Wire.write(0x00);  // the other one: 0x00, 0x80
        Wire.write(pot1);
        Wire.endTransmission();
        Wire.beginTransmission(0x2C);
        Wire.write(0x80);  // the other one: 0x00, 0x80
        Wire.write(pot2);
        rotateby = wrapAngle((float)z);
        desiredAngle = wrapAngle((float)z);
        angleMeasure = wrapAngle((float)z);
        angleSetpoint = desiredAngle;
        gyroMeasure = 0.0;//(float)gyro.x();
        
        // Compute the PID controller output
        OuterLoop.compute();
        gyroSetpoint = (float)OuterControllerOutput; //setting gyro setpoint in deg/s
        motorPID.compute();
        justonce = false;
      }

      angleMeasure = wrapAngle((float)z);
      //current angular velocity as input to the innerloop controller
      gyroMeasure = (float)gyro.x();
      // angle setpoint to the outerloop controller which is the desired angle in steps of 10degs
      angleSetpoint = desiredAngle;
      // first compute the outerloop and 
      OuterLoop.compute();
      // error of outer loop
      errorO = OuterLoop.geterror();
      //output of outer loop is desired angular velocity which we give as gyroscope setpoint to the innerloop controller
      gyroSetpoint = (float)OuterControllerOutput;//errorO; 
      // compute the inner loop controller
      motorPID.compute();
      // error of the innerloop controller
      error = motorPID.geterror();
    
      //if((abs(errorO) > 2)){
      // Apply the controller output to the motors to turn
      if (abs(errorO) > 1) {
        // Turn 
        
          turn(speedControllerOutput);
        }else{
        error = 0;
        errorO = 0;
        int pot1 = midlinear;//129;// 5.924
        int pot2 = midturn;//128;// 5.920

        Wire.beginTransmission(0x2C);
        Wire.write(0x00);  // the other one: 0x00, 0x80
        Wire.write(pot1);
        Wire.endTransmission();
        Wire.beginTransmission(0x2C);
        Wire.write(0x80);  // the other one: 0x00, 0x80
        Wire.write(pot2);

        }

        prevAngle = angleMeasure;

    }

    delay(10);


}
