#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>
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

//obstacle detection 
rcl_subscription_t obstacle_subscriber;
geometry_msgs__msg__Vector3 obstacle_msg;
rclc_executor_t obstacle_executor;

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

//rcl_publisher_t current_velocity_publisher;
//std_msgs__msg__Float32 current_velocity_msg;
//rclc_executor_t current_velocity_msg_executor;

rcl_publisher_t reset_flag_publisher;
std_msgs__msg__Bool reset_flag_msg;
rclc_executor_t reset_flag_executor;

rcl_publisher_t current_state_publisher;
std_msgs__msg__Int32 current_state_msg;
rclc_executor_t current_state_executor;

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


/* =========== JOYSTICK ===============*/

int minJoy = 300; 
int maxJoy = 470;

/*=======DIGITAL POTENTIOMETER ========*/

int midlinear = 168;// 168
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

float max_speed = 80; // 60 in case of wheelchair (profile 1 or 4)
// PID controller setup (Cascaded PID where motor speed (pwm signal) is the output of the OuterController(Angle controller))
float angleSetpoint, angleMeasure;
int ControllerOutput;

//controller type 
pid_controller_t controllertype = typePID;
// PID OuterLoop
float kp_angle = 6;
float ki_angle = 0.0005;
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
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
      // Publish data
  //RCSOFTCHECK(rcl_publish(&currentAngle_publisher, &currentAngle_msg, NULL));
  //RCSOFTCHECK(rcl_publish(&current_velocity_publisher, &current_velocity_msg, NULL));
  RCSOFTCHECK(rcl_publish(&relative_angle_publisher, &relative_angle_msg, NULL));
  RCSOFTCHECK(rcl_publish(&angleError_publisher, &angle_error_msg, NULL));
  RCSOFTCHECK(rcl_publish(&reset_flag_publisher, &reset_flag_msg, NULL));
  RCSOFTCHECK(rcl_publish(&current_state_publisher, &current_state_msg, NULL)); // current state
  }
}

// Forward declarations of functions
void stop();
void goforward(int speed, int speed2);
void turn(int speed);
void angle_callback(const void * msgin);
void state_callback(const void * msgin);
void obstacle_callback(const void * msgin);
float getUnwrappedAngle(float);
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
      //while( /*!bno.isFullyCalibrated()){ imu_calibrate(&bno, &event);*/ }
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
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
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
  RCCHECK(rclc_subscription_init_best_effort(
    &state_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "setpoint/state"));
  
  RCCHECK(rclc_subscription_init_best_effort(
    &obstacle_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "obstacleStatus"));

  RCCHECK(rclc_subscription_init_best_effort(
    &angleSetpoint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "setpoint/angle"));

/*  RCCHECK(rclc_publisher_init_best_effort(
    &currentAngle_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/currentAngle"));*/

  RCCHECK(rclc_publisher_init_best_effort(
    &angleError_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/angleError"));

 /* RCCHECK(rclc_publisher_init_best_effort(
    &current_velocity_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/currentSpeed"));*/

  RCCHECK(rclc_publisher_init_best_effort(
    &relative_angle_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "loop/relativeAngle"));

    RCCHECK(rclc_publisher_init_default(
    &reset_flag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "loop/reset_flag"));
    
  RCCHECK(rclc_publisher_init_default(
    &current_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "loop/currentState"));
  
    // create timer,
  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

 // create executor
  //RCCHECK(rclc_executor_init(&current_angle_msg_executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&current_angle_msg_executor, &timer));
  //RCCHECK(rclc_executor_init(&current_velocity_msg_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&relative_angle_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&relative_angle_executor, &timer));

  RCCHECK(rclc_executor_init(&angle_error_msg_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&angle_error_msg_executor, &timer));

  RCCHECK(rclc_executor_init(&reset_flag_executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_init(&current_state_executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_init(&angleExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&angleExecutor, &angleSetpoint_subscriber, &angleSetpoint_msg, &angle_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&state_subscriber_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&state_subscriber_executor, &state_subscriber, &state_subscriber_msg, &state_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&obstacle_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&obstacle_executor, &obstacle_subscriber, &obstacle_msg, &obstacle_callback, ON_NEW_DATA));

  /* ==================== CONTROLLER  ========================== */


  // Initialize the PID controller

  bool check = anglePID.setup();

  anglePID.setControllerType(controllertype);
  
  // Set the sample time (in milliseconds)
  anglePID.setSampleTime(1);
  // Set output limits for turning speed
  anglePID.setOutputLimits(-max_speed, max_speed);
  
  // Initialize the motor control pins as outputs
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Wire.begin();

  currentAngle_msg.data = 0;
  //current_velocity_msg.data = 0;

  stop();

  startTime = millis();
  
  
}//end setup


/* END SETUP */

/* =========== LOOP VARIABLES ========*/
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

  RCCHECK(rclc_executor_spin_some(&angleExecutor, RCL_MS_TO_NS(1)));
  RCCHECK(rclc_executor_spin_some(&state_subscriber_executor, RCL_MS_TO_NS(1)));
  RCCHECK(rclc_executor_spin_some(&obstacle_executor, RCL_MS_TO_NS(1)));
  //RCSOFTCHECK(rclc_executor_spin_some(&current_angle_msg_executor, RCL_MS_TO_NS(10)));
  RCSOFTCHECK(rclc_executor_spin_some(&relative_angle_executor, RCL_MS_TO_NS(1)));
  RCSOFTCHECK(rclc_executor_spin_some(&angle_error_msg_executor, RCL_MS_TO_NS(1)));
  // Loop spin rate
  currentTime = millis();
  tau = (currentTime - startTime);
  if(tau > 1){
    // Loop spin rate
    startTime = millis();

    // Update current orientation (from IMU)
    imu::Vector<3> angle = bno.getQuat().toEuler();
    currentAngle = getUnwrappedAngle(angle.x()*radToDeg);

    if(justonce == true){
      
      stop();
      initial_angle = currentAngle;
      angleMeasure = currentAngle;
      angleSetpoint = currentAngle;
      // Compute the PID controller output
      anglePID.compute();
      justonce = false;
      reset_flag_msg.data = false;
      //RCSOFTCHECK(rcl_publish(&reset_flag_publisher, &reset_flag_msg, NULL));
    }
    // Update motion state (based on WebGUI)
    updateState();

    switch (desired_motion_state)
    {
      case WheelchairState::Stopped:
        current_state_msg.data = 0;
        stop();
        motorpwm1 = 0;
        motorpwm2 = 0;
        angleSetpoint = currentAngle;
        break;

      case WheelchairState::MovingForward:
        current_state_msg.data = 1;
        angleSetpoint = initial_angle + angleSetpoint_msg.data;
        angleMeasure = currentAngle;
        anglePID.compute();
        error_angle = anglePID.geterror();
        angle_error_msg.data = error_angle;
        if(abs(error_angle) > 2){
          motorpwm2 = ControllerOutput;
        }else{
            motorpwm2 = 0;}
        motorpwm1 = max_speed-15;
        if(obstacle_msg.x == 1){
            reset_flag_msg.data = true;
            reset();
        }else{goforward(motorpwm1,motorpwm2);}
        break;

      case WheelchairState::Turning:
         // this is to publish for moving forward only
        angleSetpoint = initial_angle + angleSetpoint_msg.data;
        angleMeasure = currentAngle;
        anglePID.compute();
        error_angle = anglePID.geterror();
        angle_error_msg.data = error_angle;
        current_state_msg.data = 2;

        if(abs(error_angle) > 2){
          motorpwm1 = 0;
          motorpwm2 = ControllerOutput;

          if(ControllerOutput>0){ // turn left ig
            if(obstacle_msg.y == 1){
                reset_flag_msg.data = true;
                reset();
                } else{ turn(motorpwm2);}
            }else if(ControllerOutput<0){
                if(obstacle_msg.z == 1){
                    reset_flag_msg.data = true;
                    reset();
                }else{turn(motorpwm2);}
            }
          //turn(motorpwm2);
        }else{
            current_state_msg.data = 3;
            motorpwm1 = 0;
            motorpwm2 = 0;
            stop();
        }
        break;

      default:
        motorpwm1 = 0;
        motorpwm2 = 0;
        stop();
        break;
    }
    
  }

  relative_angle_msg.data = currentAngle - initial_angle;
  
  //current_velocity_msg.data = motorpwm1;
  currentAngle_msg.data = currentAngle;

  RCSOFTCHECK(rcl_publish(&reset_flag_publisher, &reset_flag_msg, NULL));
  RCSOFTCHECK(rcl_publish(&current_state_publisher, &current_state_msg, NULL)); // current state

  //delay(BNO055_SAMPLERATE_DELAY_MS);


}// end loop

/*====== UTILITY FUNCTIONS ============*/

/* LIMIT VALUES FUNCTION */
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

float previousAngle = 0.0;
int completeRotations = 0;

float getUnwrappedAngle(float newAngle) {
    float delta = newAngle - previousAngle;

    if (delta > 180) {
        // Counter-clockwise crossing 0
        completeRotations -= 1;
    } else if (delta < -180) {
        // Clockwise crossing 360
        completeRotations += 1;
    }

    previousAngle = newAngle;
    return newAngle + completeRotations * 360;
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
  //reset_flag_msg.data = false;
}

void goforward(int speed, int speed2){
  
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear + speed);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn-speed2);
  Wire.endTransmission();
  //reset_flag_msg.data = false;
}

void turn(int speed){
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn-speed); // if positive then turns left ig and negative turns right
  Wire.endTransmission();
  //reset_flag_msg.data = false;
}

void reset(){
    stop();
    motorpwm1 = midlinear;
    motorpwm2 = midturn;
    previousAngle = 0.0;
    completeRotations = 0;
    justonce = true;
    reset_flag_msg.data = true;
    RCSOFTCHECK(rcl_publish(&reset_flag_publisher, &reset_flag_msg, NULL));
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

void obstacle_callback(const void * msgin){
  

}