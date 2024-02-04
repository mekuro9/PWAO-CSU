/*==============================================================
// Filename :       loop_controller.cpp
// Authors :        Mrinal Magar 
// Version :        v1.0
// License :  
// Description :    Loop controller for sending commands to power wheelchair. 
//                  Sensors used are IMU BNO055, RPlidar S2       
==============================================================*/

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>

#include <vector>
#include <string>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include "imu_util.h"
#include "lidar_util.h"


/* ================== IMU =================== */

bool zero = true;
bool calibrate = true;
int zeroTime = 50;
double degToRad = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(11);

/* ================= RPLIDAR ================ */

#define RAD2DEG(x) ((x)*180./M_PI)
std::vector<float> scan_angle;
std::vector<float> scan_range;

/* ================= USER INPUT ================ */

std::string key_;

/* ================= Micro-ros ============== */

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t teensy_imuPub_; // publisher for imu data: topic name 'imu'
sensor_msgs__msg__Imu imuData;   // message type for imu
rclc_executor_t executor_pub;
rcl_timer_t timer;

rcl_subscription_t teensy_lidarSub_; // subscriber for lidar data, topic name: 'scan'
sensor_msgs__msg__LaserScan lidarscan;  // message type for lidarscan
rclc_executor_t executor_sub;

rcl_subscription_t teensy_keyboard_; // subscriber for keyboard input 
std_msgs__msg__String keyboard_data; // message type for keyboard data
rclc_executor_t executor_keyboard;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
// Timer callback function for publishing imu data to ros2 
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if(timer != NULL){
    RCSOFTCHECK(rcl_publish(&teensy_imuPub_, &imuData, NULL));
  }
}

//Subscriber callback function for lidar data
void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__LaserScan * lidarscan = (const sensor_msgs__msg__LaserScan *)msgin;
  int count = lidarscan->scan_time / lidarscan->time_increment; 
  for (int i = 0; i < count; i++){
    float degree = RAD2DEG(lidarscan->angle_min + lidarscan->angle_increment*i);
    scan_angle.push_back(degree);
    scan_range.push_back(lidarscan->ranges.data[i]);

  }
}

void keyboard_callback(const void * msgin)
{
  const std_msgs__msg__String * keyboard_data = (const std_msgs__msg__String *)msgin;
  key_ = keyboard_data.data;
}

void setup() {
    
    /* =========================== Input ================================ */

  key_ = "X";
    /* =========================== IMU ================================ */

  if(!bno.begin()){
    // Send imu_status as not connected TO DO ********* HOW ??
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

  // create subscriber for lidar
  RCCHECK(rclc_subscription_init_default(
    &teensy_lidarSub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"));

  //create subscriber for keyboard input
  RCCHECK(rclc_subscription_init_default(
    &teensy_keyboard_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "userinput"))

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &teensy_imuPub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"
  ));

  //create timer (imu output at the rate of 100hz)
  const unsigned int timer_timeout = 10; //10ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));


  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &teensy_lidarSub_, &lidarscan, &subscription_callback, ON_NEW_DATA));
  
  RCCHECK(rclc_executor_init(&executor_keyboard, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_keyboard, &teensy_keyboard_, &keyboard_data, &keyboard_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  
  /* ==================== CONTROLLER  ========================== */


  
}

void loop() {

  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  
  do{

  int angle_start = find_closest(scan_angle,-180);
  int angle_end = find_closest(scan_angle,-90);
  int angle_start2 = find_closest(scan_angle,90);
  int angle_end2 = find_closest(scan_angle,180);

  make_blind(scan_angle, scan_range, angle_start, angle_end); 
  make_blind(scan_angle, scan_range, angle_start2, angle_end2); 

  }while(false);


  int i = 0;
  float obstacleThreshold = 0.25;  // Adjust this threshold based on your environment and sensor characteristics

  for(auto itr : scan_range){

    if(itr > obstacleThreshold){
      ++i;
        if(i > 15){
          //stop the wheelchair
          break;
        }
    }

  }

 
  }
