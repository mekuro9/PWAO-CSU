#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <string>

//#include <Arduino.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>
#include <vector>
//#include <EEPROM.h>
#include <math.h>
//#include "imu_util.h"
//#include "lidar_util.h"


/* ================= RPLIDAR ================ */
float scan_angle[3240];
float scan_range[3240];
#define RAD2DEG(x) ((x)*180./M_PI)
const int obstacleThreshold = 0.30; 

int scanLeftPI_3(){
  int angle;
  for(int i = 0; i < 1080; i++){
    if(scan_range[i]< obstacleThreshold){
      angle = (int)scan_angle[i];
      break;
    }
    else{
      angle = 500;
      continue;
    }
  }
  return angle;

}

int scanRightPI_3(){
  int angle;
  for(int i = 0; i < 1080; i++){
    if(scan_range[3239-i]< obstacleThreshold){
     angle = (int)scan_angle[3239-i];
      break;
    }
    else{
      angle = 500;
      continue;
    }
  }
  return angle;
}
/* ================= Micro-ros =============== */

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t teensy_lidarSub_; // subscriber for lidar data, topic name: 'scan'
sensor_msgs__msg__LaserScan lidarscan;  // message type for lidarscan
rclc_executor_t executor_sub;

rcl_publisher_t teensy_lidarPub_; // publisher for string data: topic name 'scanStatus'
std_msgs__msg__Int32 scan_status;
//std_msgs__msg__Int32 scan_status;   // message type 
rclc_executor_t executor_pub;
rcl_timer_t timer;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//Subscriber callback function for lidar data
void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__LaserScan * lidarscan = (const sensor_msgs__msg__LaserScan *)msgin;
  int count = lidarscan->scan_time / lidarscan->time_increment; 
  for (int i = 0; i < count; i++){
    float degree = RAD2DEG(lidarscan->angle_min + lidarscan->angle_increment*i + M_PI);
    float range;
    scan_angle[i] = degree;

    if(lidarscan->ranges.data[i] > lidarscan->range_max){
      range = 30;
    }
    else if(lidarscan->ranges.data[i] < lidarscan->range_min){
      range = 0.15;

    }
    else if(isinf(lidarscan->ranges.data[i])){
      range = 15;
    }
    else{
      range = lidarscan->ranges.data[i];
    }
    scan_range[i] = range;

  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  
  int leftScan = scanLeftPI_3();
  int rightScan = scanRightPI_3();

    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  if(leftScan == 500 && rightScan == 500){
    
    scan_status.data = 1500;
    //RCLCPP_INFO(this->get_logger(), " No obstacle on the right and left");
  } else if(leftScan != 500 and rightScan == 500){
    scan_status.data = leftScan;
    //RCLCPP_INFO(this->get_logger(),"Obstacle to the left at angle : '%s'", scan_status.data.c_str());
  } else if(leftScan == 500 and rightScan != 500){
    scan_status.data = rightScan;
    //RCLCPP_INFO(this->get_logger(),"Obstacle to the right at angle : '%s'", scan_status.data.c_str());
  }else if(leftScan != 500 and rightScan != 500){
    scan_status.data = leftScan + rightScan;
    //RCLCPP_INFO(this->get_logger(),"Obstacle to the right : '%s' and to the left at : '%s'", rightScan.c_str(), leftScan.c_str());
  }else{
    scan_status.data = 180; // this shouldnt be zero
    //RCLCPP_INFO(this->get_logger(),"Invalid Scan Data");  
  }
  RCLC_UNUSED(last_call_time);
  if(timer != NULL){
    RCSOFTCHECK(rcl_publish(&teensy_lidarPub_, &scan_status, NULL));
  }
}



void setup() {

  //Serial.begin(115200);
  //delay(1000);

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
    // create publisher
  RCCHECK(rclc_publisher_init_default(
    &teensy_lidarPub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scanStatus"
  ));

  //create timer ( output at the rate of 10hz)
  const unsigned int timer_timeout = 100; //100ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &teensy_lidarSub_, &lidarscan, &subscription_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

}
 //scan_range and scan_angle (angle is from -179 to +179)

void loop() {

  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));

  delay(100); 
}