#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>

#define RAD2DEG(x) ((x)*180./M_PI)

rcl_publisher_t teensy_imuPub_; // publisher for imu data: topic name 'imu'
rcl_subscription_t teensy_lidarSub_; // subscriber for lidar data, topic name: 'scan'
sensor_msgs_msg__Imu imuData;   // message type for imu
sensor_msgs__msg__LaserScan lidarscan;  // message type for lidarscan
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float scan_angle[3239];
float scan_range[3239];

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__LaserScan * lidarscan = (const sensor_msgs__msg__LaserScan *)msgin;
  int count = lidarscan->scan_time / lidarscan->time_increment; 
  for (int i = 0; i < count; i++){
    float degree = RAD2DEG(lidarscan->angle_min + lidarscan->angle_increment*i);
    scan_angle[i] = degree;
    scan_range[i] = lidarscan->ranges.data[i];

  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &teensy_lidarSub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"));

  // create publisher
  RCCHECK(rckc_publisher_init_default(
    &teensy_imuPub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"
  ));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &teensy_lidarSub_, &lidarscan, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));


}