#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp" 
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <math.h>
#include "obstacle_detection/LidarUtility.h"
using namespace std::chrono_literals;
using std::placeholders::_1;
#define RAD2DEG(x) ((x)*180./M_PI)

class ObstacleDetector : public rclcpp::Node
{
  public:
    ObstacleDetector()
    : Node("ObstacleDetector"), count_(0)
    {  
    // subscriber of scan topic by rplidar
      scanData_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&ObstacleDetector::topic_callback, this, _1));

      publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("obstacleStatus", rclcpp::SensorDataQoS());
      publisher_with_time_ = this-> create_publisher<geometry_msgs::msg::Vector3Stamped>("obstacleStatus/Stamped", rclcpp::SensorDataQoS());
      for(int i=0; i<3240; i++){
      scan_angle[i] = 0;
      scan_range[i] = 0;
      }
    }

  private:

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {   
      int count = scan->scan_time / scan->time_increment;
      for (int i = 0; i < count; i++) { // adding mpi here to convert angles to 0-360 left to right
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i + M_PI);
        float range;
        scan_angle[i] = degree;
        if(scan->ranges[i] > scan->range_max){
          range = 30;}
        else if(scan->ranges[i] < scan->range_min){
          range = 0.15;}
        else if(isinf(scan->ranges[i])){
          range = 30;}
        else{
          range = scan->ranges[i];}
        scan_range[i] = range;
      }
      geometry_msgs::msg::Vector3 obstacleStatus;
      geometry_msgs::msg::Vector3Stamped obstacle;

      LidarUtility scanner(scan_angle,scan_range);
      std::vector<std::pair<float,float>>frontLeftScan = scanner.getScanDataInRange(1,30,3240);
      std::vector<std::pair<float,float>>frontRightScan = scanner.getScanDataInRange(330,359,3240);
      bool frontLeftObstacle = scanner.checkObstacleInFilteredData(frontLeftScan,0.5);
      bool frontRightObstacle = scanner.checkObstacleInFilteredData(frontRightScan,0.5);
      float front = ((frontLeftObstacle == true) or (frontRightObstacle==true)) ? 1 : 0;

      std::vector<std::pair<float,float>>leftScan = scanner.getScanDataInRange(30,90,3240);
      std::vector<std::pair<float,float>>rightScan = scanner.getScanDataInRange(270,330,3240);
      bool leftObstacle = scanner.checkObstacleInFilteredData(leftScan,0.5);
      float left = (leftObstacle == true) ? 1 : 0;

      bool rightObstacle = scanner.checkObstacleInFilteredData(rightScan,0.5);
      float right = (rightObstacle == true) ? 1: 0;

      obstacle.header.frame_id = std::to_string(count_++);
      obstacle.header.stamp = this->get_clock()->now();

      obstacleStatus.x = front;
      obstacleStatus.y = left;
      obstacleStatus.z = right;

      obstacle.vector.x = front;
      obstacle.vector.y = left;
      obstacle.vector.z = right;

      publisher_->publish(obstacleStatus);
      RCLCPP_INFO(this->get_logger(), "Obstacle Publisher front: '%f' Obstacle left: '%f', Obstacle right: '%f'", front, left, right);
      publisher_with_time_->publish(obstacle);
      
    
    }
  
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanData_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_with_time_;
    float scan_angle[3240];
    float scan_range[3240];
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}