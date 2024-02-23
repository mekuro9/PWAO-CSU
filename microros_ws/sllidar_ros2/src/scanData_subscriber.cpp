
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

using std::placeholders::_1;

#define RAD2DEG(x) ((x)*180./M_PI)

class scanData_subscriber : public rclcpp::Node
{
  public:
    scanData_subscriber()
    : Node("minimal_subscriber")
    {
      scanData_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&scanData_subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
    {   
    int count = scan->scan_time / scan->time_increment;
    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
            } 
    RCLCPP_INFO(this->get_logger(), "count: '%d'", count);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanData_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanData_subscriber>());
  rclcpp::shutdown();
  return 0;
}