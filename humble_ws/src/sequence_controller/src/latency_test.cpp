#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class LatencyTest : public rclcpp::Node
{
public:
  LatencyTest()
  : Node("latency_test")
  {
    subscription_command_ = this->create_subscription<std_msgs::msg::String>(
      "/webinput/command", 10, std::bind(&LatencyTest::webinput_callback, this, _1));
    subscription_time_ = this->create_subscription<std_msgs::msg::String>(
      "/webinput/timestamp", 10, std::bind(&LatencyTest::timer_callback, this, _1));
    subscription_count_= this->create_subscription<std_msgs::msg::String>(
      "/webinput/counter", 10, std::bind(&LatencyTest::count_callback, this, _1));
  }

private: 
  void webinput_callback(const std_msgs::msg::String & msg) const
  {
    //RCLCPP_INFO(this->get_logger(), "Command: '%s'", msg.data.c_str());
  }

  void timer_callback(const std_msgs::msg::String & msg) const {

    RCLCPP_INFO(this->get_logger(), "Time sent: '%s'", msg.data.c_str());

  }

  void count_callback(const std_msgs::msg::String & msg) const {

    RCLCPP_INFO(this->get_logger(), "Count: '%s'", msg.data.c_str());

  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_command_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatencyTest>());
  rclcpp::shutdown();
  return 0;
}

