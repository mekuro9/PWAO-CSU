#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher()
    : Node("sim_lidar_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Header>("lidar_topic", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LidarPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Header();
        message.stamp = this->get_clock()->now();
        message.frame_id = "Lidar count: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.frame_id.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPublisher>());
    rclcpp::shutdown();
    return 0;
}
