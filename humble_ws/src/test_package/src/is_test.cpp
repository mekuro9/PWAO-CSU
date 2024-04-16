#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class EchoNode : public rclcpp::Node
{
public:
    EchoNode() : Node("echo_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("echo_topic", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "webinput/counter", 10, std::bind(&EchoNode::webinput_callback, this, _1)); 
    }

private:
    void webinput_callback(const std_msgs::msg::String & msg) const{

      auto echo_msg = std_msgs::msg::String();
      echo_msg.data = "ROS recieved: " + msg.data;
      publisher_->publish(echo_msg);
      RCLCPP_INFO(this->get_logger(), "Echoing back: '%s'", echo_msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EchoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
