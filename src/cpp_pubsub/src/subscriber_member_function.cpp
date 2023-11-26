// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <string>

#include <bits/stdc++.h>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    from_arduino_ = this->create_publisher<std_msgs::msg::String>("arduino", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
  
    serialPort_ = open("/dev/ttyACM0", O_RDWR);    
    if (serialPort_ < 0)
      printf("Error opening port: %d %s\n", errno, strerror(errno));
    
    termios tty;
    if (tcgetattr(serialPort_, &tty) != 0)
      printf("Error getting props: %d %s\n", errno, strerror(errno));

    cfsetspeed(&tty, B115200);

    if (tcsetattr(serialPort_, TCSANOW, &tty) != 0)
      printf("Error setting props: %d %s\n", errno, strerror(errno));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    std::string data = "FROM ROS2: " + msg.data + "\r\n"; 
    write(serialPort_, data.c_str(), data.length());
  }
  void read_serial(){
    read(serialPort_,buffer_,64);
  }

  void timer_callback()
  {
    read_serial();
    auto message = std_msgs::msg::String();
    s_ = buffer_;
    message.data = "From Arduino " + s_;
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    from_arduino_->publish(message);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int serialPort_;
  rclcpp::TimerBase::SharedPtr timer_;  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr from_arduino_;
  char buffer_[64];
  std::string s_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
