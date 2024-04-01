#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "error_handler/ErrorHandler.hpp"

using namespace std::chrono_literals;

struct TopicData {
  
  std::string topicName;
  rclcpp::Time lastMessageTime;

  //Constructor 
  TopicData(std::string name, rclcpp::Time current_time): topicName(name), lastMessageTime(current_time){}
};

class ErrorPublisher : public rclcpp::Node
{
public:
  ErrorPublisher()
  : Node("error_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("error", 5);
    
    initializeTopicData();
    setupSubscriptions();

    timer_ = this->create_wall_timer(
      500ms, std::bind(&ErrorPublisher::timer_callback, this));
  }

private:

  void initializeTopicData(){
    // make this for sensor data and communication failure
    rclcpp::Time current_time = this->now();
    topics_.emplace_back("scan",current_time);
    topics_.emplace_back("scan/processed",current_time);
    topics_.emplace_back("imu",current_time);
    topics_.emplace_back("color/image",current_time);
  }

  void setupSubscriptions(){
    for(auto& topic : topics_){
      auto callback = [this, &topic](const std_msgs::msg::String::SharedPtr msg){
        topic.lastMessageTime = this->now();
      };
      topicSubscriptions_.push_back(this->create_subscription<std_msgs::msg::String>(topic.topicName,10, callback));
    }
  }
  
  void publish_error(ErrorHandler& error)
  { 
    std_msgs::msg::String message;  
    message.data = error.getMessage();
    publisher_->publish(message);
    //can also log error here
    //error.logError();
  }
  void timer_callback()
  {
    for(auto& topic: topics_){
      if((this->now() - topic.lastMessageTime).seconds() > timeoutThreshold_){
        std::string error_message = "Timeout detected on " + topic.topicName;
        
        ErrorType errorType;
        // here logic switch case or if else based on topic name
        if(topic.topicName == "scan"){
          errorType = ErrorType::NO_SENSOR_MESSAGE;
        } else if(topic.topicName == "scan/processed"){
          errorType = ErrorType::NO_SENSOR_MESSAGE;
        }else if(topic.topicName == "imu"){
          errorType = ErrorType::NO_SENSOR_MESSAGE;
        }else if(topic.topicName == "color/image"){
          errorType = ErrorType::NO_SENSOR_MESSAGE;
        }

        ErrorHandler ErrorHandler(this, errorType, error_message);
        ErrorHandler.logError();

        std_msgs::msg::String msg;
        msg.data = ErrorHandler.getMessage();
        publisher_->publish(msg);
      }
    }
  }

  std::vector<TopicData> topics_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> topicSubscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  const int timeoutThreshold_ = 3; // 3 seconds
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ErrorPublisher>());
  rclcpp::shutdown();
  return 0;
  
}
