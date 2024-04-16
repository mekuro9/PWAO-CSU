#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <sstream>

class DataPublisher : public rclcpp::Node
{
public:
    DataPublisher() 
    : Node("data_publisher"), count_(0), index_(0), num_messages_(1000) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("data_topic", 10); //rclcpp::SensorDataQoS()
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DataPublisher::publish_data, this));
        sizes_ = {30, 100, 317, 1000, 3162, 10000, 31623, 100000, 316228, 1000000}; // Byte sizes
    }

private:
    void publish_data() {
        if (count_ >= num_messages_) {
            count_ = 0;
            ++index_;
            if (index_ >= sizes_.size()) {
                index_ = 0; // Restart with first size or stop the timer if done with all sizes
            }
        }

        std_msgs::msg::String message;
        std::stringstream ss;
        // Append count, message size and timestamp
        ss << count_ << " " << std::string(sizes_[index_], 'A') << " " << this->now().nanoseconds();
        message.data = ss.str();
        publisher_->publish(message);
        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<size_t> sizes_;
    size_t count_;
    size_t index_;
    size_t num_messages_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
