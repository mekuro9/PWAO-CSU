#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class WheelchairState{
    Stopped,
    MovingForward,
    TurningRight,
    TurningLeft
};

class SequenceController : public rclcpp::Node {
public:
    SequenceController() 
        : Node("sequence_controller"), 
          state_(WheelchairState::Stopped), 
          forward(false), turn(0), current_angle_(0.0), angle_error_(0.0), 
          current_velocity_(0.0), velocity_error_(0.0),is_forward_timer_active_(false)  {
        
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/webinput/command", 1, std::bind(&SequenceController::webinput_callback, this, _1));

        current_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "loop/currentAngle", 1, std::bind(&SequenceController::currentAngleCallback, this, _1));

        angle_error_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "loop/angleError", 1, std::bind(&SequenceController::angleErrorCallback, this, _1));

        current_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "loop/currentSpeed", 1, std::bind(&SequenceController::currentVelocityCallback, this, _1));

        velocity_error_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "loop/speedError", 1, std::bind(&SequenceController::velocityErrorCallback, this, _1));

        speed_publisher_ = this->create_publisher<std_msgs::msg::Bool>("setpoint/speed",1);
        angle_publisher_ = this->create_publisher<std_msgs::msg::Int32>("setpoint/angle",1);        


        timer_ = this->create_wall_timer(
            100ms, std::bind(&SequenceController::timer_callback,this)
        );

        
    }

private:
        
    void webinput_callback(const std_msgs::msg::String & msg) {
        if (msg.data == "stop") {
            transitionToState(WheelchairState::Stopped);
        } else if (msg.data == "forward") {
            transitionToState(WheelchairState::MovingForward);
        } else if (msg.data == "right") {
            transitionToState(WheelchairState::TurningRight);
        } else if (msg.data == "left") {
            transitionToState(WheelchairState::TurningLeft);
        }
    }

    void currentAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_angle_ = msg->data;
    }

    void angleErrorCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        angle_error_ = msg->data;
    }

    void currentVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_velocity_ = msg->data;
    }

    void velocityErrorCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        velocity_error_ = msg->data;
    }

    void transitionToState(WheelchairState new_state) {
        // Ignore new commands if the forward timer is active
        if (is_forward_timer_active_ && new_state != WheelchairState::Stopped) {
            return;
        }

        // Logic for transitioning to each state
        switch (new_state) {
            case WheelchairState::MovingForward:
                if (state_ != WheelchairState::MovingForward) {
                    forward = true;
                    turn = 0;
                    is_forward_timer_active_ = true;
                    speed_duration_timer_ = this->create_wall_timer(
                        3s, [this]() {
                            transitionToState(WheelchairState::Stopped);
                            is_forward_timer_active_ = false;
                        });
                }
                break;

            case WheelchairState::TurningRight:
                // Stop the forward timer if it's running
                if (is_forward_timer_active_) {
                    speed_duration_timer_->cancel();
                    is_forward_timer_active_ = false;
                }
                forward = false;
                turn = 1;
                break;

            case WheelchairState::TurningLeft:
                // Stop the forward timer if it's running
                if (is_forward_timer_active_) {
                    speed_duration_timer_->cancel();
                    is_forward_timer_active_ = false;
                }
                forward = false;
                turn = -1;
                break;

            case WheelchairState::Stopped:
            default:
                // Cancel the forward timer if it's running
                if (is_forward_timer_active_) {
                    speed_duration_timer_->cancel();
                    is_forward_timer_active_ = false;
                }
                forward = false;
                turn = 0;
                break;
        }

        state_ = new_state;
    }

    
    void timer_callback(){
        
        std_msgs::msg::Bool speed_msg;
        std_msgs::msg::Int32 angle_msg; 

        // Set the message data based on the current state
        speed_msg.data = forward;
        angle_msg.data = turn;

        // Publish the commands
        speed_publisher_->publish(speed_msg);

        if (std::abs(angle_error_) <= angle_error_threshold_) {
            angle_msg.data = turn;
        } else {
            angle_msg.data = 0; // Do not change the angle if error is too high
        }

        angle_publisher_->publish(angle_msg);
          
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_angle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_error_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_error_subscriber_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr angle_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr speed_duration_timer_;

    WheelchairState state_; 
    bool forward;
    int turn;    
    std::string command_;

    double current_angle_;
    double angle_error_;
    double current_velocity_;
    double velocity_error_;
    const double angle_error_threshold_ = 1;
    bool is_forward_timer_active_;

    std::chrono::time_point<std::chrono::steady_clock> last_command_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}

