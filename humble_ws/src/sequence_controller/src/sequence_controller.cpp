#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
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
          state(0), turn(0), current_angle_(0.0), angle_error_(0.0), 
          relative_angle_(0.0),is_forward_timer_active_(false)  {
        
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/webinput/command", 1, std::bind(&SequenceController::webinput_callback, this, _1));

        current_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "loop/currentAngle", 1, std::bind(&SequenceController::currentAngleCallback, this, _1));

        angle_error_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "loop/angleError", 1, std::bind(&SequenceController::angleErrorCallback, this, _1));

        relative_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "loop/relativeAngle", 1, std::bind(&SequenceController::relativeAngleCallback, this, _1));

        
        state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("setpoint/state",1);
        angle_publisher_ = this->create_publisher<std_msgs::msg::Int32>("setpoint/angle",1);        


        timer_ = this->create_wall_timer(
            10ms, std::bind(&SequenceController::timer_callback,this)
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

    void currentAngleCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_angle_ = msg->data;
    }

    void angleErrorCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        angle_error_ = msg->data;
    }

    void relativeAngleCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        relative_angle_ = msg->data;
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
                    state = 1;
                    turn += 0;
                    is_forward_timer_active_ = true;

                    RCLCPP_INFO(this->get_logger(), "I am in Forward: '%s'", std::to_string(state));
                    
                    speed_duration_timer_ = this->create_wall_timer(
                        2s, [this]() {
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
                state = 2;
                RCLCPP_INFO(this->get_logger(), "I am in Right: '%s'", std::to_string(state));
                turn += 1*rotation_step;
            break;
            case WheelchairState::Stopped:
                if (is_forward_timer_active_) {
                    speed_duration_timer_->cancel();
                    is_forward_timer_active_ = false;
                }
                state = 0;
                RCLCPP_INFO(this->get_logger(), "I am in Stopped: '%s'", std::to_string(state));
                turn += 0;
            break;
            case WheelchairState::TurningLeft:
                // Stop the forward timer if it's running
                if (is_forward_timer_active_) {
                    speed_duration_timer_->cancel();
                    is_forward_timer_active_ = false;
                }
                state = 2;
                RCLCPP_INFO(this->get_logger(), "I am in Left: '%s'", std::to_string(state));
                turn += -1*rotation_step;
            break;
            
        }

        state_ = new_state;
    }

    
    void timer_callback(){
        
        std_msgs::msg::Int32 state_msg;
        std_msgs::msg::Int32 angle_msg; 

        // Set the message data based on the current state
        state_msg.data = state;


        // Publish the commands
        state_publisher_->publish(state_msg);
        if(state == 2){
            if (std::abs(relative_angle_ - turn) > std::abs(angle_threshold_)) {
                
            } else {
                transitionToState(WheelchairState::Stopped);
           
            }
        }
        angle_msg.data = turn;
        angle_publisher_->publish(angle_msg);
          
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_angle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_error_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr relative_angle_subscriber_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr angle_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr speed_duration_timer_;

    WheelchairState state_;
    int state; 
    int turn;    
    std::string command_;

    double current_angle_;
    double angle_error_;

    double relative_angle_;

    const double angle_threshold_ = 1;

    bool is_forward_timer_active_;

    const int rotation_step = 10;

    std::chrono::time_point<std::chrono::steady_clock> last_command_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}

