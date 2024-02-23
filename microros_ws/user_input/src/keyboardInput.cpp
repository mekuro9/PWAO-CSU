#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

/* This node gets keyboard characters from terminal and publishes it to topic 'userinput'.
Which is subscribed to by teensy node  */

/* 
 //              W(up)
 //     A(left)   S(down)   D(right)
 // X(stop)
 // W corresponds to go forward 30 cms
 // S corresponds to go back 30 cms
 // A is turn left by 10 degs
 // D is turn right by 10 degs
 // Additionally X is used to stop moving -> sets the velocity and acceleration to 0
 // Max speed will be limited to a certain speed (TO DO in teensy controller)
*/

bool need_exit = false;

class keyboardInput : public rclcpp::Node
{
  public:
  // Constructor
  keyboardInput() : Node("keyboard_input"){
    keypublisher_ = this->create_publisher<std_msgs::msg::Int32>("userinput", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
     
  }
  void loop_keyboard();
  void init();
 
  private:
  

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr keypublisher_;
  bool data_bit_;

};

void ExitHandler(int sig){
    (void)sig;
    need_exit = true;
    std::cout<<"In exit handler";
}

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto keyboard_input = std::make_shared<keyboardInput>();
  keyboard_input->loop_keyboard();
  signal(SIGINT,ExitHandler);
  rclcpp::shutdown();
 
  return 0;
}

void keyboardInput::loop_keyboard(){
  
  auto key_int = std::make_shared<std_msgs::msg::Int32>();
  char key;
  init();

  while (rclcpp::ok() && !need_exit) {

    std::cin>>key;
    key_int->data = int(key);
    keypublisher_->publish(*key_int);
    rclcpp::spin_some(shared_from_this());

  }

}

void keyboardInput::init(){
  std::cout<<"------Keyboard input (all caps)-------------\n";
  std::cout<<"---------------------------------\n";
  std::cout<< "        W(up)\n";
  std::cout<< "A(left)   S(down)   D(right)\n";
  std::cout<<"        X(stop)\n";
  std::cout<<"---------------------------------\n";
}

// make a class user_input and keyoardInput will be a child class, but what will it inherit ??
// For now, keep it as a keyboard input as a parent class  