#include <chrono>
#include <functional>
#include <memory>
#include <boost/bind/bind.hpp>
#include "rclcpp/rclcpp.hpp"

//#include "cv_bridge/cv_bridge.h"
//#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
//#include "sensor_msgs/msg/camera_info.h"
//#include "sensor_msgs/msg/compressed_image.hpp"
//#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>


using namespace std::chrono_literals;

class TestSync :public rclcpp::Node
{
    public:
        TestSync() : Node("test_sync"),sync(mySyncPolicy(5),lidar_subscriber_,image_subscriber_){
        
        lidar_subscriber_.subscribe(this,"scan");
        image_subscriber_.subscribe(this,"/color/image");
        
        sync.registerCallback(&TestSync::callback,this);

        }

    private:
    
        // methods
        void callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg, const sensor_msgs::msg::Image::SharedPtr image_msg){
            
            if(lidar_msg == NULL || image_msg == NULL){
                // this is never excecuted so means that callback is called only when both messages are published
                std::cout<<"Lidar or image not recieved"<<std::endl;
            }
            else{
            
                RCLCPP_INFO(this->get_logger(), "Recieved Laser and Image message" );
            }
            
        }


        // members

        message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_subscriber_;
        message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::Image> mySyncPolicy;
        message_filters::Synchronizer<mySyncPolicy> sync;
        
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSync>());
  rclcpp::shutdown();
  return 0;
  
}