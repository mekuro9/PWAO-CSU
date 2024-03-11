#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.h"
#include "sensor_msgs/msg/compressed_image.hpp"
//#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

#include "lidar_visualization/Point2Image.hpp"
#include "lidar_visualization/Polar2Cartesian.hpp"

using namespace std::chrono_literals;

class lidarImagePublisher : public rclcpp::Node
{
public:
  lidarImagePublisher()
  : Node("lidar2image") 
    //,transformation_matrix_({1.0, 0.0, -0.54402111,5.44021111,0.0,1.0,0.0,0.0, 0.54402111,0.0,-0.83907153, 8.39071529,0.0,0.0,0.0,1.0})
   ,sync_(mySyncPolicy(5),lidar_subscriber_,image_subscriber_)
  {
    //{1.0, 0.0, -0.54402111,5.44021111,0.0,1.0,0.0,0.0, 0.54402111,0.0,-0.83907153,8.39071529,0.0,0.0,0.0,1.0};
    // for lidar data the QOS was: rclcpp::QoS(rclcpp::KeepLast(10)) "scan" topic
    //lidar_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "scan", rclcpp::SensorDataQoS());
    //TODO How to use custom QOS
    lidar_subscriber_.subscribe(this,"scan");
    // for image dont know but lets test with default sensordataQOS color/image topic
    //image_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "color/image", rclcpp::SensorDataQoS());
    image_subscriber_.subscribe(this,"/color/image");
    sync_.registerCallback(&lidarImagePublisher::sync_callback, this);
    
    //sync_ = std::make_shared<message_filters::Synchronizer<lidarImageSyncPolicy_>>(lidarImageSyncPolicy_(5), lidar_subscriber_, image_subscriber_);
    //sync_->registerCallback(&lidarImagePublisher::sync_callback, this);

    // Initialize image transport and publisher for compressed images
    // shared_from_this() returns a shared_ptr of rclcpp Node
    compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("lidarImage/compressed", 5);
  }

private:
  void sync_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg, const sensor_msgs::msg::Image::SharedPtr image_msg)
  {  
    if(lidar_msg == NULL || image_msg == NULL){
        // this is never excecuted so means that callback is called only when both messages are published
        std::cout<<"Lidar or image not recieved"<<std::endl;
       }
    else{

        RCLCPP_INFO(this->get_logger(), "Recieved Laser and Image message" );
        //Converting ros message to open cv image
        cv::Mat image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
        // do the utility class methods
        int count = lidar_msg->scan_time / lidar_msg->time_increment;
        for (int i = 0; i < count; i++) {
            float angle_rad = lidar_msg->angle_min + lidar_msg->angle_increment * i + 3.14159;
            auto max_scan = lidar_msg->range_max;
            auto min_scan = lidar_msg->range_min;
            auto scan = lidar_msg->ranges[i];
            
            if(scan > min_scan && scan < max_scan){
              auto cartesian_points = cartesian_coordinates_.Convert2Cartesian(angle_rad, scan);
              camera_coordinates_.transform2CameraFrame(cartesian_points.first,cartesian_points.second,0.15);
              
              auto camera_points = camera_coordinates_.transformTo2DImage();

              if(camera_points.first != -1 || camera_points.second != -1){
                //RCLCPP_INFO(this->get_logger(), "'%d'",camera_points.first );
                cv::circle(image, cv::Point(camera_points.first, camera_points.second), 5, CV_RGB(0, 255, 0), -1);
            
              }
            }

        } 
      
        cv_bridge::CvImage cv_image(image_msg->header, "bgr8", image);
        sensor_msgs::msg::CompressedImage::SharedPtr output_msg = cv_image.toCompressedImageMsg();
        compressed_pub_->publish(*output_msg);
        RCLCPP_INFO(this->get_logger(), "Sent Laser and Image message" );
       
       }  

  }

  //std::vector<float> transformation_matrix_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::Image> mySyncPolicy;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  Point2Image camera_coordinates_;
  Polar2Cartesian cartesian_coordinates_;
  //keep this last :: synchronizer
  message_filters::Synchronizer<mySyncPolicy> sync_;
  //std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::Image>>> sync_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidarImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
