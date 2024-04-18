#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

using namespace std_msgs::msg;
using namespace message_filters;

class SyncNode : public rclcpp::Node
{
    public:
        SyncNode()
        : Node("sync_node")
        {
            // Configure the QoS settings for best effort
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
            qos.best_effort();

            lidar_sub_.subscribe(this, "lidar_topic", qos.get_rmw_qos_profile());
            camera_sub_.subscribe(this, "camera_topic", qos.get_rmw_qos_profile());

            // Exact Time Policy
            exact_sync_ = std::make_shared<TimeSynchronizer<Header, Header>>(lidar_sub_, camera_sub_, 5);
            exact_sync_->registerCallback(std::bind(&SyncNode::exact_callback, this, std::placeholders::_1, std::placeholders::_2));

            // Approximate Time Policy
            typedef sync_policies::ApproximateTime<Header, Header> ApproxPolicy;
            approx_sync_ = std::make_shared<Synchronizer<ApproxPolicy>>( ApproxPolicy(5), lidar_sub_, camera_sub_);
            approx_sync_->registerCallback(std::bind(&SyncNode::approx_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
        
        

    private:

        void approx_callback(const std_msgs::msg::Header::ConstSharedPtr& lidar_msg, const std_msgs::msg::Header::ConstSharedPtr& camera_msg)
        {
            // Log the reception time and the timestamp from the header
            RCLCPP_INFO(this->get_logger(), "Approximate Sync - Lidar: [%s], Camera: [%s], Lidar Time: %ld.%09ld, Camera Time: %ld.%09ld",
                        lidar_msg->frame_id.c_str(), camera_msg->frame_id.c_str(),
                        lidar_msg->stamp.sec, lidar_msg->stamp.nanosec,
                        camera_msg->stamp.sec, camera_msg->stamp.nanosec);
        }
        
        void exact_callback(const std_msgs::msg::Header::ConstSharedPtr& lidar_msg, const std_msgs::msg::Header::ConstSharedPtr& camera_msg)
        {
            // Log the reception time and the timestamp from the header
            RCLCPP_INFO(this->get_logger(), "Exact Sync - Lidar: [%s], Camera: [%s], Lidar Time: %ld.%09ld, Camera Time: %ld.%09ld",
                        lidar_msg->frame_id.c_str(), camera_msg->frame_id.c_str(),
                        lidar_msg->stamp.sec, lidar_msg->stamp.nanosec,
                        camera_msg->stamp.sec, camera_msg->stamp.nanosec);
        }
        message_filters::Subscriber<Header> lidar_sub_;
        message_filters::Subscriber<Header> camera_sub_;
        std::shared_ptr<TimeSynchronizer<Header, Header>> exact_sync_;
        typedef sync_policies::ApproximateTime<Header, Header> mySyncPolicy;
        std::shared_ptr<Synchronizer<sync_policies::ApproximateTime<Header, Header>>> approx_sync_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncNode>());
    rclcpp::shutdown();
    return 0;
}
