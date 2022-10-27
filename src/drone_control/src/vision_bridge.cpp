#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/companion_process_status.hpp"

using namespace std::chrono_literals;
typedef mavros_msgs::msg::CompanionProcessStatus MAV_STATE;

class VisionBridge : public rclcpp::Node
{
  public:
    VisionBridge()
    : Node("vision_bridge"){

        last_callback_time_ = this->now();

        // subscribers
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/drone/realsense/pose/sample", sensor_qos, std::bind(&VisionBridge::OdomInCallback, this, std::placeholders::_1));
        
        // publishers
        mavros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/drone/mavros/odometry/out", 10);
        mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>("/drone/mavros/companion_process/status", 1);

        // call publisher once in a while
        timer_ = this->create_wall_timer(1000ms, std::bind(&VisionBridge::PublishSystemStatus, this));

        RCLCPP_INFO(this->get_logger(), "Vision bridge started");
    }

  private:
    void PublishSystemStatus(){
        // only send heartbeat after first pose has been recieved
        if(!flag_first_pose_received_)
            return;

        // Verify new odom data is coming consistently
        if((this->now()-last_callback_time_) > rclcpp::Duration(500ms) ){
            RCLCPP_WARN_STREAM(this->get_logger(), "Stopped receiving data from T265");
            system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
        }

        mavros_msgs::msg::CompanionProcessStatus status_msg;
        status_msg.header.stamp = this->now();
        status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        status_msg.state = system_status_;

        mavros_system_status_pub_->publish(status_msg);
    }

    void OdomInCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        // run at ~50Hz (odom data is 200Hz)
        decimation_counter_++;
        decimation_counter_ %= 4;
        if(decimation_counter_ != 0)
            return;

        flag_first_pose_received_ = true;
        last_callback_time_ = this->now();
        last_system_status_ = system_status_;


        // forward odometry
        mavros_odom_pub_->publish(*msg);

        // Update companion computer status
        // check confidence in vision estimate by looking at covariance
        if(msg->pose.covariance[0] > 0.1 ) // low confidence -> reboot companion
            system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
        else if(msg->pose.covariance[0] == 0.1 ) // medium confidence
            system_status_ = MAV_STATE::MAV_STATE_CRITICAL;
        else if(msg->pose.covariance[0] == 0.01 ) // high confidence
            system_status_ = MAV_STATE::MAV_STATE_ACTIVE;
        else
            RCLCPP_WARN_STREAM(this->get_logger(), "Unexpected vision sensor variance");

        // publish system status immediately if it changed
        if(last_system_status_ != system_status_)
            PublishSystemStatus();

    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;
    rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    MAV_STATE::_state_type system_status_ = MAV_STATE::MAV_STATE_UNINIT;
    MAV_STATE::_state_type last_system_status_ = MAV_STATE::MAV_STATE_UNINIT;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_callback_time_;

    bool flag_first_pose_received_ = false;
    uint8_t decimation_counter_ = 0;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBridge>());
    rclcpp::shutdown();
    return 0;
}