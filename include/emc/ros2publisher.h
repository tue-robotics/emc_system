#ifndef EMC_ROS2PUBLISHER_H_
#define EMC_ROS2PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace emc
{

/**
 * collection of all publishers used in the emc system
*/
class Ros2Publisher : public rclcpp::Node
{

public:

    Ros2Publisher() : rclcpp::Node("emc_publishers")
    {
        std::string laser_param, odom_param, bumper_f_param, bumper_b_param, base_ref_param, open_door_param, speak_param, play_param;
        laser_param = "laser_scan";
        odom_param = "odom";
        bumper_f_param = "bumper_f";
        bumper_b_param = "bumper_b";
        base_ref_param = "cmd_vel";
        open_door_param = "open_door";
        speak_param = "speak";
        play_param = "play";

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(base_ref_param, 10);
        pub_open_door_ = this->create_publisher<std_msgs::msg::Empty>(open_door_param, 10);
        pub_speak_ = this->create_publisher<std_msgs::msg::String>(speak_param, 10);
        pub_play_ = this->create_publisher<std_msgs::msg::String>(play_param, 10);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);

        pub_tf2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    };

    void sendBaseVelocity(double vx, double vy, double va)
    {
        geometry_msgs::msg::Twist ref;
        ref.linear.x = vx;
        ref.linear.y = vy;
        ref.angular.z = va;

        pub_cmd_vel_->publish(ref);
    };

    void sendOpendoorRequest()
    {
        std_msgs::msg::Empty msg;
        pub_open_door_->publish(msg);
    };

    void sendMarker(visualization_msgs::msg::Marker marker)
    {
        pub_marker_->publish(marker);
    };

    void speak(const std::string& text)
    {
        std_msgs::msg::String str;
        str.data = text;
        pub_speak_->publish(str);
    };
    
    void play(const std::string& file)
    {
        std_msgs::msg::String str;
        str.data = file;
        pub_play_->publish(str);
    };

    // Postion data
    void sendPoseEstimate(const geometry_msgs::msg::Transform& pose)
    {
        // Publish tf transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now(); //rclcpp::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = robot_frame_name;
        transformStamped.transform = pose;
        pub_tf2_->sendTransform(transformStamped);
    };


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_open_door_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_speak_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_play_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    // pose publishing
    std::string robot_frame_name;
    std::unique_ptr<tf2_ros::TransformBroadcaster> pub_tf2_;
};

} // end namespace emc

#endif

   