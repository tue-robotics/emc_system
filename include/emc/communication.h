#ifndef EMC_COMMUNICATION_H_
#define EMC_COMMUNICATION_H_

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/bumper.h"

#include "rclcpp/rclcpp.hpp"
//#include <ros/callback_queue.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nav_msgs/msg/map_meta_data.hpp"

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <memory>

namespace emc
{

class Communication : public rclcpp::Node
{

public:

    Communication(std::string robot_name="pyro");

    ~Communication();

    void init();

    bool readLaserData(LaserData& scan);

    bool readOdometryData(OdometryData& odom);

    bool readFrontBumperData(BumperData& bumper);
    bool readBackBumperData(BumperData& bumper);

    void sendBaseVelocity(double vx, double vy, double va);

    void sendOpendoorRequest();

    void sendMarker(visualization_msgs::msg::Marker marker);

    void speak(const std::string& text);
    
    void play(const std::string& file);

    // Postion data
    void sendPoseEstimate(const geometry_msgs::msg::Transform& pose);

private:

    // Base velocity reference
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_ref_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_open_door_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_speak_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_play_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    // Position data

    std::unique_ptr<tf2_ros::TransformBroadcaster> pub_tf2; //has to be defined after ros::init(), which is called in the constructor


    // Laser data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;

    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);


    // Odometry data

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Bumper data
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bumper_f_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bumper_b_;

    std_msgs::msg::Bool::SharedPtr bumper_f_msg_;
    std_msgs::msg::Bool::SharedPtr bumper_b_msg_;

    void bumperfCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void bumperbCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // pose publishing
    std::string robot_frame_name;
};

} // end namespace emc

#endif
