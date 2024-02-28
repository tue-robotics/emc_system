#include "emc/communication.h"

//#include <ros/node_handle.h>
//#include <ros/subscribe_options.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <string>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Communication::Communication(std::string /*robot_name*/) : rclcpp::Node("emc_system")
{
/*
    ros::VP_string args;
    ros::init(args, "emc_system", ros::init_options::AnonymousName);
    ros::Time::init();
*/
  //  ros::NodeHandle nh;
    std::string laser_param, odom_param, bumper_f_param, bumper_b_param, base_ref_param, open_door_param, speak_param, play_param;
    // temp hardcode param names
    laser_param = "laser_scan";
    odom_param = "odom";
    bumper_f_param = "bumper_f";
    bumper_b_param = "bumper_b";
    base_ref_param = "cmd_vel";
    open_door_param = "open_door";
    speak_param = "speak";
    play_param = "play";
    /*
    // get robot parameters
    if (!nh.getParam("laser_", laser_param)) {ROS_ERROR_STREAM("Parameter " << "laser_" << " not set");};
    if (!nh.getParam("odom_", odom_param)) {ROS_ERROR_STREAM("Parameter " << "odom_" << " not set");};
    if (!nh.getParam("bumper_f_", bumper_f_param)) {ROS_ERROR_STREAM("Parameter " << "bumper_f_" << " not set");};
    if (!nh.getParam("bumper_b_", bumper_b_param)) {ROS_ERROR_STREAM("Parameter " << "bumper_b_" << " not set");};
    if (!nh.getParam("base_ref_", base_ref_param)) {ROS_ERROR_STREAM("Parameter " << "base_ref_" << " not set");};
    if (!nh.getParam("open_door_", open_door_param)) {ROS_ERROR_STREAM("Parameter " << "open_door_" << " not set");};
    if (!nh.getParam("speak_", speak_param)) {ROS_ERROR_STREAM("Parameter " << "speak_" << " not set");};
    if (!nh.getParam("play_", play_param)) {ROS_ERROR_STREAM("Parameter " << "play_" << " not set");};
    if (!nh.getParam("base_link_", robot_frame_name)) {ROS_ERROR_STREAM("Parameter " << "base_link_" << " not set");};
*/
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_param, 10, std::bind(&Communication::laserCallback, this, _1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_param, 10, std::bind(&Communication::odomCallback, this, _1));
    sub_bumper_f_ = this->create_subscription<std_msgs::msg::Bool>(bumper_f_param, 10, std::bind(&Communication::bumperfCallback, this, _1));
    sub_bumper_b_ = this->create_subscription<std_msgs::msg::Bool>(bumper_b_param, 10, std::bind(&Communication::bumperbCallback, this, _1));

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(base_ref_param, 10);
    pub_open_door_ = this->create_publisher<std_msgs::msg::Empty>(open_door_param, 10);
    pub_speak_ = this->create_publisher<std_msgs::msg::String>(speak_param, 10);
    pub_play_ = this->create_publisher<std_msgs::msg::String>(play_param, 10);
    pub_marker_ = this->create_publisher<std_msgs::msg::String>("/marker", 10);

/*
    ros::SubscribeOptions laser_sub_options = ros::SubscribeOptions::create<sensor_msgs::LaserScan>(laser_param, 1, boost::bind(&Communication::laserCallback, this, _1), ros::VoidPtr(), &laser_cb_queue_);
    sub_laser_ = nh.subscribe(laser_sub_options);

    ros::SubscribeOptions odom_sub_options = ros::SubscribeOptions::create<nav_msgs::Odometry>(odom_param, 1, boost::bind(&Communication::odomCallback, this, _1), ros::VoidPtr(), &odom_cb_queue_);
    sub_odom_ = nh.subscribe(odom_sub_options);

    ros::SubscribeOptions bumper_f_sub_options = ros::SubscribeOptions::create<std_msgs::Bool>(bumper_f_param, 1, boost::bind(&Communication::bumperfCallback, this, _1), ros::VoidPtr(), &bumper_f_cb_queue_);
    sub_bumper_f_ = nh.subscribe(bumper_f_sub_options);

    ros::SubscribeOptions bumper_b_sub_options = ros::SubscribeOptions::create<std_msgs::Bool>(bumper_b_param, 1, boost::bind(&Communication::bumperbCallback, this, _1), ros::VoidPtr(), &bumper_b_cb_queue_);
    sub_bumper_b_ = nh.subscribe(bumper_b_sub_options);

    pub_base_ref_ = nh.advertise<geometry_msgs::Twist>(base_ref_param, 1);

    pub_open_door_ = nh.advertise<std_msgs::Empty>(open_door_param, 1);

    pub_speak_ = nh.advertise<std_msgs::String>(speak_param, 1);
    pub_play_ = nh.advertise<std_msgs::String>(play_param, 1);

    pub_marker_ = nh.advertise<visualization_msgs::Marker>("/marker", 1);
*/
    pub_tf2 = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
}

// ----------------------------------------------------------------------------------------------------

Communication::~Communication()
{
    
}

// ----------------------------------------------------------------------------------------------------

void Communication::init()
{
}

// ----------------------------------------------------------------------------------------------------

bool Communication::readLaserData(LaserData& scan)
{
    laser_msg_.reset();
    //laser_cb_queue_.callAvailable();

    if (!laser_msg_)
        return false;

    scan.range_min = laser_msg_->range_min;
    scan.range_max = laser_msg_->range_max;
    scan.ranges = laser_msg_->ranges;
    scan.angle_min = laser_msg_->angle_min;
    scan.angle_max = laser_msg_->angle_max;
    scan.angle_increment = laser_msg_->angle_increment;
    scan.timestamp = laser_msg_->header.stamp.toSec();

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Communication::readOdometryData(OdometryData& odom)
{
    odom_msg_.reset();
    //odom_cb_queue_.callAvailable();

    if (!odom_msg_)
        return false;

    odom.x = odom_msg_->pose.pose.position.x;
    odom.y = odom_msg_->pose.pose.position.y;

    // Calculate yaw rotation from quaternion
    const geometry_msgs::Quaternion& q = odom_msg_->pose.pose.orientation;
    odom.a = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

    odom.timestamp = odom_msg_->header.stamp.toSec();

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Communication::readFrontBumperData(BumperData& bumper)
{
    bumper_f_msg_.reset();

    //bumper_f_cb_queue_.callAvailable();

    if (!bumper_f_msg_)
        return false;

    bumper.contact = bumper_f_msg_->data;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Communication::readBackBumperData(BumperData& bumper)
{
    bumper_b_msg_.reset();

    //bumper_b_cb_queue_.callAvailable();

    if (!bumper_b_msg_)
        return false;

    bumper.contact = bumper_b_msg_->data;
    return true;
}

// ----------------------------------------------------------------------------------------------------

void Communication::sendBaseVelocity(double vx, double vy, double va)
{
    geometry_msgs::msg::Twist ref;
    ref.linear.x = vx;
    ref.linear.y = vy;
    ref.angular.z = va;

    pub_cmd_vel_->publish(ref);
}

// ----------------------------------------------------------------------------------------------------

void Communication::sendOpendoorRequest()
{
    std_msgs::msg::Empty msg;
    pub_open_door_->publish(msg);
}

// ----------------------------------------------------------------------------------------------------

void Communication::sendMarker(visualization_msgs::msg::Marker marker)
{
    pub_marker_->publish(marker);
}

// ----------------------------------------------------------------------------------------------------

void Communication::speak(const std::string& text)
{
    std_msgs::msg::String str;
    str.data = text;
    pub_speak_->publish(str);
}

void Communication::play(const std::string& file)
{
    std_msgs::msg::String str;
    str.data = file;
    pub_play_->publish(str);
}

void Communication::sendPoseEstimate(const geometry_msgs::msg::Transform& pose)
{
    // Publish tf transform
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = robot_frame_name;
    transformStamped.transform = pose;
    pub_tf2->sendTransform(transformStamped);
}

// ----------------------------------------------------------------------------------------------------

void Communication::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::bumperfCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bumper_f_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::bumperbCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bumper_b_msg_ = msg;
}

} // end namespace emc

