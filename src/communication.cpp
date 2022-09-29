#include "emc/communication.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

//#include <emc_system/controlEffort.h

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Communication::Communication(std::string robot_name)
{

    ros::VP_string args;
    ros::init(args, "emc_system", ros::init_options::AnonymousName);
    ros::Time::init();

    ros::NodeHandle nh;
    ros::SubscribeOptions laser_sub_options = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/transformed_scan", 1, boost::bind(&Communication::laserCallback, this, _1), ros::VoidPtr(), &laser_cb_queue_);
    sub_laser_ = nh.subscribe(laser_sub_options);

    ros::SubscribeOptions odom_sub_options = ros::SubscribeOptions::create<nav_msgs::Odometry>("/odom", 1, boost::bind(&Communication::odomCallback, this, _1), ros::VoidPtr(), &odom_cb_queue_);
    sub_odom_ = nh.subscribe(odom_sub_options);

    /*
    ros::SubscribeOptions ce_sub_options = ros::SubscribeOptions::create<emc_system::controlEffort>("/" + robot_name + "/controlEffort", 1, boost::bind(&Communication::controlEffortCallback, this, _1), ros::VoidPtr(), &ce_cb_queue_);
    sub_ce_ = nh.subscribe(ce_sub_options);
    */

    ros::SubscribeOptions bumper_f_sub_options = ros::SubscribeOptions::create<std_msgs::Bool>("/" + robot_name + "/base_f_bumper_sensor", 1, boost::bind(&Communication::bumperfCallback, this, _1), ros::VoidPtr(), &bumper_f_cb_queue_);
    sub_bumper_f_ = nh.subscribe(bumper_f_sub_options);

    ros::SubscribeOptions bumper_b_sub_options = ros::SubscribeOptions::create<std_msgs::Bool>("/" + robot_name + "/base_b_bumper_sensor", 1, boost::bind(&Communication::bumperbCallback, this, _1), ros::VoidPtr(), &bumper_b_cb_queue_);
    sub_bumper_b_ = nh.subscribe(bumper_b_sub_options);

    pub_base_ref_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    pub_open_door_ = nh.advertise<std_msgs::Empty>("/" + robot_name + "/open_door", 1);

    pub_speak_ = nh.advertise<std_msgs::String>("/" + robot_name + "/text_to_speech/input", 1);
    pub_play_ = nh.advertise<std_msgs::String>("/" + robot_name + "/text_to_speech/file", 1);
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
    laser_cb_queue_.callAvailable();

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
    odom_cb_queue_.callAvailable();

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

    bumper_f_cb_queue_.callAvailable();

    if (!bumper_f_msg_)
        return false;

    bumper.contact = bumper_f_msg_->data;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Communication::readBackBumperData(BumperData& bumper)
{
    bumper_b_msg_.reset();

    bumper_b_cb_queue_.callAvailable();

    if (!bumper_b_msg_)
        return false;

    bumper.contact = bumper_b_msg_->data;
    return true;
}

// ----------------------------------------------------------------------------------------------------
/*
bool Communication::readControlEffort(ControlEffort& ce)
{
    ce_msg_.reset();
    ce_cb_queue_.callAvailable();

    if (!ce_msg_)
        return false;

    ce.x  = ce_msg_->I_x;
    ce.y  = ce_msg_->I_y;
    ce.th = ce_msg_->I_th;

    ce.timestamp = ce_msg_->header.stamp.toSec();

    return true;
}
*/

// ----------------------------------------------------------------------------------------------------

void Communication::sendBaseVelocity(double vx, double vy, double va)
{
    geometry_msgs::Twist ref;
    ref.linear.x = vx;
    ref.linear.y = vy;
    ref.angular.z = va;

    pub_base_ref_.publish(ref);
}

// ----------------------------------------------------------------------------------------------------

void Communication::sendOpendoorRequest()
{
    std_msgs::Empty msg;
    pub_open_door_.publish(msg);
}

// ----------------------------------------------------------------------------------------------------

void Communication::speak(const std::string& text)
{
    std_msgs::String str;
    str.data = text;
    pub_speak_.publish(str);
}

void Communication::play(const std::string& file)
{
    std_msgs::String str;
    str.data = file;
    pub_play_.publish(str);
}

// ----------------------------------------------------------------------------------------------------

void Communication::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::bumperfCallback(const std_msgs::BoolConstPtr& msg)
{
    bumper_f_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::bumperbCallback(const std_msgs::BoolConstPtr& msg)
{
    bumper_b_msg_ = msg;
}
// ----------------------------------------------------------------------------------------------------
/*
void Communication::controlEffortCallback(const emc_system::controlEffortConstPtr& msg)
{
    ce_msg_ = msg;
}
*/

} // end namespace emc

