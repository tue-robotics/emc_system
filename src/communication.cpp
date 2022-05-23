#include "emc/communication.h"

#include <ros/node_handle.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
//#include <emc_system/controlEffort.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Range.h>

#include <math.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Communication::Communication(std::string robot_name)
{

    ros::VP_string args;
    ros::init(args, "emc_system", ros::init_options::AnonymousName);
    ros::Time::init();

    ros::NodeHandle nh_laser;
    nh_laser.setCallbackQueue(&laser_cb_queue_);
    sub_laser_ = nh_laser.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Communication::laserCallback, this);

    ros::NodeHandle nh_odom;
    nh_odom.setCallbackQueue(&odom_cb_queue_);
    sub_odom_ = nh_odom.subscribe<nav_msgs::Odometry>("/odom", 1, &Communication::odomCallback, this);

    /*
    ros::NodeHandle nh_ce;
    nh_ce.setCallbackQueue(&ce_cb_queue_);
    sub_ce_ = nh_ce.subscribe<emc_system::controlEffort>("/" + robot_name + "/controlEffort", 1, &Communication::controlEffortCallback, this);
*/

    ros::NodeHandle nh_bumper_fl;
    nh_bumper_fl.setCallbackQueue(&bumper_fl_cb_queue_);
    sub_bumper_fl_ = nh_bumper_fl.subscribe<sensor_msgs::Range>("/range/fl", 1, &Communication::bumperflCallback, this);
    
    ros::NodeHandle nh_bumper_fr;
    nh_bumper_fr.setCallbackQueue(&bumper_fr_cb_queue_);
    sub_bumper_fr_ = nh_bumper_fr.subscribe<sensor_msgs::Range>("/range/fr", 1, &Communication::bumperfrCallback, this);

     ros::NodeHandle nh_bumper_bl;
    nh_bumper_bl.setCallbackQueue(&bumper_bl_cb_queue_);
    sub_bumper_bl_ = nh_bumper_bl.subscribe<sensor_msgs::Range>("/range/bl", 1, &Communication::bumperblCallback, this);
    
    ros::NodeHandle nh_bumper_br;
    nh_bumper_br.setCallbackQueue(&bumper_br_cb_queue_);
    sub_bumper_br_ = nh_bumper_br.subscribe<sensor_msgs::Range>("/range/br", 1, &Communication::bumperbrCallback, this);

    pub_base_ref_ = nh_laser.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    pub_open_door_ = nh_laser.advertise<std_msgs::Empty>("/" + robot_name + "/open_door", 1);

    pub_speak_ = nh_laser.advertise<std_msgs::String>("/" + robot_name + "/text_to_speech/input", 1);
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
    scan.angle_min = laser_msg_->angle_min + M_PI;
    scan.angle_max = laser_msg_->angle_max + M_PI;
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
    bumper_fl_msg_.reset();
    bumper_fr_msg_.reset();

    bumper_fl_cb_queue_.callAvailable();
    bumper_fr_cb_queue_.callAvailable();

    if (!bumper_fl_msg_ && !bumper_fr_msg_)
        return false;

    bumper.contact = bumper_fl_msg_->range < 1; 
    bumper.contact = bumper.contact || bumper_fr_msg_->range < 1 ;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Communication::readBackBumperData(BumperData& bumper)
{
    bumper_bl_msg_.reset();
    bumper_br_msg_.reset();

    bumper_bl_cb_queue_.callAvailable();
    bumper_br_cb_queue_.callAvailable();

    if (!bumper_bl_msg_ && !bumper_br_msg_)
        return false;

    bumper.contact = bumper_bl_msg_->range < 1; 
    bumper.contact = bumper.contact || bumper_br_msg_->range < 1 ;
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

void Communication::bumperflCallback(const sensor_msgs::RangeConstPtr& msg)
{
    bumper_fl_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::bumperfrCallback(const sensor_msgs::RangeConstPtr& msg)
{
    bumper_fr_msg_ = msg;
}


// ----------------------------------------------------------------------------------------------------

void Communication::bumperblCallback(const sensor_msgs::RangeConstPtr& msg)
{
    bumper_bl_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::bumperbrCallback(const sensor_msgs::RangeConstPtr& msg)
{
    bumper_br_msg_ = msg;
}


// ----------------------------------------------------------------------------------------------------
/*
void Communication::controlEffortCallback(const emc_system::controlEffortConstPtr& msg)
{
    ce_msg_ = msg;
}
*/

} // end namespace emc

