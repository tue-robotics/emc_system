#include "emc/communication.h"

#include <ros/node_handle.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#include <signal.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------
void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    std::cout << "Stopping robot.\n";

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/pico/base/reference", 1);
    geometry_msgs::Twist ref;
    ref.linear.x = 0;
    ref.linear.y = 0;
    ref.angular.z = 0;
    pub.publish(ref);
   // cleanup and close up stuff here
   // terminate program
    ros::shutdown();

    exit(signum);
}

// ----------------------------------------------------------------------------------------------------

Communication::Communication()
{

    ros::VP_string args;
    ros::init(args, "emc_system", ros::init_options::AnonymousName);
    ros::Time::init();

    ros::NodeHandle nh_laser;
    nh_laser.setCallbackQueue(&laser_cb_queue_);
    sub_laser_ = nh_laser.subscribe<sensor_msgs::LaserScan>("/pico/laser", 1, &Communication::laserCallback, this);

    ros::NodeHandle nh_odom;
    nh_odom.setCallbackQueue(&odom_cb_queue_);
    sub_odom_ = nh_odom.subscribe<nav_msgs::Odometry>("/pico/odom", 1, &Communication::odomCallback, this);

    pub_base_ref_ = nh_laser.advertise<geometry_msgs::Twist>("/pico/base/reference", 1);

    pub_open_door_ = nh_laser.advertise<std_msgs::Empty>("/pico/open_door", 1);

    signal(SIGINT, signalHandler);
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

void Communication::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_msg_ = msg;
}

} // end namespace emc

