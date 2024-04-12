#include "emc/communication.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Communication::Communication(std::string /*robot_name*/)
{

    ros::VP_string args;
    ros::init(args, "emc_system", ros::init_options::AnonymousName);
    ros::Time::init();

    ros::NodeHandle nh;
    std::string laser_param, pose_param, odom_param, bumper_f_param, bumper_b_param, base_ref_param, open_door_param, speak_param, play_param;
    if (!nh.getParam("laser_", laser_param)) {ROS_ERROR_STREAM("Parameter " << "laser_" << " not set");};
    if (!nh.getParam("pose_", pose_param)) {ROS_ERROR_STREAM("Parameter " << "pose_" << " not set");};
    if (!nh.getParam("odom_", odom_param)) {ROS_ERROR_STREAM("Parameter " << "odom_" << " not set");};
    if (!nh.getParam("bumper_f_", bumper_f_param)) {ROS_ERROR_STREAM("Parameter " << "bumper_f_" << " not set");};
    if (!nh.getParam("bumper_b_", bumper_b_param)) {ROS_ERROR_STREAM("Parameter " << "bumper_b_" << " not set");};
    if (!nh.getParam("base_ref_", base_ref_param)) {ROS_ERROR_STREAM("Parameter " << "base_ref_" << " not set");};
    if (!nh.getParam("open_door_", open_door_param)) {ROS_ERROR_STREAM("Parameter " << "open_door_" << " not set");};
    if (!nh.getParam("speak_", speak_param)) {ROS_ERROR_STREAM("Parameter " << "speak_" << " not set");};
    if (!nh.getParam("play_", play_param)) {ROS_ERROR_STREAM("Parameter " << "play_" << " not set");};
    if (!nh.getParam("base_link_", robot_frame_name)) {ROS_ERROR_STREAM("Parameter " << "base_link_" << " not set");};

    ros::SubscribeOptions laser_sub_options = ros::SubscribeOptions::create<sensor_msgs::LaserScan>(laser_param, 1, boost::bind(&Communication::laserCallback, this, _1), ros::VoidPtr(), &laser_cb_queue_);
    sub_laser_ = nh.subscribe(laser_sub_options);

    ros::SubscribeOptions pose_sub_options = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(pose_param, 1, boost::bind(&Communication::poseCallback, this, _1), ros::VoidPtr(), &pose_cb_queue_);
    sub_pose_ = nh.subscribe(pose_sub_options);

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

    pub_tf2 = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);


    // publishers used to visualize information in the localization exercises (particle filter):
    localization_visualization_pub_laser_msg_ = nh.advertise<sensor_msgs::LaserScan>("/laser_match", 1);
    localization_visualization_pub_particle_ = nh.advertise<geometry_msgs::PoseArray>("/particles", 1);
    localization_visualization_pub_pose_ = nh.advertise<geometry_msgs::PoseArray>("/pose_estimate", 1);
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

bool Communication::readPoseData(PoseData& pose)
{
    pose_msg_.reset();
    pose_cb_queue_.callAvailable();

    if (!pose_msg_)
        return false;
    
    // Store position. Optitrack swaps the y and z axis,
    // but to use the convention of rviz, we swap them back
    pose.x = pose_msg_->pose.position.x;
    pose.y = pose_msg_->pose.position.z;
    pose.z = pose_msg_->pose.position.y;

    // Convert quaternion to roll, pitch, yaw
    // Again, y and z are swapped to match rviz
    tf2::Quaternion q(pose_msg_->pose.orientation.x,
                      pose_msg_->pose.orientation.z,
                      pose_msg_->pose.orientation.y,
                      -pose_msg_->pose.orientation.w);
    
    tf2::Matrix3x3 T(q);

    double roll, pitch, yaw;
    T.getRPY(roll, pitch, yaw);

    // Store orientation
    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;

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

void Communication::sendMarker(visualization_msgs::Marker marker)
{
    pub_marker_.publish(marker);
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

void Communication::sendPoseEstimate(const geometry_msgs::Transform& pose)
{
    // Publish tf transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = robot_frame_name;
    transformStamped.transform = pose;
    pub_tf2->sendTransform(transformStamped);
}

// ----------------------------------------------------------------------------------------------------

void Communication::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Communication::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    pose_msg_ = msg;
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
// publishing functions used to visualize information in the localization exercises (particle filter):

void Communication::localization_viz_send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction)
{
    sensor_msgs::LaserScan msg{};

    msg.angle_min = angle_min;
    msg.angle_max = angle_max;
    msg.angle_increment = angle_inc * subsample;

    msg.range_min = 0.01;
    msg.range_max = 10;

    msg.header.frame_id = "internal/base_link";
    msg.header.stamp = ros::Time::now();

    msg.ranges = prediction;

    this->localization_visualization_pub_laser_msg_.publish(msg); //make correct publisher
    // pub_node_->send_laser_scan(angle_min, angle_max, angle_inc, subsample, prediction);
}

void Communication::localization_viz_send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation)
{
    geometry_msgs::PoseArray msg{};

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    tf2::Quaternion a;
    msg.poses.reserve(N);
    for (int i = 0; i < N; i++)
    {
        geometry_msgs::Pose posemsg;
        auto pose_i = particle_poses[i];

        posemsg.position.x = std::cos(mapOrientation) * pose_i[0] - std::sin(mapOrientation) * pose_i[1];
        posemsg.position.y = std::sin(mapOrientation) * pose_i[0] + std::cos(mapOrientation) * pose_i[1];
        posemsg.position.z = 0;

        a.setRPY(0, 0, pose_i[2] - mapOrientation);
        posemsg.orientation.w = a.getW();
        posemsg.orientation.x = a.getX();
        posemsg.orientation.y = a.getY();
        posemsg.orientation.z = a.getZ();

        msg.poses.push_back(posemsg);
    }

    this->localization_visualization_pub_particle_.publish(msg);
}

void Communication::localization_viz_send_pose(std::vector<double> pose, double mapOrientation)
{
    geometry_msgs::PoseArray msg;

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    tf2::Quaternion a;

    msg.poses.reserve(1);

    geometry_msgs::Pose posemsg;
    posemsg.position.x = std::cos(mapOrientation) * pose[0] - std::sin(mapOrientation) * pose[1];
    posemsg.position.y = std::sin(mapOrientation) * pose[0] + std::cos(mapOrientation) * pose[1];
    posemsg.position.z = 0;

    a.setRPY(0, 0, pose[2] - mapOrientation);
    posemsg.orientation.w = a.getW();
    posemsg.orientation.x = a.getX();
    posemsg.orientation.y = a.getY();
    posemsg.orientation.z = a.getZ();

    msg.poses.push_back(posemsg);

    this->localization_visualization_pub_pose_.publish(msg);
}

} // end namespace emc


