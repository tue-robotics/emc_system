#include "emc/communication.h"

#include <functional>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace emc
{

    Communication::Communication(std::string /*robot_name*/)
    {
        std::cout << "constructor of Communication" << std::endl;
        rclcpp::init(0, nullptr);

        std::string laser_param, odom_param, bumper_f_param, bumper_b_param, base_ref_param, open_door_param, speak_param, play_param;
        // temp hardcode param names
        laser_param = "scan";
        odom_param = "odometry/filtered";
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

        laser_node_ = std::make_shared<emc::Ros2Subscriber<sensor_msgs::msg::LaserScan>>(laser_param, "emc_laser");
        laser_executor_ = new rclcpp::executors::SingleThreadedExecutor;
        laser_executor_->add_node(laser_node_);

        odom_node_ = std::make_shared<emc::Ros2Subscriber<nav_msgs::msg::Odometry>>(odom_param, "emc_odom");
        odom_executor_ = new rclcpp::executors::SingleThreadedExecutor;
        odom_executor_->add_node(odom_node_);

        pub_node_ = new Ros2Publisher();
    }

    Communication::~Communication()
    {
    }

    void Communication::init()
    {
    }

    bool Communication::readLaserData(LaserData &scan)
    {
        laser_executor_->spin_once(std::chrono::nanoseconds(0)); // wait 0 nanoseconds for new messages. just empty the buffer.

        sensor_msgs::msg::LaserScan msg;
        if (!laser_node_->readMsg(msg))
            return false;

        scan.range_min = msg.range_min;
        scan.range_max = msg.range_max;
        scan.ranges = msg.ranges;
        scan.angle_min = msg.angle_min;
        scan.angle_max = msg.angle_max;
        scan.angle_increment = msg.angle_increment;
        scan.timestamp = rclcpp::Time(msg.header.stamp).seconds();
        return true;
    }

    bool Communication::readOdometryData(OdometryData &odom)
    {
        odom_executor_->spin_once(std::chrono::nanoseconds(0)); // wait 0 nanoseconds for new messages. just empty the buffer.

        nav_msgs::msg::Odometry msg;
        if (!odom_node_->readMsg(msg))
            return false;

        odom.x = msg.pose.pose.position.x;
        odom.y = msg.pose.pose.position.y;

        // Calculate yaw rotation from quaternion
        const geometry_msgs::msg::Quaternion &q = msg.pose.pose.orientation;
        odom.a = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

        odom.timestamp = rclcpp::Time(msg.header.stamp).seconds();

        return true;
    }

    bool Communication::readFrontBumperData(BumperData & /*bumper*/)
    {
        return false;
        /*
        bumper_f_msg_.reset();

        //bumper_f_cb_queue_.callAvailable();

        if (!bumper_f_msg_)
            return false;

        bumper.contact = bumper_f_msg_->data;
        return true;
        */
    }

    bool Communication::readBackBumperData(BumperData & /*bumper*/)
    {
        return false;
        /*
        bumper_b_msg_.reset();

        //bumper_b_cb_queue_.callAvailable();

        if (!bumper_b_msg_)
            return false;

        bumper.contact = bumper_b_msg_->data;
        return true;
        */
    }

    void Communication::sendBaseVelocity(double vx, double vy, double va)
    {
        pub_node_->sendBaseVelocity(vx, vy, va);
    }

    void Communication::sendOpenDoorRequest()
    {
        pub_node_->sendOpenDoorRequest();
    }

    void Communication::sendMarker(visualization_msgs::msg::Marker marker)
    {
        pub_node_->sendMarker(marker);
    }

    void Communication::speak(const std::string &text)
    {
        pub_node_->speak(text);
    }

    void Communication::play(const std::string &file)
    {
        pub_node_->play(file);
    }

    void Communication::sendPoseEstimate(const geometry_msgs::msg::Transform &pose)
    {
        pub_node_->sendPoseEstimate(pose);
    }

    // publishers used to visualize information in the localization exercises (particle filter):

    void Communication::localization_viz_send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction)
    {
        pub_node_->localization_viz_send_laser_scan(angle_min, angle_max, angle_inc, subsample, prediction);
    }

    void Communication::localization_viz_send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation)
    {
        pub_node_->localization_viz_send_particles(N, particle_poses, mapOrientation);
    }

    void Communication::localization_viz_send_pose(std::vector<double> pose, double mapOrientation)
    {
        pub_node_->localization_viz_send_pose(pose, mapOrientation);
    }

} // end namespace emc
