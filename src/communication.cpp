#include "emc/communication.h"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace emc
{

    Communication::Communication(std::string /*robot_name*/)
    {
        // std::cout << "constructor of Communication" << std::endl;
        rclcpp::init(0, nullptr);

        auto node = std::make_shared<rclcpp::Node>("communication_node");
        auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/global_parameter_server");
        
        // Wait for the parameter server to be available
        while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for the parameter service to be available...");
        }

        // Get parameters from the global parameter server
        auto parameters_future = parameters_client->get_parameters(
            {"laser_", "odom_", "bumper_f_", "bumper_b_", "base_ref_", "open_door_", "speak_", "play_", "base_link_"},
            [this, node](std::shared_future<std::vector<rclcpp::Parameter>> future) {
                auto result = future.get();
                std::map<std::string, std::string> params;
                for (const auto &param : result) {
                    params[param.get_name()] = param.as_string();
                }

                std::string laser_param = params["laser_"];
                std::string odom_param = params["odom_"];
                std::string bumper_f_param = params["bumper_f_"];
                std::string bumper_b_param = params["bumper_b_"];
                std::string base_ref_param = params["base_ref_"];
                std::string open_door_param = params["open_door_"];
                std::string speak_param = params["speak_"];
                std::string play_param = params["play_"];
                std::string base_link_param = params["base_link_"];

                RCLCPP_INFO(node->get_logger(), "Got parameters from global parameter server");

                // Initialize nodes with the retrieved parameters
                laser_node_ = std::make_shared<emc::Ros2Subscriber<sensor_msgs::msg::LaserScan>>(laser_param, "emc_laser");
                laser_executor_ = new rclcpp::executors::SingleThreadedExecutor;
                laser_executor_->add_node(laser_node_);

                odom_node_ = std::make_shared<emc::Ros2Subscriber<nav_msgs::msg::Odometry>>(odom_param, "emc_odom");
                odom_executor_ = new rclcpp::executors::SingleThreadedExecutor;
                odom_executor_->add_node(odom_node_);

                pub_node_ = new Ros2Publisher();
            });

        // Spin the node to process the parameter retrieval callback
        rclcpp::spin_some(node);
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
