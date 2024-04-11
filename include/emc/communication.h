#ifndef EMC_COMMUNICATION_H_
#define EMC_COMMUNICATION_H_

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/bumper.h"
#include "emc/ros2publisher.h"
#include "emc/ros2subscriber.h"

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

class Communication
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

    void sendOpenDoorRequest();

    void sendMarker(visualization_msgs::msg::Marker marker);

    void speak(const std::string& text);
    
    void play(const std::string& file);

    // Postion data
    void sendPoseEstimate(const geometry_msgs::msg::Transform& pose);

    bool sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color, double width, int id)
    {
        return pub_node_->sendPath(path, color, width, id);
    }

    // publishers used to visualize information in the localization exercises (particle filter):
    void send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction);
    void send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation);
    void send_pose(std::vector<double> pose, double mapOrientation);

private:
    Ros2Publisher* pub_node_;

    std::shared_ptr<emc::Ros2Subscriber<sensor_msgs::msg::LaserScan>> laser_node_;
    rclcpp::executors::SingleThreadedExecutor* laser_executor_;

    std::shared_ptr<emc::Ros2Subscriber<nav_msgs::msg::Odometry>> odom_node_;
    rclcpp::executors::SingleThreadedExecutor* odom_executor_;
};

} // end namespace emc

#endif
