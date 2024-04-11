#ifndef EMC_COMMUNICATION_H_
#define EMC_COMMUNICATION_H_

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/pose.h"
#include "emc/bumper.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/MapMetaData.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
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

    bool readPoseData(PoseData& pose);

    bool readOdometryData(OdometryData& odom);

    bool readFrontBumperData(BumperData& bumper);
    bool readBackBumperData(BumperData& bumper);


    void sendBaseVelocity(double vx, double vy, double va);

    void sendOpendoorRequest();

    void sendMarker(visualization_msgs::Marker marker);

    void speak(const std::string& text);
    
    void play(const std::string& file);

    // Postion data
    void sendPoseEstimate(const geometry_msgs::Transform& pose);

    // publishing functions used to visualize information in the localization exercises (particle filter):
    void send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction);
    void send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation);
    void send_pose(std::vector<double> pose, double mapOrientation);

private:

    // Base velocity reference

    ros::Publisher pub_base_ref_;

    ros::Publisher pub_open_door_;

    ros::Publisher pub_speak_;
    
    ros::Publisher pub_play_;

    ros::Publisher pub_marker_;

    // publishers used to visualize information in the localization exercises (particle filter):
    ros::Publisher localization_visualization_pub_laser_msg_;
    ros::Publisher localization_visualization_pub_particle_;
    ros::Publisher localization_visualization_pub_pose_;

    // Position data

    std::unique_ptr<tf2_ros::TransformBroadcaster> pub_tf2; //has to be defined after ros::init(), which is called in the constructor


    // Laser data

    ros::CallbackQueue laser_cb_queue_;

    ros::Subscriber sub_laser_;

    sensor_msgs::LaserScanConstPtr laser_msg_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    // Pose data

    ros::CallbackQueue pose_cb_queue_;

    ros::Subscriber sub_pose_;

    geometry_msgs::PoseStampedConstPtr pose_msg_;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    // Odometry data

    ros::CallbackQueue odom_cb_queue_;

    ros::Subscriber sub_odom_;

    nav_msgs::OdometryConstPtr odom_msg_;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    // Bumper data

    ros::CallbackQueue bumper_f_cb_queue_;
    ros::CallbackQueue bumper_b_cb_queue_;

    ros::Subscriber sub_bumper_f_;
    ros::Subscriber sub_bumper_b_;

    std_msgs::BoolConstPtr bumper_f_msg_;
    std_msgs::BoolConstPtr bumper_b_msg_;

    void bumperfCallback(const std_msgs::BoolConstPtr& msg);
    void bumperbCallback(const std_msgs::BoolConstPtr& msg);

    // pose publishing
    std::string robot_frame_name;
};

} // end namespace emc

#endif
