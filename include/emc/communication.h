#ifndef EMC_COMMUNICATION_H_
#define EMC_COMMUNICATION_H_

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/bumper.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/MapMetaData.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <emc_system/controlEffort.h>
#include <string>

namespace emc
{

struct MapConfig{
    double mapResolution, mapOffsetX, mapOffsetY, mapOrientation;
    bool mapInitialised;
};

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

    //bool readControlEffort(ControlEffort& ce);

    void sendBaseVelocity(double vx, double vy, double va);

    void sendOpendoorRequest();

    void speak(const std::string& text);
    
    void play(const std::string& file);

    // Postion data
    void sendPoseEstimate(const geometry_msgs::Transform& pose);

    // Map data
    bool getMapConfig(MapConfig& config);

private:

    // Base velocity reference

    ros::Publisher pub_base_ref_;

    ros::Publisher pub_open_door_;

    ros::Publisher pub_speak_;
    
    ros::Publisher pub_play_;

    // Position data

    ros::Publisher pub_joints;

    tf2_ros::TransformBroadcaster* pub_tf2; //has to be defined after ros::init(), which is called in the constructor


    // Laser data

    ros::CallbackQueue laser_cb_queue_;

    ros::Subscriber sub_laser_;

    sensor_msgs::LaserScanConstPtr laser_msg_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);


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

    // Map data

    ros::CallbackQueue mapdata_cb_queue_;

    ros::Subscriber sub_mapdata_;

    MapConfig mapconfig;

    void mapCallback(const nav_msgs::MapMetaData::ConstPtr& msg);

/*
    // Control effort data

    ros::CallbackQueue ce_cb_queue_;

    ros::Subscriber sub_ce_;

    emc_system::controlEffortConstPtr ce_msg_;

    void controlEffortCallback(const emc_system::controlEffortConstPtr& msg);
*/
    

};

} // end namespace emc

#endif
