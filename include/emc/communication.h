#ifndef EMC_COMMUNICATION_H_
#define EMC_COMMUNICATION_H_

#include "emc/data.h"
#include "emc/odom.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <emc_system/controlEffort.h>
#include <string>

namespace emc
{

class Communication
{

public:

    Communication(std::string robot_name="pico");

    ~Communication();

    void init();

    bool readLaserData(LaserData& scan);

    bool readOdometryData(OdometryData& odom);

    bool readControlEffort(ControlEffort& ce);

    void sendBaseVelocity(double vx, double vy, double va);

    void sendOpendoorRequest();

    void speak(const std::string& text);

private:

    // Base velocity reference

    ros::Publisher pub_base_ref_;

    ros::Publisher pub_open_door_;

    ros::Publisher pub_speak_;

    


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


    // Control effort data

    ros::CallbackQueue ce_cb_queue_;

    ros::Subscriber sub_ce_;

    emc_system::controlEffortConstPtr ce_msg_;

    void controlEffortCallback(const emc_system::controlEffortConstPtr& msg);

};

} // end namespace emc

#endif
