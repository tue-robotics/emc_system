#ifndef EMC_COMMUNICATION_H_
#define EMC_COMMUNICATION_H_

#include "emc/data.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <sensor_msgs/LaserScan.h>

namespace emc
{

class Communication
{

public:

    Communication();

    ~Communication();

    void init();

    void fillData(ComputationData& data);

private:

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_laser_;
    ros::Publisher pub_base_ref_;

    sensor_msgs::LaserScanConstPtr laser_msg_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

};

} // end namespace emc

#endif
