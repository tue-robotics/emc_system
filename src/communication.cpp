#include "emc/communication.h"

#include <ros/node_handle.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Communication::Communication()
{
}

// ----------------------------------------------------------------------------------------------------

Communication::~Communication()
{
}

// ----------------------------------------------------------------------------------------------------

void Communication::init()
{
    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);
    sub_laser_ = nh.subscribe<sensor_msgs::LaserScan>("/pico/laser", 1, &Communication::laserCallback, this);
}

// ----------------------------------------------------------------------------------------------------

void Communication::fillData(ComputationData& data)
{
    laser_msg_.reset();
    cb_queue_.callAvailable();

    if (laser_msg_)
    {
        data.laser_data.ranges = laser_msg_->ranges;
    }
    else
        data.laser_data.ranges.clear();
}

// ----------------------------------------------------------------------------------------------------

void Communication::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    laser_msg_ = msg;
}

} // end namespace emc

