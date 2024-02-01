#include "emc/rate.h"

#include <ros/rate.h>
<<<<<<< HEAD
#include <ros/ros.h>
#include <ros/console.h>
=======
>>>>>>> parent of 5d965d5 (spin when sleeping)

namespace emc
{

Rate::Rate(double freq)
{
    ros::Time::init();
    rate_ = new ros::Rate(freq);
}

Rate::~Rate()
{
    if (rate_)
        delete rate_;
}

void Rate::sleep()
{
    if (!rate_->sleep())
    {
        ROS_WARN_STREAM("Could not complete the cycle in " << rate_->expectedCycleTime() << ", instead took " << rate_->cycleTime());
    }
}

}
