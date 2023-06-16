#include "emc/rate.h"

#include <ros/rate.h>
#include <ros/ros.h>

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
    ros::spinOnce();
    rate_->sleep();
}

}
