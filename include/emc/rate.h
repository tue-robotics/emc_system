#ifndef EMC_SYSTEM_RATE_H_
#define EMC_SYSTEM_RATE_H_

#include <ros/rate.h>

namespace emc
{

class Rate
{

public:

    Rate(double freq) : rate_(freq) {}

    void sleep() { rate_.sleep(); }

private:

    ros::Rate rate_;

};


} // end namespace emc

#endif
