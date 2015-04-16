#include "emc/data.h"

#include "emc/communication.h"

#include <ros/init.h>  // for ros::ok()

namespace emc
{

bool Data::readLaserData(LaserData& scan)
{
    return comm_->readLaserData(scan);
}

void Data::sendBaseReference(double vx, double vy, double va)
{
    comm_->sendBaseReference(vx, vy, va);
}

bool Data::running() const
{
    return ros::ok();
}

}
