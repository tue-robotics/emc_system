#include "emc/io.h"

#include "emc/communication.h"

#include <ros/init.h>  // for ros::ok()

namespace emc
{

IO::IO(Communication* comm) : comm_(comm)
{
    if (!comm_)
        comm_ = new Communication;
}

IO::~IO()
{
    delete comm_;
}

bool IO::readLaserData(LaserData& scan)
{
    return comm_->readLaserData(scan);
}

void IO::sendBaseReference(double vx, double vy, double va)
{
    comm_->sendBaseVelocity(vx, vy, va);
}

bool IO::ok()
{
    return ros::ok();
}

}
