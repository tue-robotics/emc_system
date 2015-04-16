#include "emc/system.h"
#include "emc/communication.h"

#include <ros/init.h>
#include <ros/rate.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

System::System() : comm_(new Communication)
{
}

// ----------------------------------------------------------------------------------------------------

System::~System()
{
    delete comm_;
}

// ----------------------------------------------------------------------------------------------------

void System::run()
{
    ros::VP_string args;
    ros::init(args, "emc_system");
    ros::Time::init();

    comm_->init();

    double cycle_freq = 100;
    double cycle_time = 1.0 / cycle_freq;

    ros::Rate r(100);
    while(ros::ok())
    {
        ComputationData data;
        data.dt = cycle_time;
        comm_->fillData(data);

        f_computation_(data);

        r.sleep();
    }
}

} // end namespace emc

