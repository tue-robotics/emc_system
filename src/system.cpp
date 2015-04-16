#include "emc/system.h"
#include "emc/communication.h"

#include <ros/init.h>
#include <ros/rate.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

System::System() : comm_(new Communication), state_(-1)
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
    if (state_ < 0)
    {
        std::cout << "Please provide starting state" << std::endl;
        return;
    }

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

        StateDetail& s = state_details[state_];
        int event = s.func(data);
        std::map<int, int> ::const_iterator it = s.transitions.find(event);
        if (it == s.transitions.end())
        {
            std::cout << "Cannot deal with event " << event << " in state " << state_ << std::endl;
            return;
        }

        state_ = it->second;

        r.sleep();
    }
}

} // end namespace emc

