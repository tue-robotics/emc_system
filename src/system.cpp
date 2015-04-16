#include "emc/system.h"
#include "emc/communication.h"

#include <ros/init.h>
#include <ros/rate.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

System::System() : comm_(new Communication), state_(-1), has_error_(false)
{
    // Register the special 'null' event
    events.push_back("NO_EVENT");
    event_to_int[""] = 0;
}

// ----------------------------------------------------------------------------------------------------

System::~System()
{
    delete comm_;
}

// ----------------------------------------------------------------------------------------------------

void System::run()
{
    if (has_error_)
        return;

    if (state_ < 0)
    {
        addError("Initial state not specified");
        return;
    }

    ros::VP_string args;
    ros::init(args, "emc_system");
    ros::Time::init();

    comm_->init();

    while(ros::ok())
    {
        ComputationData data;
        comm_->fillData(data);

        StateDetail& s = state_details[state_];
        s.func(data);

        int event_id = getEvent(data.event.c_str());
        if (event_id < 0)
        {
            addError("Unknown event '" + data.event + "' raised in state '" + stateToString(state_) + "'");
            break;
        }

        std::map<int, int> ::const_iterator it = s.transitions.find(event_id);
        if (it == s.transitions.end())
        {
            addError("Cannot deal with event '" + eventToString(event_id) + "'' in state '" + stateToString(state_) + "'");
            break;
        }

        state_ = it->second;
    }
}

} // end namespace emc

