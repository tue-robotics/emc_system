#include "emc/engine.h"
#include "emc/communication.h"

#include <ros/init.h>
#include <ros/rate.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Engine::Engine() : comm_(new Communication), state_(-1), has_error_(false), loop_freq_(0), user_data_(0)
{
    // Register the special 'null' event
    events.push_back("NO_EVENT");
    event_to_int[""] = 0;
}

// ----------------------------------------------------------------------------------------------------

Engine::~Engine()
{
    delete comm_;
}

// ----------------------------------------------------------------------------------------------------

void Engine::run()
{
    if (has_error_)
        return;

    if (state_ < 0)
    {
        addError("Initial state not specified");
        return;
    }

    if (loop_freq_ <= 0)
    {
        addError("Loop frequency not set.");
        return;
    }

    ros::VP_string args;
    ros::init(args, "emc_system");
    ros::Time::init();

    comm_->init();

    ros::Rate r(loop_freq_);
    while(ros::ok())
    {
        IO io(comm_);
        FSMInterface fsm;

        StateDetail& s = state_details[state_];
        s.func(fsm, io, user_data_);

        if (!ros::ok())
            break;

        int event_id = getEvent(fsm.event().c_str());
        if (event_id < 0)
        {
            addError("Unknown event '" + fsm.event() + "' raised in state '" + stateToString(state_) + "'");
            break;
        }

        if (event_id > 0)
        {
            std::map<int, int> ::const_iterator it = s.transitions.find(event_id);
            if (it == s.transitions.end())
            {
                addError("Cannot deal with event '" + eventToString(event_id) + "'' in state '" + stateToString(state_) + "'");
                break;
            }

            state_ = it->second;
        }

        r.sleep();
    }
}

} // end namespace emc

