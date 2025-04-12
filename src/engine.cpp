#include "emc/engine.h"
#include "emc/communication.h"

#include <rclcpp/rclcpp.hpp>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

Engine::Engine() : io_(nullptr), state_(-1), user_data_(nullptr), loop_freq_(0), has_error_(false)
{
    // Register the special 'null' event
    events.push_back("NO_EVENT");
    event_to_int[""] = 0;
}

// ----------------------------------------------------------------------------------------------------

Engine::~Engine()
{
    if (io_)
        delete io_;
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

//    comm_->init();
    if (io_)
        delete io_;

    io_ = new IO;

    auto node = rclcpp::Node::make_shared("engine_node");
    rclcpp::Rate rate(loop_freq_);

    while (rclcpp::ok())
    {
        FSMInterface fsm;

        StateDetail& s = state_details[state_];
        s.func(fsm, *io_, user_data_);

        if (!rclcpp::ok())
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

        rate.sleep();
    }
}

} // end namespace emc

