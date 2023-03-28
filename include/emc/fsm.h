#ifndef EMC_SYSTEM_SYSTEM_H_
#define EMC_SYSTEM_SYSTEM_H_

#include <map>
#include <vector>
#include <iostream>
#include <ros/init.h>
#include <ros/rate.h>

namespace emc
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
static const char* no_event = 0;
#pragma GCC diagnostic push

class Communication;

class FSMInterface
{

public:

    void raiseEvent(const char* event)
    {
        event_ = event;
    }

    bool running() const;

    const std::string& event() const { return event_; }

private:

    std::string event_;

};

template <class T>
using state_function = void (*)(FSMInterface&, T&);

template <class T>
class FSM
{

public:

    FSM()
    {
        // Register the special 'null' event
        events.push_back("NO_EVENT");
        event_to_int[""] = 0;
    }

    ~FSM()
    {

    }

    void setInitialState(const char* state)
    {
        state_ = getState(state);
        if (state_ < 0)
            addError("While setting initial state: Unknown state: '" + std::string(state) + "'");
    }

    void registerState(const char* state, state_function<T> func)
    {
        int s_id = addState(state);
        if (s_id < 0)
            return;

        StateDetail& s = state_details[s_id];
        s.func = func;
    }

    void registerTransition(const char* state1, const char* event, const char* state2)
    {
        int s1 = getState(state1);
        int s2 = getState(state2);
        int e = getOrAddEvent(event);

        if (s1 < 0 || s2 < 0)
        {
            std::string err = "While adding transition '" + std::string(state1) + "' -> '" + eventToString(e)
                    + "' -> '" + std::string(state2) + "':\n\n";

            if (s1 < 0)
                err += "    Unknown state: '" + std::string(state1) + "'\n";
            if (s2 < 0)
                err += "    Unknown state: '" + std::string(state2) + "'\n";

            addError(err);
            return;
        }

        StateDetail& s1_data = state_details[s1];
        s1_data.transitions[e] = s2;
    }

    void run()
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

        ros::Rate r(loop_freq_);
        while(ros::ok())
        {
            FSMInterface fsm;

            StateDetail& s = state_details[state_];
            s.func(fsm, (*user_data_));

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


    void setLoopFrequency(double freq) { loop_freq_ = freq; }

    void setUserData(T& user_data) { user_data_ = &user_data; }

private:

    int state_;

    struct StateDetail
    {
        StateDetail() : func(0) {}

        std::string name;
        std::map<int, int> transitions;
        state_function<T> func;
    };

    std::map<std::string, int> event_to_int;

    std::map<std::string, int> state_to_int;

    std::vector<std::string> events;

    std::vector<StateDetail> state_details;

    T* user_data_;

    double loop_freq_;

    int addState(const char* state)
    {
        if (state_to_int.find(state) != state_to_int.end())
        {
            addError("State '" + std::string(state) + "' already added.");
            return -1;
        }

        int s_id = state_details.size();
        state_to_int[state] = s_id;
        state_details.push_back(StateDetail());
        StateDetail& s = state_details[s_id];
        s.name = state;

        return s_id;
    }

    int getState(const char* state)
    {
        std::map<std::string, int>::const_iterator it = state_to_int.find(state);
        if (it != state_to_int.end())
            return it->second;
        return -1;
    }

    const std::string& stateToString(int state_id)
    {
        return state_details[state_id].name;
    }

    int getEvent(const char* event)
    {
        std::map<std::string, int>::const_iterator it = event_to_int.find(event);
        if (it != event_to_int.end())
            return it->second;
        return -1;
    }

    int getOrAddEvent(const char* event)
    {
        // Special case: the 'null' event
        if (!event)
            return 0;

        std::map<std::string, int>::const_iterator it = event_to_int.find(event);
        if (it != event_to_int.end())
            return it->second;

        int e_id = event_to_int.size();
        event_to_int[event] = e_id;
        events.push_back(event);

        return e_id;
    }

    const std::string& eventToString(int event_id)
    {
        return events[event_id];
    }

    bool has_error_ = false;

    void addError(const std::string& err)
    {
        std::cout << "[EMC SYSTEM] ERROR: " << err << std::endl;
        has_error_ = true;
    }

};

} // end namespace emc

#endif
