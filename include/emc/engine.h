#ifndef EMC_SYSTEM_SYSTEM_H_
#define EMC_SYSTEM_SYSTEM_H_

#include "emc/data.h"

#include <map>
#include <iostream>

namespace emc
{

static const char* no_event = 0;

typedef void (*state_function)(FSMInterface&, IO& io);

class Communication;

class Engine
{

public:

    Engine();

    ~Engine();

    void setInitialState(const char* state)
    {
        state_ = getState(state);
        if (state_ < 0)
            addError("While setting initial state: Unknown state: '" + std::string(state) + "'");
    }

    void registerState(const char* state, state_function func)
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

    void run();

private:

    Communication* comm_;

    int state_;

    struct StateDetail
    {
        StateDetail() : func(0) {}

        std::string name;
        std::map<int, int> transitions;
        state_function func;
    };

    std::map<std::string, int> event_to_int;

    std::map<std::string, int> state_to_int;

    std::vector<std::string> events;

    std::vector<StateDetail> state_details;

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

    bool has_error_;

    void addError(const std::string& err)
    {
        std::cout << "[EMC SYSTEM] ERROR: " << err << std::endl;
        has_error_ = true;
    }

};

} // end namespace emc

#endif
