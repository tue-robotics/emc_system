#ifndef EMC_SYSTEM_SYSTEM_H_
#define EMC_SYSTEM_SYSTEM_H_

#include "emc/data.h"

#include <map>
#include <iostream>

namespace emc
{

class Communication;

class System
{

public:

    System();

    ~System();

    void setStartState(int state) { state_ = state; }

    void registerState(int state, int (*func)(const ComputationData&))
    {
        while((int)state_details.size() <= state)
            state_details.push_back(StateDetail());

        StateDetail& s = state_details[state];
        s.func = func;
    }

    void registerTransition(int state1, int event, int state2)
    {
        if (state_details.size() <= state1 || !state_details[state1].func)
        {
            std::cout << "Unknown state: " << state1 << std::endl;
            return;
        }

        if (state_details.size() <= state2 || !state_details[state2].func)
        {
            std::cout << "Unknown state: " << state2 << std::endl;
            return;
        }

        StateDetail& s1 = state_details[state1];
        StateDetail& s2 = state_details[state2];

        s1.transitions[event] = state2;
    }

    void run();

private:

    Communication* comm_;

    int state_;

    struct StateDetail
    {
        StateDetail() : func(0) {}

        std::map<int, int> transitions;
        int (*func)(const ComputationData&);
    };

    std::vector<StateDetail> state_details;
};

} // end namespace emc

#endif
