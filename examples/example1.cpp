#include <emc/system.h>

#include <iostream>

enum
{
    STATE_A,
    STATE_B
};

enum
{
    EVENT_0
};

// ----------------------------------------------------------------------------------------------------

int state_a(const emc::ComputationData& data)
{
    std::cout << "A" << std::endl;
    return EVENT_0;
}

// ----------------------------------------------------------------------------------------------------

int state_b(const emc::ComputationData& data)
{
    std::cout << "B" << std::endl;
    return EVENT_0;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    emc::System system;

    system.registerState(STATE_A, state_a);
    system.registerState(STATE_B, state_b);
    system.registerTransition(STATE_A, EVENT_0, STATE_B);
    system.registerTransition(STATE_B, EVENT_0, STATE_A);

    system.setStartState(STATE_A);

    system.run();

    return 0;
}
