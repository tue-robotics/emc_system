#include <emc/system.h>

#include <iostream>

// ----------------------------------------------------------------------------------------------------

void state_a(emc::ComputationData& data)
{
    std::cout << "A" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void state_b(emc::ComputationData& data)
{
    std::cout << "B" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    emc::System system;

    system.registerState("STATE_A", state_a);
    system.registerState("STATE_B", state_b);
    system.registerTransition("STATE_A", emc::no_event, "STATE_B");
    system.registerTransition("STATE_B", emc::no_event, "STATE_A");

    system.setInitialState("STATE_A");

    system.run();

    return 0;
}
