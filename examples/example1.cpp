#include <emc/system.h>

#include <iostream>

// ----------------------------------------------------------------------------------------------------

void computation(const emc::ComputationData& data)
{
    if (!data.laser_data.ranges.empty())
        std::cout << data.laser_data.ranges[0] << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    emc::System system;

    system.registerComputation(computation);

    system.run();

    return 0;
}
