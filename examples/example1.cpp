#include <emc/engine.h>

#include <iostream>
#include <unistd.h> // usleep

// ----------------------------------------------------------------------------------------------------

double calculateMinimumDistance(const emc::LaserData& scan)
{
    double r_min = scan.ranges[0];
    for(unsigned int i = 1; i < scan.ranges.size(); ++i)
    {
        if (scan.ranges[i] < r_min)
            r_min = scan.ranges[i];
    }

    return r_min;
}

// ----------------------------------------------------------------------------------------------------

void state_driving(emc::FSMInterface& fsm, emc::IO& io)
{
    std::cout << "driving" << std::endl;

    while(fsm.running())
    {
        emc::LaserData scan;
        if (io.readLaserData(scan))
        {
            double min_dist = calculateMinimumDistance(scan);
            if (min_dist < 0.2)    // magic number!
            {
                fsm.raiseEvent("obstacle_near");
                return;
            }

            // No obstacles near, so let's go!
            io.sendBaseReference(0.3, 0, 0);
        }

        usleep(100000);
    }
}

// ----------------------------------------------------------------------------------------------------

void state_waiting(emc::FSMInterface& fsm, emc::IO& io)
{
    std::cout << "waiting" << std::endl;

    // Stop the base!
    io.sendBaseReference(0, 0, 0);

    while(fsm.running())
    {
        emc::LaserData scan;
        if (io.readLaserData(scan))
        {
            double min_dist = calculateMinimumDistance(scan);
            if (min_dist > 0.2)
            {
                // All clear!
                fsm.raiseEvent("all_clear");
                return;
            }
        }

        usleep(100000);
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    emc::Engine engine;

    engine.registerState("driving", state_driving);
    engine.registerState("waiting", state_waiting);
    engine.registerTransition("driving", "obstacle_near", "waiting");
    engine.registerTransition("waiting", "all_clear", "driving");

    engine.setInitialState("waiting");

    engine.run();

    return 0;
}
