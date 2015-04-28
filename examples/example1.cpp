#include <emc/engine.h>

#include <iostream>
#include <unistd.h> // usleep

// ----------------------------------------------------------------------------------------------------

// This data struct will be available in all functions

struct MyData
{
    double max_obstacle_distance;
};

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

void state_initialize(emc::FSMInterface& fsm, emc::IO& io, void* user_data)
{
    std::cout << "initializing..." << std::endl;

    // ...

    std::cout << "... Done!" << std::endl;

    fsm.raiseEvent("initialized");
}

// ----------------------------------------------------------------------------------------------------

void state_driving(emc::FSMInterface& fsm, emc::IO& io, void* user_data)
{
    // The FSM engine does not know what kind of user data will be put into it, so it stores it as a
    // 'void pointer'. A void pointer is simply a pointer to data of which the type is unknown. But
    // we DO know the type! Therefore we can 'cast' it to our MyData type that we defined above. You
    // don't have to exactly know what is going on here, just know that the resulting 'my_data' is a
    // pointer to our data, with the correct type.
    MyData* my_data = static_cast<MyData*>(user_data);

    emc::LaserData scan;
    if (!io.readLaserData(scan))
        return; // No data, so not much to do

    double min_dist = calculateMinimumDistance(scan);
    if (min_dist < my_data->max_obstacle_distance)    // magic number!
    {
        fsm.raiseEvent("obstacle_near");
        return;
    }

    // No obstacles near, so let's go!
    io.sendBaseReference(0.3, 0, 0);

    // We're driving...!
    std::cout << "driving" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void state_waiting(emc::FSMInterface& fsm, emc::IO& io, void* user_data)
{
    // Cast the data to our MyData type! (See explanation in state_driving)
    MyData* my_data = static_cast<MyData*>(user_data);

    // Stop the base!
    io.sendBaseReference(0, 0, 0);

    // We're waiting..!
    std::cout << "waiting" << std::endl;

    emc::LaserData scan;
    if (!io.readLaserData(scan))
        return; // No data, so not much to do

    double min_dist = calculateMinimumDistance(scan);
    if (min_dist > my_data->max_obstacle_distance)
    {
        // All clear!
        fsm.raiseEvent("all_clear");
        return;
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Initialize the state machine engine
    emc::Engine engine;

    // Initialize the data that we will use. This data will be available in all state functions
    MyData data;
    data.max_obstacle_distance = 0.2; // Just a magic number ...
    engine.setUserData(&data);

    // Set the loop frequency of the engine. This means or state functions will be called at this frequency
    engine.setLoopFrequency(10);

    // Register the state functions to the engine. This tells the engine which functions should be called in which state
    engine.registerState("initialize", state_initialize);
    engine.registerState("driving", state_driving);
    engine.registerState("waiting", state_waiting);

    // Register the transitions. The arguments are: State1, Event, State2 and say: if Event is raised when in State1, go to State2
    engine.registerTransition("initialize", "initialized", "driving");
    engine.registerTransition("driving", "obstacle_near", "waiting");
    engine.registerTransition("waiting", "all_clear", "driving");

    // Set the state in which the engine will start running
    engine.setInitialState("initialize");

    // Start the engine!
    engine.run();

    return 0;
}
