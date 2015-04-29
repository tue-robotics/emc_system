#include <emc/io.h>
#include <unistd.h> // Needed for usleep

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;

    // Loop while we are properly connected
    while(io.ok())
    {
        // Send a reference to the base controller (vx, vy, vtheta)
        io.sendBaseReference(0.1, 0, 0);
        usleep(1000000); // sleep 100 000 microseconds
    }

    return 0;
}
