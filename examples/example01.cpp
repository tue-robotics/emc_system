#include <emc/io.h>
#include <emc/rate.h>

#include <iostream>

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;

    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    bool waiting_for_door = false;

    // Loop while we are properly connected
    while(io.ok())
    {
        // Send a reference to the base controller (vx, vy, vtheta)
//        io.sendBaseReference(0.1, 0, 0);

//        emc::OdometryData odom;
//        if (io.readOdometryData(odom))
//            std::cout << odom.a << std::endl;

        emc::LaserData scan;
        if (io.readLaserData(scan))
        {
            float r = scan.ranges[scan.ranges.size() / 2];
            if (r > scan.range_min && r < scan.range_max && r > 0.5)
            {
                io.sendBaseReference(0.3, 0, 0);
                waiting_for_door = false;
            }
            else
            {
                io.sendBaseReference(0, 0, 0);

                if (!waiting_for_door)
                {
                    std::cout << "Sending request" << std::endl;
                    waiting_for_door = true;
                    io.sendOpendoorRequest();
                }
            }
        }
        else
        {
            io.sendBaseReference(0, 0, 0);
        }

        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
