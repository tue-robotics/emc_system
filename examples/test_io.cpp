#include <emc/io.h>
#include <emc/rate.h>

#include <iostream>

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;

    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    // Loop while we are properly connected
    while(io.ok())
    {
        // Send a reference to the base controller (vx, vy, vtheta)
        io.sendBaseReference(0, 0, 0.3);

        emc::OdometryData odom;
        if (io.readOdometryData(odom))
            std::cout << "Odometry: " << odom.x << ", " << odom.y << ", " << odom.a << std::endl;

        emc::LaserData scan;
        if (io.readLaserData(scan))
            std::cout << "Laser: " << scan.ranges.size() << " beams" << std::endl;

        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
