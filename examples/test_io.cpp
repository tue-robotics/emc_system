#include <emc/io.h>
#include <emc/rate.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"

int main()
{
    rclcpp::init(0,nullptr);
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
        else
            std::cout << "No Odom received" << std::endl;

        emc::LaserData scan;
        if (io.readLaserData(scan))
            std::cout << "Laser: " << scan.ranges.size() << " beams" << std::endl;
        else
            std::cout << "No laser received" << std::endl;

        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
