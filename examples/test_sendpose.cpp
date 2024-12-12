#include <emc/io.h>
#include <emc/rate.h>

#include <iostream>
#include <cmath>

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;

    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    // Loop while we are properly connected
    while(io.ok())
    {
        emc::OdometryData odom;
        if (io.readOdometryData(odom))
        {
            x = x + cos(yaw) * odom.x - sin(yaw) * odom.y;
            y = y + sin(yaw) * odom.x + cos(yaw) * odom.y;
            yaw = yaw + odom.a;
        }    
        std::cout << "Pose: " << x << ", " << y << ", " << yaw << std::endl;

        io.sendPoseEstimate(x,y,yaw);
        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
