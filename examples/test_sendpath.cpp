#include <emc/io.h>
#include <emc/rate.h>

#include <iostream>
#include <vector>
#include <cmath>

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;

    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    // Loop while we are properly connected
    while(io.ok())
    {
        double radius = 5;
        double dtheta = 0.3;
        std::vector<std::vector<double>> path;
        for (int i=0; i<10; i++)
        {
            double theta = i* dtheta;
            std::vector<double> point;
            point.push_back(cos(theta)*radius);
            point.push_back(sin(theta)*radius);
            path.push_back(point);
        }
        // Send a reference to the base controller (vx, vy, vtheta)
        io.sendPath(path, {0.0, 0.0, 1.0});
        std::cout << "sending path" << std::endl;

        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
