#include <unistd.h>
#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>

// --------------------------------------------------------------
// This is an example of how to read pose data from the robot 
// using Optitrack.
//
// Author: Tom van Eemeren
// Date: 4-3-2024
// --------------------------------------------------------------

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;
    
    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    // Create a PoseData object to store the pose data
    emc::PoseData pose;

    while (io.ok())
    {
        if (io.readPoseData(pose))
        {
            // Print pose data to terminal
            std::cout << "Pose:\n";
            std::cout << "  x: " << pose.x << std::endl;
            std::cout << "  y: " << pose.y << std::endl;
            std::cout << "  z: " << pose.z << std::endl;
            std::cout << "  roll: " << pose.roll << std::endl;
            std::cout << "  pitch: " << pose.pitch << std::endl;
            std::cout << "  yaw: " << pose.yaw << std::endl;
        }
        else
        {
            std::cout << "No pose data available" << std::endl;
        }
        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
