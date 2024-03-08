#include "emc/ros2publisher.h"

#include <emc/rate.h>
#include "rclcpp/rclcpp.hpp"

int main()
{
    rclcpp::init(0,nullptr);

    // Create publisher
    emc::Ros2Publisher pub;

    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    // Loop while we are properly connected
    while(rclcpp::ok())
    {
        std::cout << "sending msgs" << std::endl;
        pub.sendBaseVelocity(1, 0, 0.1);
        pub.speak("hello world");
        
        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
