#include "emc/ros2subscriber.h"

#include <emc/rate.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

int main()
{
    rclcpp::init(0,nullptr);

    // Create subscriber
    std::shared_ptr<emc::Ros2Subscriber<sensor_msgs::msg::LaserScan>> laser_node = std::make_shared<emc::Ros2Subscriber<sensor_msgs::msg::LaserScan>>("laser_scan", "emc_laser");
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(laser_node);

    // Create Rate object, which will help using keeping the loop at a fixed frequency
    emc::Rate r(10);

    // Loop while we are properly connected
    while(rclcpp::ok())
    {
        std::cout << "spin" <<std::endl;
        executor.spin_once(std::chrono::nanoseconds(0)); // wait 0 nanoseconds for new messages. just empty the buffer.
        sensor_msgs::msg::LaserScan msg;
        bool newdata = laser_node->readMsg(msg);
        std::cout << "newdata: " << newdata << std::endl;
        std::cout << "Laser: " << msg.range_min << " range_min" << std::endl;
        
        // Sleep remaining time
        r.sleep();
    }

    return 0;
}
