#ifndef EMC_SYSTEM_RATE_H_
#define EMC_SYSTEM_RATE_H_

#include "rclcpp/rclcpp.hpp"

namespace emc
{

class Rate
{
public:
    Rate(double freq);
    ~Rate();
    void sleep();

private:
    rclcpp::Rate* rate_;
    //rclcpp::Logger logger_;
};

} // end namespace emc

#endif
