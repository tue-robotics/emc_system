#ifndef EMC_SYSTEM_RATE_H_
#define EMC_SYSTEM_RATE_H_

#include <chrono>

namespace rclcpp
{
template<typename Type> class GenericRate;
}

namespace emc
{

class Rate
{

public:

    Rate(double freq);

    ~Rate();

    void sleep();

private:

    rclcpp::GenericRate<std::chrono::system_clock>* rate_;
    //rclcpp::Logger logger_;
};


} // end namespace emc

#endif
