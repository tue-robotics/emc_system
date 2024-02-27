#ifndef EMC_SYSTEM_RATE_H_
#define EMC_SYSTEM_RATE_H_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logger.hpp>

/*namespace rclcpp
{
class GenericRate;
}
*/
namespace emc
{

class Rate2
{

public:

    Rate2(double freq);

    ~Rate2();

    void sleep();

private:

    rclcpp::GenericRate<std::chrono::system_clock>* rate_;
    //rclcpp::Logger logger_;
};


} // end namespace emc

#endif
