#include "emc/rate.h"


emc::Rate2::Rate2(double freq)
{
    //rclcpp::Time::init();
    rate_ = new rclcpp::Rate(freq);
    //logger_ = rclcpp::get_logger("ratelogger");
}

emc::Rate2::~Rate2()
{
    if (rate_)
    {
        delete rate_;
    }
}

void emc::Rate2::sleep()
{
    if (!rate_->sleep())
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("ratelogger"), "Could not complete the cycle in "); //<< rate_->period() << ", instead took ??");
    }
}

