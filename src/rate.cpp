#include "emc/rate.h"


emc::Rate::Rate(double freq)
{
    //rclcpp::Time::init();
    rate_ = new rclcpp::Rate(freq);
    //logger_ = rclcpp::get_logger("ratelogger");
}

emc::Rate::~Rate()
{
    if (rate_)
    {
        delete rate_;
    }
}

void emc::Rate::sleep()
{
    if (!rate_->sleep())
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("ratelogger"), "Could not complete the cycle in "); //<< rate_->period() << ", instead took ??");
    }
}

