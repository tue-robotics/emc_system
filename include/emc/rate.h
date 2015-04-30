#ifndef EMC_SYSTEM_RATE_H_
#define EMC_SYSTEM_RATE_H_

namespace ros
{
class Rate;
}

namespace emc
{

class Rate
{

public:

    Rate(double freq);

    void sleep();

private:

    ros::Rate* rate_;

};


} // end namespace emc

#endif
