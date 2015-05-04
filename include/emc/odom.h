#ifndef EMC_SYSTEM_ODOM_H_
#define EMC_SYSTEM_ODOM_H_

namespace emc
{

// ----------------------------------------------------------------------------------------------------

struct OdometryData
{
    double x;
    double y;
    double a;
    double timestamp;
};

} // end namespace emc

#endif
