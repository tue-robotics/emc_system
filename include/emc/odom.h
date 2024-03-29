#ifndef EMC_SYSTEM_ODOM_H_
#define EMC_SYSTEM_ODOM_H_

namespace emc
{

// ----------------------------------------------------------------------------------------------------

struct OdometryData
{
    double x; // m
    double y; // m
    double a; // rad
    double timestamp;
};

} // end namespace emc

#endif
