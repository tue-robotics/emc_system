#ifndef EMC_SYSTEM_DATA_H_
#define EMC_SYSTEM_DATA_H_

#include <vector>
#include <string>

namespace emc
{

class Communication;

// ----------------------------------------------------------------------------------------------------

struct LaserData
{
    double range_min;
    double range_max;
    double angle_min;
    double angle_max;
    double angle_increment;
    double timestamp;
    std::vector<float> ranges;
};

struct ControlEffort
{
    double x;
    double y;
    double th;
    double timestamp;
};

// ----------------------------------------------------------------------------------------------------


} // end namespace emc

#endif
