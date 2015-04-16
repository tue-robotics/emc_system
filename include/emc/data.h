#ifndef EMC_SYSTEM_DATA_H_
#define EMC_SYSTEM_DATA_H_

#include <vector>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

struct LaserData
{
    std::vector<float> ranges;
};

// ----------------------------------------------------------------------------------------------------

struct ComputationData
{
    double dt;

    LaserData laser_data;

    bool readLaserData(LaserData& scan) {}

};

} // end namespace emc

#endif
