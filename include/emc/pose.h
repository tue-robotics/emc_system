#ifndef EMC_SYSTEM_POSE_H_
#define EMC_SYSTEM_POSE_H_

namespace emc
{

// ----------------------------------------------------------------------------------------------------

struct PoseData
{
    double x;       // m
    double y;       // m
    double z;       // m
    double roll;    // rad
    double pitch;   // rad
    double yaw;     // rad
    double timestamp;
};

} // end namespace emc

#endif