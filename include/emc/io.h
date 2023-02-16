#ifndef EMC_SYSTEM_IO_H_
#define EMC_SYSTEM_IO_H_

#include <vector>
#include <string>

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/bumper.h"

#include <geolib/datatypes.h>

namespace emc
{

class Communication;

class IO
{

public:

    IO(Communication* comm = nullptr);
    IO(std::string robot_name);

    ~IO();

    bool readLaserData(LaserData& scan);

    bool readOdometryData(OdometryData& odom);

    bool readFrontBumperData(BumperData& bumper);
    bool readBackBumperData(BumperData& bumper);

    //bool readControlEffort(ControlEffort& ce);

    void sendBaseReference(double vx, double vy, double va);

    void sendOpendoorRequest();

    bool ok();

    void speak(const std::string& text);
    
    void play(const std::string& file);

    void sendPoseEstimate(double& px, double& py, double& pz, double& rx, double& ry, double& rz, double& rw); //use quaternion

    void sendPoseEstimate(double& px, double& py, double& pz, double& rr, double& rp, double& ry); //use roll pitch yaw

    void sendPoseEstimate(geo::pose3D& pose); //use pose


private:

    Communication* comm_;

};

} // end namespace emc

#endif
