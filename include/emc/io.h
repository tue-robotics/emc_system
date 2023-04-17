#ifndef EMC_SYSTEM_IO_H_
#define EMC_SYSTEM_IO_H_

#include <array>
#include <vector>
#include <string>

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/bumper.h"

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

    bool sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color = {0.0, 0.0, 0.0}, double width = 0.02, int id = 0);

    void sendPoseEstimate(double px, double py, double pz, double rx, double ry, double rz, double rw); //use quaternion

    void sendPoseEstimate(double px, double py, double pz, double rr, double rp, double ry); //use roll pitch yaw

private:

    Communication* comm_;

};

} // end namespace emc

#endif
