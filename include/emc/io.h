#ifndef EMC_SYSTEM_IO_H_
#define EMC_SYSTEM_IO_H_

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

    void resetMarkers();

    void addMarker(const Point &point);

    void sendMarkers();

private:

    Communication* comm_;

};

} // end namespace emc

#endif
