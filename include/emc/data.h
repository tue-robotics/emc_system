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
    double range_max;
    std::vector<float> ranges;
};

// ----------------------------------------------------------------------------------------------------

class Data
{

public:

    Data(Communication* comm) : comm_(comm) {}

    void raiseEvent(const char* event)
    {
        event_ = event;
    }

    bool readLaserData(LaserData& scan);

    void sendBaseReference(double vx, double vy, double va);

    bool running() const;

    const std::string& event() const { return event_; }

private:

    Communication* comm_;

    std::string event_;

};

} // end namespace emc

#endif
