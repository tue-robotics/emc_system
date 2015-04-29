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

class FSMInterface
{

public:

    void raiseEvent(const char* event)
    {
        event_ = event;
    }

    bool running() const;

    const std::string& event() const { return event_; }

private:

    std::string event_;

};

} // end namespace emc

#endif
