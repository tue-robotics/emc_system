#ifndef EMC_SYSTEM_IO_H_
#define EMC_SYSTEM_IO_H_

#include <vector>
#include <string>

#include "emc/data.h"

namespace emc
{

class Communication;

class IO
{

public:

    IO(Communication* comm = 0);

    ~IO();

    bool readLaserData(LaserData& scan);

    void sendBaseReference(double vx, double vy, double va);

    bool ok();

private:

    Communication* comm_;

};

} // end namespace emc

#endif
