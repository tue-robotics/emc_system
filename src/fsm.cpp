#include "emc/fsm.h"

#include <ros/init.h>
#include <ros/rate.h>

namespace emc
{

// ----------------------------------------------------------------------------------------------------

bool FSMInterface::running() const
{
    return ros::ok();
}

} // end namespace emc

