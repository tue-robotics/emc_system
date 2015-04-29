#include "emc/data.h"

#include <ros/init.h>  // for ros::ok()

namespace emc
{

bool FSMInterface::running() const
{
    return ros::ok();
}

}
