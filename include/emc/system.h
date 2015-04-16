#ifndef EMC_SYSTEM_SYSTEM_H_
#define EMC_SYSTEM_SYSTEM_H_

#include "emc/data.h"

namespace emc
{

class Communication;

class System
{

public:

    System();

    ~System();

    void registerComputation(void (*f_computation)(const ComputationData&))
    {
        f_computation_ = f_computation;
    }

    void run();

private:

    Communication* comm_;

    void (*f_computation_)(const ComputationData&);
};

} // end namespace emc

#endif
