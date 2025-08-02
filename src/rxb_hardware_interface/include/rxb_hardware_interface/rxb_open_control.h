#ifndef __RXB_OPEN_CONTROL
#define __RXB_OPEN_CONTROL

#include <list>
#include <iostream>
#include "global.h"
#include "rxb_model_interface.h"

namespace RxbHardwareInterface {

    void p2p_jointTrajectoryPlan(const jointState &, jointState &, std::list<jointState> &);
    bool judgeArrival(const fp64 *now, fp64 *end , fp64 ERR);


}



#endif
