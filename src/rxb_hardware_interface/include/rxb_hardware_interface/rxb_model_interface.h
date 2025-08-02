#ifndef RXB_ALGORITHM_
#define RXB_ALGORITHM_

#include "global.h"
#include <vector>

namespace RxbHardwareInterface {

    void RXB_JntAngle_To_Cable_Length_new(double *JointAngle, double *CableLengthCalcted );
    void jointPlan2CablePlan(std::list<jointState> &RXB_joint_plan, std::list<cableState> &RXB_Cable_Plan);
    //void RXB_Cable_To_JntAngle(double Cable_l[],double Jnt_Angle[]);
    //完善正逆运动学函数
    void RXB_CableLenOfSingleModuJoint(const int ModularJointNum, const double Cable_Position[3], const double r, const double *JointAngle, double *CableLengthCalcted);
    void RXB_JntState_To_CableVel(double Jnt_Angle[], double dJnt_Angle[], double Cable_v[]);

}

#endif
