#ifndef __RXB_ALGORITHM
#define __RXB_ALGORITHM

#include "global.h"
#include "Spline.h"
#include <list>

namespace RxbHardwareInterface {

    void cable_Interpolation(std::list<cableState> &plan_I, std::list<cableState> &plan_O, int &num);
    void joints_Interpolation(std::list<jointState> &plan_I, std::list<jointState> &plan_O, int &num);

    void joints_SimpleInterpolation(int num, jointState &joints1, jointState &jointsn, std::list<jointState> &RXB_joint_plan);

    void computerCableVel(int Motor_num, cableState &rxb_setCableState, cableState &rxb_priorCableState, fp32 &g_Cable_Vel_Cacu);
    //void single_Interpolation_1(double *t_Array, double *joint, int num, double *t_Cau_Array, double *joint_Cau, int num_Cau);
    void single_Interpolation_1(double *t_Array, double *joint, int num, double *t_Cau_Array, double *joint_Cau,  double *joint_vel, double *joint_acc, int num_Cau);
}


#endif
