#ifndef _TXB_CLOSE_CONTROL_
#define _TXB_CLOSE_CONTROL_


#include <list>
#include <iostream>
#include <vector>
#include "global.h"
#include "rxb_model_interface.h"
#include "motor_driver.h"
#include "rxb_algorithm.h"

using namespace std;



namespace RxbHardwareInterface {



    extern double *cur_joint_angle;
    extern const double *cur_cable_force;
    extern const double *cur_joint_torque;
    extern const double *cur_ext_force;

    extern jointState pos_jointq;
    extern jointState admi_jointq;
    extern double outVelo;

    // 关节速度 pid
    extern double pid_kp[Controllable_DOF_num_all];
    extern double pid_kd[Controllable_DOF_num_all];
    extern double pid_ki[Controllable_DOF_num_all];
    // 关节速度 e de ie
    extern double error_joint[Controllable_DOF_num_all];
    extern double derror_joint[Controllable_DOF_num_all];
    extern double ierror_joint[Controllable_DOF_num_all];
    extern double pid_out[Controllable_DOF_num_all];

    void cable_interp_tmp(cableState &, cableState &, std::vector<cableState> &);
    void joint_interp_tmp(jointState &, jointState &, double *next_joint);
    void cable_Cloop_Interpolation(std::vector<cableState> &, double *, double *);
    void single_Cable_Interpolation(double *, double *, int, double, double, double &, double &);
    void calculateCablePosV();
    void except_pos_ctrl();


}
#endif
