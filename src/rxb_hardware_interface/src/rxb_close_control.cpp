#include "rxb_close_control.h"
#include <math.h>
#include "rxb_model_interface.h"
#include "global.h"


namespace RxbHardwareInterface {


        double *cur_joint_angle;
        const double *cur_cable_force;
        const double *cur_joint_torque;
        const double *cur_ext_force;

        jointState pos_jointq;
        jointState admi_jointq;
        double outVelo;

//every time of joint_state will use this function
    void cable_interp_tmp(cableState &Current_cablelen, cableState &Target_cablelen, std::vector<cableState> &cable_plan_I) {
        cableState cable_tmp;
        fp32 cable_begin, cable_end, cableV_0;
        fp32 t_begin, t_end, t_tmp;
        fp32 a0, a1, a2, a3;
        cable_plan_I.clear();
        for(int i = 0; i < Motor_num_all; i++) {
            cable_tmp.len[i] = 0;
        }
        cable_tmp.t = 0;
        for(int i = 0; i < 11; i++) {//11 tmp points
            cable_plan_I.push_back(cable_tmp);
        }
        t_begin = Current_cablelen.t;
        t_end = Target_cablelen.t;

        for(int i = 0; i < Motor_num_all; i++) {
            cable_begin = Current_cablelen.len[i];
            cable_end = Target_cablelen.len[i];
            cableV_0 = cloop_cable_v0[i];
            for(int j = 0; j < 11; j++) {
                t_tmp = t_begin + (t_end - t_begin) / 10 * j;
                a0 = cable_begin;
                a1 = cableV_0;
                a2 = 3 * (cable_end - cable_begin) / (t_end * t_end) - (2 * cableV_0) / t_end;
                a3 = 2 * (cable_begin - cable_end) / (t_end * t_end * t_end) + (cableV_0) / (t_end * t_end);
                cable_plan_I[j].len[i] = a0 + a1 * t_tmp + a2 * t_tmp * t_tmp + a3 * t_tmp * t_tmp * t_tmp;
                cable_plan_I[j].t = t_tmp;
            }

        }

    }

    void resetTargetT(jointState &current_joint, jointState &target_joint) {
        double delta_j = 0.0;
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            if(abs(current_joint.j[i] - target_joint.j[i]) > delta_j) {
                delta_j = abs(current_joint.j[i] - target_joint.j[i]);
            }
        }
        target_joint.t = delta_j / (90 / g_cloopT_s); //deg/s
    }
    //unknown jointV_0
    void joint_interp_tmp(jointState &current_joint, jointState &target_joint, double *next_joint) {
        fp32 joint_begin, joint_end, jointV_0;
        fp32 t_begin, t_end, t_tmp;
        fp32 a0, a1, a2, a3;
        resetTargetT(current_joint, target_joint);
        t_begin = current_joint.t;
        t_end = target_joint.t;
        t_tmp = t_begin + global_T_Interval_s;
        if(t_tmp > t_begin && t_tmp < t_end) {
            for (int i = 0; i < Controllable_DOF_num_all; i++) {
                joint_begin = current_joint.j[i];
                joint_end = target_joint.j[i];
                jointV_0 = cloop_joint_v0[i];
                a0 = joint_begin;
                a1 = jointV_0;
                a2 = 3 * (joint_end - joint_begin) / (t_end * t_end) - (2 * jointV_0) / t_end;
                a3 = 2 * (joint_begin - joint_end) / (t_end * t_end * t_end) + (jointV_0) / (t_end * t_end);
                next_joint[i] = a0 + a1 * t_tmp + a2 * t_tmp * t_tmp + a3 * t_tmp * t_tmp * t_tmp;
            }
        } else if(t_tmp > t_begin && t_tmp >= t_end) {
            for (int i = 0; i < Controllable_DOF_num_all; i++) {
                next_joint[i] = target_joint.j[i];
            }
        }
    }

    void cable_Cloop_Interpolation(std::vector<cableState> &plan_I, double *cable_pos, double *cable_vel) {
        //int mul_Num = 5;
        int t_Num, t_Num_Cau;
        double t_Min, t_Max, t_Tmp, duration_All;
        double duration_Tmp = 0;
        double cableP_tmp, cableV_tmp;

        cableState cs;
        std::vector<cableState>::iterator iter;

        t_Num = plan_I.size();
        cableP_tmp = 0;
        cableV_tmp = 0;

        double *t_Array = new double[t_Num];
        double **cablen = new double *[Motor_num_all];
        for(int i = 0; i < Motor_num_all; i++) {
            cablen[i] = new double[t_Num];
        }

        int i = 0;
        for( iter = plan_I.begin(); iter != plan_I.end(); iter++) {
            t_Array[i] = (iter->t);
            for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
                cablen[cable_num][i] = (iter->len[cable_num]);
            }
            i++;
        }
        t_Min = t_Array[0];
        t_Max = t_Array[t_Num - 1];
        t_Tmp = t_Min + global_T_Interval_s;
        duration_All = t_Max - t_Min;

        if((t_Tmp > t_Min) && (t_Tmp < t_Max)) {
            for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
                single_Cable_Interpolation(t_Array, cablen[cable_num], t_Num, cloop_cable_v0[cable_num], t_Tmp, cableP_tmp, cableV_tmp);
                cable_pos[cable_num] = cableP_tmp;
                cable_vel[cable_num] = cableV_tmp;
            }
        } else if((t_Tmp > t_Max) && (t_Min < t_Max)) {
            for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
                cable_pos[cable_num] = cablen[cable_num][t_Num - 1];
                cable_vel[0] = (cablen[cable_num][t_Num - 1] - cablen[cable_num][0]) / (t_Max - t_Min);
            }
        } else if(t_Min >= t_Max) {
            for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
                cable_pos[cable_num] = cablen[cable_num][t_Num - 1];
                cable_vel[0] = 0;
            }

        }

        delete[] t_Array;
        for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
            delete[] cablen[cable_num];
        }
        delete[] cablen;


    }


    void single_Cable_Interpolation(double *t_Array, double *joint, int num, double leftBound, double t, double &y, double &dy) {
        double  rightBound = 0;
        SplineSpace::Spline sp(t_Array, joint, num, SplineSpace::GivenFirstOrder, leftBound, rightBound);
        sp.SinglePointInterp(t, y, dy);

    }



}
