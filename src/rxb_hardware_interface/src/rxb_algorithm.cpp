#include "rxb_algorithm.h"
#include <vector>
#include <list>
#include "Spline.h"

namespace RxbHardwareInterface {


    void computerCableVel(int Motor_num, cableState &rxb_setCableState, cableState &rxb_priorCableState, fp32 &g_Cable_Vel_Cacu) {

        //g_Cable_Vel_Cacu = (rxb_setCableState.len[Motor_num - 1] - rxb_priorCableState.len[Motor_num - 1]) / rxb_setCableState.duration;
        g_Cable_Vel_Cacu = (rxb_setCableState.len[Motor_num - 1] - rxb_priorCableState.len[Motor_num - 1]) / global_T_Interval_s;

    }
/*
    void cable_Interpolation(std::list<cableState> &plan_I, std::list<cableState> &plan_O, int &num) {
        //int mul_Num = 5;
        int t_Num, t_Num_Cau;
        double t_Min, t_Max, t_Tmp, duration_All;
        double duration_Tmp = 0;
        cableState cs;
        std::list<cableState>::iterator iter;

        t_Num = plan_I.size();

        double *t_Array = new double[t_Num];
        double **cablen = new double* [Motor_num_all];
        for(int i = 0; i < Motor_num_all; i++) {
            cablen[i] = new double[t_Num];
        }

        int i = 0, j = 0;
        for( iter = plan_I.begin(); iter != plan_I.end(); iter++) {
            t_Array[i] = (iter->t);
            for(j = 0; j < Motor_num_all; j++) {
                cablen[j][i] = (iter->len[j]);
            }
            i++;
        }
        t_Min = t_Array[0];
        t_Max = t_Array[t_Num - 1];
        duration_All = t_Max - t_Min;
        t_Num_Cau = (int)(duration_All / global_T_Interval_s) + 1 + 1; //begin with the 2nd points

        double *t_Cau_Array = new double[t_Num_Cau];
        double **cablen_Cal = new double *[Motor_num_all];
        for(int i = 0; i < Motor_num_all; i++) {
            cablen_Cal[i] = new double[t_Num_Cau];
        }
        for(int i = 0; i < t_Num_Cau - 1; i++) {
            t_Tmp = t_Min + i * global_T_Interval_s;
            t_Cau_Array[i] = t_Tmp;
        }
        t_Cau_Array[t_Num_Cau - 1] = t_Array[t_Num - 1];

        for(j = 0; j < Motor_num_all; j++) {
            single_Interpolation_1(t_Array, cablen[j], t_Num, t_Cau_Array, cablen_Cal[j], t_Num_Cau);
        }

        for(int i = 1; i < t_Num_Cau; i++) {
            cs.t = t_Cau_Array[i];
            cs.duration = cs.t - duration_Tmp;
            for(j = 0; j < Motor_num_all; j++) {
                cs.len[j] = cablen_Cal[j][i];
            }

            if(i != 0) {
                plan_O.push_back(cs);
            }
            duration_Tmp = cs.t;
        }
        num = t_Num_Cau;
        delete[] t_Array;
        delete[] t_Cau_Array;
        for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
            delete[] cablen[cable_num];
        }
        delete[] cablen;
        for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
            delete[] cablen_Cal[cable_num];
        }
        delete[] cablen_Cal;
    }
*/
    void cable_Interpolation(std::list<cableState> &plan_I, std::list<cableState> &plan_O, int &num) {
        //int mul_Num = 5;
        int t_Num, t_Num_Cau;
        double t_Min, t_Max, t_Tmp, duration_All;
        double duration_Tmp = 0;
        cableState cs;
        std::list<cableState>::iterator iter;

        t_Num = plan_I.size();

        double *t_Array = new double[t_Num];
        double **cablen = new double* [Motor_num_all];
        for(int i = 0; i < Motor_num_all; i++) {
            cablen[i] = new double[t_Num];
        }

        int i = 0, j = 0;
        for( iter = plan_I.begin(); iter != plan_I.end(); iter++) {
            t_Array[i] = (iter->t);
            for(j = 0; j < Motor_num_all; j++) {
                cablen[j][i] = (iter->len[j]);
            }
            i++;
        }
        t_Min = t_Array[0];
        t_Max = t_Array[t_Num - 1];
        duration_All = t_Max - t_Min;
        t_Num_Cau = (int)(duration_All / global_T_Interval_s) + 1 + 1; //begin with the 2nd points

        double *t_Cau_Array = new double[t_Num_Cau];
        double **cablen_Cal = new double *[Motor_num_all];
        double **cablen_vel = new double *[Motor_num_all];
        double **cablen_acc = new double *[Motor_num_all];
        for(int i = 0; i < Motor_num_all; i++) {
            cablen_Cal[i] = new double[t_Num_Cau];
            cablen_vel[i] = new double[t_Num_Cau];
            cablen_acc[i] = new double[t_Num_Cau];
        }
        for(int i = 0; i < t_Num_Cau - 1; i++) {
            t_Tmp = t_Min + i * global_T_Interval_s;
            t_Cau_Array[i] = t_Tmp;
        }
        t_Cau_Array[t_Num_Cau - 1] = t_Array[t_Num - 1];

        for(j = 0; j < Motor_num_all; j++) {
            single_Interpolation_1(t_Array, cablen[j], t_Num, t_Cau_Array, cablen_Cal[j], cablen_vel[j], cablen_acc[j], t_Num_Cau);
        }

        for(int i = 1; i < t_Num_Cau; i++) {
            cs.t = t_Cau_Array[i];
            for(j = 0; j < Motor_num_all; j++) {
                cs.len[j] = cablen_Cal[j][i];
            }

            if(i != 0) {
                plan_O.push_back(cs);
            }
            duration_Tmp = cs.t;
        }
        num = t_Num_Cau;
        delete[] t_Array;
        delete[] t_Cau_Array;
        for(int cable_num = 0; cable_num < Motor_num_all; cable_num++) {
            delete[] cablen[cable_num];
            delete[] cablen_Cal[cable_num];
            delete[] cablen_vel[cable_num];
            delete[] cablen_acc[cable_num];
        }
        delete[] cablen;
        delete[] cablen_Cal;
        delete[] cablen_vel;
        delete[] cablen_acc;
    }
    void joints_Interpolation(std::list<jointState> &plan_I, std::list<jointState> &plan_O, int &num) {

        int t_Num, t_Num_Cau;
        double t_Min, t_Max, t_Tmp, duration_All;
        double duration_Tmp = 0;
        jointState cs;
        std::list<jointState>::iterator iter;

        t_Num = static_cast<int>(plan_I.size());  //plan_I的容量

        double *t_Array = new double[t_Num];
        double **jointn = new double* [Controllable_DOF_num_all]; //Controllable_DOF_num_all * t_Num的矩阵
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            jointn[i] = new double[t_Num];
        }

        int i = 0, j = 0;
        for( iter = plan_I.begin(); iter != plan_I.end(); iter++) {
            t_Array[i] = (iter->t);
            for(j = 0; j < Controllable_DOF_num_all; j++) {
                jointn[j][i] = (iter->j[j]);
            }
            i++;
        }
        t_Min = t_Array[0];
        t_Max = t_Array[t_Num - 1];
        duration_All = t_Max - t_Min;
        t_Num_Cau = static_cast<int>(duration_All / global_T_Interval_s) + 1 + 1; //强转int会截断

        double *t_Cau_Array = new double[t_Num_Cau];
        double **jointn_Cal = new double *[Controllable_DOF_num_all];
        double **jointn_vel = new double *[Controllable_DOF_num_all];
        double **jointn_acc = new double *[Controllable_DOF_num_all];

        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            jointn_Cal[i] = new double[t_Num_Cau];
            jointn_vel[i] = new double[t_Num_Cau];
            jointn_acc[i] = new double[t_Num_Cau];

        }

        for(int i = 0; i < t_Num_Cau - 1; i++) {
            t_Tmp = t_Min + i * global_T_Interval_s;
            t_Cau_Array[i] = t_Tmp;
        }
        t_Cau_Array[t_Num_Cau - 1] = t_Array[t_Num - 1];

        for(j = 0; j < Controllable_DOF_num_all; j++) {
            //各个参数分别是：插值前的时间序列、单一自由度的关节角序列、时间节点数量，插值后的时间序列，需要获得的关节角、角速度、角加速度，插值后的时间节点数量
            single_Interpolation_1(t_Array, jointn[j], t_Num, t_Cau_Array, jointn_Cal[j], jointn_vel[j], jointn_acc[j], t_Num_Cau);
        }
        for(int i = 0; i < t_Num_Cau; i++) {
            cs.t = t_Cau_Array[i];

            for(j = 0; j < Controllable_DOF_num_all; j++) {
                cs.j[j] = jointn_Cal[j][i];
                cs.vel[j] = jointn_vel[j][i];
                cs.acc[j] = jointn_acc[j][i];
                //std::ofstream ofile;
                //std::string bb = " ";
                //bb = std::to_string(static_cast<long long>(j));
                //ofile.open("/home/rxb/ljx_ws/rxb_MultiLevelLink/src/rxb_hardware_interface/traj/out/jointanglex_e/jointlog_e"+bb+".txt",std::ios::app);
                //ofile << cs.j[j] << ' ';
                //ofile.close();
            }

            if(i != 0) {
                plan_O.push_back(cs);
            }
            duration_Tmp = cs.t;
        }
        num = t_Num_Cau;

        delete[] t_Array;
        delete[] t_Cau_Array;
        for(int cable_num = 0; cable_num < Controllable_DOF_num_all; cable_num++) {
            delete[] jointn[cable_num];
            delete[] jointn_Cal[cable_num];
            delete[] jointn_vel[cable_num];
            delete[] jointn_acc[cable_num];
        }
        delete[] jointn;
        delete[] jointn_Cal;
        delete[] jointn_vel;
        delete[] jointn_acc;

    }

    //各个参数分别是：插值前的时间序列、单一自由度的关节角序列、时间节点数量，插值后的时间序列，需要获得的关节角、角速度、角加速度，插值后的时间节点数量
    void single_Interpolation_1(double *t_Array, double *joint, int num, double *t_Cau_Array, double *joint_Cau,  double *joint_vel, double *joint_acc, int num_Cau) {
        double leftBound = 0, rightBound = 0;
        SplineSpace::Spline sp(t_Array, joint, num, SplineSpace::GivenFirstOrder, leftBound, rightBound);
        sp.MultiPointInterp(t_Cau_Array, num_Cau, joint_Cau, joint_vel, joint_acc);

    }


    void joints_SimpleInterpolation(int num, jointState &joints1, jointState &jointsn, std::list<jointState> &RXB_joint_plan) {
        jointState js_tmp;
        jointState js0 = joints1;
        jointState jsn = jointsn;
        // std::cout << "joint's interpolation:" << std::endl;
        RXB_joint_plan.clear();
        for(int i = 0; i < num; i++) {
            for(int j = 0; j < Controllable_DOF_num_all; j++) {
                js_tmp.j[j] = (jsn.j[j] - js0.j[j]) / (num - 1) * i + js0.j[j];
            }
            js_tmp.t = (jsn.t - js0.t) / (num - 1) * i + js0.t;
            js_tmp.duration = (jsn.t - js0.t) / (num - 1);
            RXB_joint_plan.push_back(js_tmp);
        }
    }



}
