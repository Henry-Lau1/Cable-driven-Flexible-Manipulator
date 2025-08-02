#include "rxb_open_control.h"
#include <math.h>
#include <ros/ros.h>  //用于DEBUG的ROS info输出

namespace RxbHardwareInterface {

    void p2p_jointTrajectoryPlan(const jointState &curr_jnt, jointState &tar_jnt, std::list<jointState> &jnt_plan) {//根据当前规划关节角和目标规划关节角，进行关节角规划
    /*
     *  输入： curr_jnt，当前规划关节角
     *        tar_jnt，目标规划关节角
     *  输出： jnt_plan，关节空间轨迹列表，不包括初始点
     *
     */
        double max_delta_jnt = 0;//记录目标关节角数组与当前关节角数组中最大的元素
        for(unsigned long j = 0; j < Controllable_DOF_num_all; j++) { //找到变化最大的关节角，用于规划运动时间
            if(fabs(curr_jnt.j[j] - tar_jnt.j[j]) > max_delta_jnt) {
                max_delta_jnt = fabs(curr_jnt.j[j] - tar_jnt.j[j]);
            }
        }

        double duration = max_delta_jnt/(90/g_oloopT_s);
        if(duration <= 0.001) return;

        unsigned long points_num = static_cast<unsigned long>(duration / global_T_Interval_s) +1 +1;

        std::vector<std::vector<double>> jnts_cau(points_num, std::vector<double>(Controllable_DOF_num_all, 0));
        std::vector<std::vector<double>> jnts_vel(points_num, std::vector<double>(Controllable_DOF_num_all, 0));
        std::vector<std::vector<double>> jnts_acc(points_num, std::vector<double>(Controllable_DOF_num_all, 0));
        std::vector<double> t_cau(points_num, 0);

        for(unsigned long i = 0; i < points_num; i++) {
            t_cau[i] = global_T_Interval_s * i;
            if(t_cau[i] > duration) {
                t_cau[i] = duration;
            }
        }

        double jnt_beg = 0, jnt_end = 0;
        double jntV_beg = 0, jntV_end = 0;
        double a0, a1, a2, a3;
        for(unsigned long j = 0; j < Controllable_DOF_num_all; j++) {
            jnt_beg = curr_jnt.j[j];
            jnt_end = tar_jnt.j[j];
            a0 = jnt_beg;
            a1 = jntV_beg;
            a2 = 3 * (jnt_end - jnt_beg) / (duration * duration) - (2 * jntV_beg) / duration - jntV_end / duration;
            a3 = 2 * (jnt_beg - jnt_end) / (duration * duration * duration) + (jntV_beg + jntV_end) / (duration * duration);
            for(unsigned long i = 0; i < points_num; i++) {
                jnts_cau[i][j] = a0 + (a1 + (a2 + a3 * t_cau[i]) * t_cau[i]) * t_cau[i]; //deg
                jnts_vel[i][j] = a1 + (2 * a2 + 3 * a3 * t_cau[i]) * t_cau[i]; //deg/s 新加
                jnts_acc[i][j] = 2 * a2 + 6 * a3 * t_cau[i];//deg/(s^2)新加
            }
        }

        double t_tmp = 0;
        jointState joint_tmp;
        for(unsigned long i = 0; i < points_num; i++) { //ignore the first point
            for(unsigned long j = 0; j < Controllable_DOF_num_all; j++) {
                joint_tmp.j[j] = jnts_cau[i][j];
                joint_tmp.vel[j] = jnts_vel[i][j];
                joint_tmp.acc[j] = jnts_acc[i][j];
            }
            joint_tmp.t = curr_jnt.t + t_cau[i];
            joint_tmp.duration = t_cau[i] - t_tmp;
            if(i != 0) {
                jnt_plan.push_back(joint_tmp);
            }
            t_tmp = joint_tmp.t;
        }
    }

    //判断当前是否到达位置
    bool judgeArrival(const fp64 *now, fp64 *end,  fp64 ERR) {

    //根据最靠近基座的两个角进行补偿
      double err = 0.0;
      double eof[Controllable_DOF_num_all] = {1.0, 1.0};
      for(int i = 0; i < Controllable_DOF_num_all; i++) { //Controllable_DOF_num_all
          err += fabs(now[i] - end[i]) * eof[i];
      }

      if((err) > ERR){
         //std::cout << "OpenLoop Rerun: joint . sumERR: " << err << std::endl;
        for(int k = 0; k < 2; k++)//Controllable_DOF_num_all
        {
          //std::cout << now[k] - end[k] << "\t";
        }
        //std::cout << std::endl;
        return false;
      }
      return true;

/*
        double err = 0.0;
        double eof[Controllable_DOF_num_all] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        double theta_[] = {(now[0] + now[3] + now[4] + now[7])/4 , (now[1] + now[2] + now[5] + now[6])/4};

        for(int i = 0; i < 2; i++) { //Controllable_DOF_num_all
            err += fabs(theta_[i] - end[i]) * eof[i];
        }

        if((err) > ERR){
           std::cout << "OpenLoop Rerun: joint . sumERR: " << err << std::endl;
          for(int k = 0; k < 2; k++)//Controllable_DOF_num_all
          {
            std::cout << "theta_error" << k+1 << " : " << theta_[k] - end[k] << "\t";
          }
          std::cout << std::endl;
          return false;
        }
        fre = 0;
        return true;
        */

    }



}
