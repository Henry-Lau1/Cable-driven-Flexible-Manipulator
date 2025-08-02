#ifndef  _KINEMATICS_H
#define  _KINEMATICS_H

#include <vector>
#include <math.h>
#include "global.h"
#include <iostream>
//#include <algorithm>
#include <stdexcept>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <Eigen/src/Geometry/Transform.h>
#include <geometry_msgs/Pose.h>

namespace RxbHardwareInterface {

    geometry_msgs::Pose RotMat2Quatern(Eigen::Matrix4d T);
    Eigen::Matrix4d Quatern2RotMat(geometry_msgs::Pose pose);
    void fkd_kinematics(double *theta, std::vector<Eigen::Matrix4d> &);
    void fkd_actualKinematics(double *theta, std::vector<Eigen::Matrix4d> &Ti);
    void DH_table(double theta, int link_num, double link_len, Eigen::Vector4d &DH_Paras);
    void DH_Mat(double len, double alpha, double offset, double theta, Eigen::Matrix4d &tmp_Ti);
    Eigen::MatrixXd pinv(Eigen::MatrixXd  A);
    Eigen::Matrix<double, 6, Controllable_DOF_num_all>  Jacob_Mat(std::vector<Eigen::Matrix4d> &Ti);
    Eigen::Matrix<double, 6, Controllable_DOF_num_all> Jacob_Mat_Seg(std::vector<Eigen::Matrix4d> &Ti);
    Eigen::Matrix<double, 3, Controllable_DOF_num_all> Jacob_Mat_Seg_OnlyPos(std::vector<Eigen::Matrix4d> &Ti);
    void fkd_kinematics_oneSeg(double *theta, std::vector<Eigen::Matrix4d> &Ti);
    void fkd_actualKinematics_oneSeg(double *theta, std::vector<Eigen::Matrix4d> &Ti);
    Eigen::Matrix<double, 6, 2> Jacob_Mat_Seg_oneSeg(std::vector<Eigen::Matrix4d> &Ti);

    int inverse_kinematics(Eigen::Matrix4d &T_end, double *theta, double *ang_lim);
}

#endif // ! _KINEMATICS_H
