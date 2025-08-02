#ifndef RXB_TASKSPACE_CLOSELOOP_H
#define RXB_TASKSPACE_CLOSELOOP_H

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <fstream>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include "rxb_msgs/sensorRead.h"
#include "rxb_msgs/jointControl.h"
#include "rxb_msgs/taskSpaceCtrl_singleSeg.h"

#include "modern_robotics.h"

using namespace Eigen;

namespace RXB_TaskSpace_CloseLoop{

const double pi = std::acos(-1.0);  // 定义的pi值
const double dr = pi / 180;
const double rd = 180 / pi;

class SingleSegController
{
public:
    SingleSegController(ros::NodeHandle nh_);

private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle nh;

    ros::Publisher targetT_pub;
    ros::Publisher actualT_pub;
    ros::Publisher desiredJoint_pub;
    ros::Publisher desiredT_pub;
    ros::Publisher planJoint_pub;
    ros::Publisher errorTwist_pub;

    ros::Subscriber encoderData_sub;
    void EncoderData_SubFunc(const rxb_msgs::sensorRead::ConstPtr& data);
    ros::Subscriber FKCal_sub;
    void FKCal_SubFunc(const std_msgs::Float64MultiArray& data);

    ros::ServiceServer taskSpaceCtrl_singleSeg_server;
    bool TaskSpaceCtrl_SingleSeg_ServerFunc(rxb_msgs::taskSpaceCtrl_singleSeg::Request& req, rxb_msgs::taskSpaceCtrl_singleSeg::Response& res);

    const int poly_num_ws_fit = 12; // 工作空间拟合时的多项式次数
    const double RXB_l_m = 0.125; // 柔性臂杆长
    const int RXB_l_mm = 125; // 柔性臂杆长
    const int seg_num_ttl = 1; // 柔性臂段数
    const int encoder_num_all = 8; // 柔性臂总关节个数
    const int dof = 2;
    const double dt = 0.02; // 控制周期，单位：s

    Eigen::VectorXd jointAngleMeasured;
    Eigen::VectorXd jointAngleActualAverage;
    const Eigen::MatrixXd transformM = (Eigen::MatrixXd(4, 4) << 0, 0, 1, 0,
                                                                  0, -1, 0, 0,
                                                                  1, 0, 0, 0,
                                                                  0, 0, 0, 1).finished();

    Eigen::MatrixXd TransformationMatrix(double ai, double alpha, double di, double theta);
    Eigen::Matrix4d RXB_T_Segment(const double& l, const Eigen::VectorXd& theta, int flag_RXB_base = 0); // 函数原型给默认值，函数定义处不用给默认值
    void RXB_Project2IdealPlane_GM2(int file_input_flag,
                                    const Eigen::Vector3d& p_actual,
                                    Eigen::Vector3d& p_result,
                                    double& vertical_distance);
    void RXB_Project2IdealPlane_GM2(const Eigen::VectorXd& p_ws_fit,
                                    const Eigen::MatrixXd& sc_parameter,
                                    const Eigen::Vector3d& p_actual,
                                    Eigen::Vector3d& p_result,
                                    double& vertical_distance);
    double u(double t);
    double dudt(double t);
    double RXB_SegCL_P2Theta_NM(const Eigen::Vector3d& p_actual_projected,
                                const double& x0,
                                const double& e,
                                const int& N_max);
    Eigen::Matrix2d RXB_SegCL_JR1_Calculate(const double& l, const double& phi, const double& theta);
    void RXB_SegCL_phitheta2PY(const double& phi, const double& theta, double& qz1, double& qy2);
    Eigen::Matrix2d RXB_SegCL_JR2_Calculate(const double& q1, const double& q2);
    void RXB_SegCL_Compensate(Eigen::Vector3d& p_actual_projected,
                              Eigen::Vector3d& p_desire,
                              Eigen::Vector2d& q_equivalent,
                              Eigen::Vector2d& dq_compensate);
    void RXB_SegmentalCloseLoopControl(Eigen::Vector2d& theta_ideal_temp,
                                       Eigen::VectorXd& theta_actual,
                                       Eigen::Vector2d& dq_compensate);
};

}

#endif
