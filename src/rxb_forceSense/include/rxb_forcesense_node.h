#ifndef RXB_FORCESENSE_NODE_H
#define RXB_FORCESENSE_NODE_H

// 包含ros头文件，用于创建节点、句柄、消息交互
#include <ros/ros.h>

// 包含rxb消息类型
#include "rxb_msgs/sensorRead.h"
#include "rxb_msgs/forceSenseResult.h"
#include "std_msgs/UInt32.h"

// 包含C++头文件
#include <iostream>
#include <vector>
#include <thread>
#include <deque>
#include <cmath>

// 包含Eigen库
#include <Eigen/Dense>
#include <Eigen/StdVector>

// 包含modern robotics库
#include "modern_robotics.h"

using namespace std;
using namespace Eigen;

namespace rxb_forcesense{  
  // 全局变量
  extern bool rxb_forcesense_node_running;

  // 函数
  double deg2rad(double degrees);
  vector<double>& deg2rad(vector<double>& degrees);
  double rad2deg(double radians);

  // 机械臂参数及数据
  class Database{
  private:
    // ros句柄
    ros::NodeHandle nh;

    // 订阅者
    ros::Subscriber jointAngleMeasured_sub;
    ros::Subscriber cableForceMeasured_sub;

    // 异步多线程
    ros::AsyncSpinner spinner;

  public:
    //机械臂系统常量
    const unsigned long segNum = 1; // 关节段数量
    const unsigned long jointNum = 8;  //单段关节数量
    const double dt = 0.02; //控制周期，单位：秒

    const double L_base = 95.5/1000;  // 基座连杆长度，单位：米
    const double L = 125.0/1000;  // 中间连杆长度，单位：米
    const double L_ex = (125.0/2 + 41.2053)/1000;  // 末端六维力传感器装配，进行补偿后的质心距离，单位：米
    const double L_disk = 27.0/1000;  // 十字轴处布线圆盘之间的距离的一半（与matlab代码统一），单位：米
    const double r_hole = 48.0/1000;  // 布线圆孔的半径，单位：米

    vector<double> mass;  // 质量，单位：千克
    vector<Matrix<double, 3, 3>> inertia; // 惯性矩阵，量纲：千克，米
    vector<Matrix<double, 6, 6>> GIM; // 广义惯性矩阵，量纲：千克，米

    // 保存其他功能包传过来的数据，全局变量
    // 关节角，单位：弧度
    vector<double> jointAngleMeasured;
    // 驱动绳拉力，单位：牛顿
    vector<double> actCableForceMeasured;
    // 联动绳拉力，单位：牛顿
    vector<double> corCableForceMeasured;

    // 构造函数
    Database(const ros::NodeHandle& nh_);
    // 接收数据函数
    void JointAngleMeasured_ReceiveFunc(const rxb_msgs::sensorRead::ConstPtr& jointAngle);
    void CableForceMeasured_ReceiveFunc(const rxb_msgs::sensorRead::ConstPtr& cableForce);
    // 打印函数
    void Disp(void) const;
    // 设置质量和惯性矩阵
    void SetMassInertia(void);
  };

  //滑动平滑算法对象，要求能够直接处理向量
  class SlidingWindowSmooth{
  private:
    const string name;
    const unsigned long windowSize = 5;  // 窗口大小
    deque<vector<double>> dq; // 记录数据，进行平滑处理
    vector<double> sum; // 用于提高算法效率

  public:
    const unsigned long dim;  // 数据维度
    vector<double> result;  // 保存处理结果

    SlidingWindowSmooth(const string& name_, unsigned long dim);  //构造函数
    void AddData(const vector<double>& data);  //加入数据，更新结果
    void Disp(void) const;
  };

  // 卡尔曼滤波算法对象，没有输入，平滑作用，要求能够直接处理向量输入
  class KalmanFilter{
  private:
    // 不需要更新的常量
    const string name;
    const unsigned long dim;  //数据维度

    Matrix3d A; // 状态转移矩阵
    Matrix<double, 1, 3> H; // 观测矩阵
    Matrix3d Q; // 过程噪声协方差
    Matrix<double, 1, 1> R; // 测量噪声协方差

    // 需要更新的变量
    vector<Vector3d> x; // 状态向量
    vector<Matrix3d> P; // 状态协方差

    void Predict(void); //预测步骤
    void Update(const vector<double>& z_); // 更新步骤

  public:
    vector<Vector3d> result;

    KalmanFilter(const string& name_, const unsigned long dim_, const Database& db, const vector<double>& startValue, const vector<double>& Q_);
    void AddData(const SlidingWindowSmooth& sws);
    void Disp(void) const;
  };

  // 描述机械臂主体状态对象
  class BodyState{
  private:
    // 不需要更新的常量
    vector<Matrix<double, 4, 4>> M; // 零位状态下，坐标系{i}到坐标系{i-1}的齐次变换矩阵
    Matrix<double, 6, 1> vel_0; // 基座连杆速度旋量，坐标系基座连杆
    Matrix<double, 6, 1> acc_0; // 基座连杆加速度旋量，坐标系基座连杆
    vector<Matrix<double, 6, 1>> axis;  // 关节i旋量表示，坐标系{i}
    vector<Matrix<double, 6, 6>> GIM; // 刚体广义惯性矩阵，量纲：千克，米

    // 需要更新的变量
    unsigned long dim;  // 数据维度，等于机械臂刚体数量
    vector<Matrix<double, 6, 1>> vel; // 刚体i速度旋量，坐标系{i}，量纲：米，秒，弧度
    vector<Matrix<double, 6, 1>> acc; // 刚体i加速度旋量，坐标系{i}
    vector<Matrix<double, 4, 4>> T; // 非零位状态下，坐标系{i}到坐标系{i-1}的齐次变换矩阵

    void CalT(const KalmanFilter &thetaKF);
    void CalVel(const KalmanFilter &thetaKF);
    void CalAcc(const KalmanFilter &thetaKF);
    void CalGF(void);

  public:
    vector<Matrix<double, 6, 1>> GF;  // 广义力，量纲：牛顿，米

    BodyState(const Database& db);
    void AddData(const KalmanFilter &thetaKF);
    void Disp(void) const;
  };

  // 描述驱动绳状态的对象，要求能够直接处理向量
  class ActCable{
  private:
    // 初始化后不需要更改的常量
    unsigned long dim;  // 对象维度，等于驱动绳数量
    unsigned long segNum; // 机械臂段的数量
    unsigned long jointNum; // 单段关节数量
    vector<unsigned long> segControl; // 驱动绳会影响的段的个数
    double L_disk;  // 十字轴处布线圆盘之间的距离的一半（与matlab代码统一），单位：米
    double r_hole;  // 布线圆孔的半径，单位：米
    Matrix<double, 4, 4> M; // 初始状态下十字轴处，靠近基座的绳孔到靠近末端的绳孔的齐次变换矩阵，量纲：米
    vector<Matrix<double, 6, 1>> S_Y; // 初始状态下，对于靠近基座的绳孔坐标系，Y轴旋转轴的旋量表示，量纲：米，弧度
    vector<Matrix<double, 6, 1>> S_P; // 初始状态下，对于靠近基座的绳孔坐标系，P轴旋转轴的旋量表示，量纲：米，弧度
    double dt;  // 控制周期，单位：秒
    double K_v = 0.28;  // 摩擦力粘滞系数
    double sigma = 10e3; //刚度系数

    // 需要更改的变量
    vector<vector<double>> dis2End; // 当前时刻，驱动绳各接触点到末端的路径长度，单位：米
    vector<vector<double>> dis2End_last;  // 上一时刻，驱动绳各接触点到末端的路径长度，单位：米
    vector<vector<double>> speed2End; // 当前时刻，驱动绳各接触点到末端的路径长度变化速率，单位：米/秒
    vector<vector<double>> cableAngle; // 驱动绳各段与布绳圆盘法线的夹角，单位：弧度
    vector<double> cableMeasured; // 驱动绳一端测量的拉力，单位：牛顿

    // 私有成员函数
    vector<double> CalCableDis_YP(const double theta_P, const double theta_Y, const unsigned long ca_num);
    vector<double> CalCableDis_PY(const double theta_P, const double theta_Y, const unsigned long ca_num);

  public:
    vector<double> holeAngle; // 绳孔角度，单位：弧度

    vector<vector<Vector3d>> cableDir;  // 驱动绳各段方向，注意：相对于不同的坐标系
    vector<vector<double>> cableTension;  // 驱动绳各段拉力，单位：牛顿
    vector<vector<double>> cableFriction; // 驱动绳各接触点所受摩擦力，单位：牛顿

    ActCable(const Database& db, const KalmanFilter& thetaKF, const vector<double>& holeAngle_);
    void CalCableDis(const KalmanFilter& thetaKF);
    void CalCableTen(const KalmanFilter& forceKF);
    void Disp(void);
  };

  // 描述联动绳状态的对象，要求能够直接处理变量
  class BCorCable{
  private:
    // 初始化后不需要更改的成员变量
    unsigned long dim;  // 数据维度，实际上等于关节段数量的2倍
    unsigned long jointNum; // 单段的关节数量
    double dt;  // 控制周期
    const double wrapAngle_0 = deg2rad(31.07);  // 零位状态下，联动绳缠绕角度，单位：弧度
    const double mu = 0.2;  // 摩擦系数
    const double radius = 34.75e-3; // 绳盘半径，单位：米
    const double yita = 1e-2; // 粘滞系数
    const double sigma = 10e3;  // 刚度系数

    // 需要更改的成员变量
    vector<double> wrapAngle; // 当前时刻，联动绳缠绕角度，单位：弧度
    vector<double> wrapAngle_last;  // 上一时刻，联动绳缠绕角度，单位：弧度
    vector<double> cableSpeed;  // 联动绳相对速率，单位：米每秒
    vector<double> cableMeasured; // 联动绳一端测量的拉力，单位：牛顿

    // 私有成员函数
    void UpdateInf(const KalmanFilter& thetaKF, const KalmanFilter& forceKF);

  public:
    vector<double> cableTension;  // 联动绳经过绳盘后的绳索拉力，单位：牛顿
    vector<double> cableFriction; // 联动绳经过绳盘受到的摩擦力，单位：牛顿

    BCorCable(const Database& db, const KalmanFilter& thetaKF, const KalmanFilter& forceKF);
    void CalCableTen(const KalmanFilter& thetaKF, const KalmanFilter& forceKF);
    void Disp(void);
  };

  // 描述动力学方程的对象，通过matalb获得的解析式直接得到系数矩阵A和非齐次向量b，每段一个对象相互解耦
  class DynamicEquation{
  private:
    // ros句柄
    ros::NodeHandle nh;

    // 话题相关
    ros::Publisher forceSenseResult_pub;
    ros::Subscriber forceSenseZero_sub;

    // 初始化后不需要更改的成员变量
    unsigned long segNum; // 机械臂段的数量
    unsigned long jointNum; // 单段关节数量
    unsigned long segID; // 动力学模型对应的段，从0开始计数
    double holeRadius;
    vector<double> holeAngle;
    const double bColOutAngle_0 = deg2rad(65); // 零位状态下，大8联动绳离开绳盘法线与水平线的夹角，单位：弧度
    const double bColInAngle_0 = deg2rad(33.93);  // 零位状态下，大8联动绳进入绳盘法线与水平线的夹角，单位：弧度
    const double sColOutAngle_0 = deg2rad(101.16);  // 零位状态下，小8联动绳离开绳盘法线与竖直线的夹角，单位：弧度
    const double bColRadius = 34.6/1000;  // 大8联动绳缠绕半径，单位：米
    const double sColRadius = 12.1/1000;  // 小8联动绳缠绕半径，单位：米
    const double bColDis = 24.3231/1000;  // 大8联动绳相交点到连杆质心距离，单位：米
    const double sColDis = 30.3/1000; // 小8联动绳缠绕平面到十字轴质心距离，单位：米
    const double mcDisComp = 41.2053/1000;  // 装载六维力传感器质心位置补偿，单位：米
    const double forcePointDisComp = 28.5/1000; // 末端力作用点位置补偿，单位：米
    const double linkLength = 62.5/1000;  // 连杆质心到铰链的距离，单位：米
    const double cableDiskDis = 35.5/1000;  // 连杆质心到布线圆盘的距离，单位：米
    vector<Vector3d> rActCable; // 连杆质心到各绳孔的向量

    // 需要更改的成员变量
    double theta_Y; // 单段最后一个Yaw关节转角，单位：弧度
    double theta_P; // 单段最后一个Pitch关节转角，单位：弧度
    double bColForce[2];  // 大8联动绳测量拉力，先正后负，单位：牛顿
    double bColTension[2];  // 大8联动绳受摩擦力后拉力，先正后负，单位：牛顿
    double sColForce[2];  // 小8联动绳测量拉力，先正后负，单位：牛顿
    vector<double> actTension;  // 相关驱动绳拉力，单位：牛顿
    vector<Vector3d> actCableDir; // 相关驱动绳方向
    Matrix<double, 6, 1> resGF[2]; // 广义力，量纲：牛顿，米
    Matrix<double, 6, 1> actGF;
    Matrix<double, 12, 12> A; // 系数矩阵A
    Matrix<double, 12, 1> b; //非齐次向量b
    Matrix<double, 12, 1> solution; // 线性方程组求解结果，最后两个元素分别为x和y方向上的力
    double result_0[2]; // 力感知结果初值

    // 私有成员函数
    void UpdateInf(const KalmanFilter& thetaKF, const KalmanFilter& forceKF, const BCorCable& bcc, const ActCable& ac, const BodyState& bs);
    void CalA(void);
    void Calb(void);
    void SolveEq(void);
    void Advertise(void);
    void ForceSenseZero_ReceiveFunc(const std_msgs::UInt32::ConstPtr& data);

  public:
    double result[2]; // 力感知结果，分别为x和y方向上的力

    DynamicEquation(const ros::NodeHandle& nh_, const Database& db, const ActCable& ac, unsigned long segID_);
    void ForceSense(const KalmanFilter &thetaKF, const KalmanFilter &forceKF, const BodyState &bs, const ActCable &ac, const BCorCable &bcc);
    void Disp(void);
  };

  // 与matlab对比，验证算法的正确性
  void Test(void);
}

#endif
