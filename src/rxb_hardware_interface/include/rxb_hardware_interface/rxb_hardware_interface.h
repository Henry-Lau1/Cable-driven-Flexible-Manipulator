#ifndef __RXB_HARDWARE_INTERFACE
#define __RXB_HARDWARE_INTERFACE

#include "global.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
//实施action servor的标准库
#include <actionlib/server/simple_action_server.h>
// 这个头文件用于存action文件
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <list>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include "rxb_algorithm.h"
#include "motor_driver.h"
#include "rxb_close_control.h"
#include "rxb_open_control.h"
#include "rxb_kinematics.h"
#include "usbcan.h"
#include "modern_robotics.h"
//#include "rxb_msgs/motorRead.h"
#include "rxb_msgs/RxbMode.h"
#include "rxb_msgs/MotorMode.h"
#include "rxb_msgs/RxbPose.h"
#include "rxb_msgs/RxbPoseCmd.h"
#include "rxb_msgs/sensorRead.h"
#include "rxb_msgs/serialCommu.h"
#include "rxb_msgs/forceSenseResult.h"
#include "rxb_msgs/neuralNetworkCommand.h"
#include "rxb_msgs/jointControl.h"
#include "rxb_msgs/taskSpaceCtrl_singleSeg.h"
#include "rxb_msgs/trajectoryDemo.h"
#include "rxb_msgs/corCableForceNN.h"
#include <time.h>
#include <random>

using namespace Eigen;

namespace RxbHardwareInterface {

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectoryServer;

    class RxbHardwareInterface {
    public:
        RxbHardwareInterface(ros::NodeHandle n);
        void mainLoop();

    private:
        ifstream ifs;
        ofstream ofs;

        double deltaJoint[Controllable_DOF_num_all];

        jointState rxb_setJointState;
        jointState rxb_priorJointState; //前一刻的构型角值
        cableState rxb_setCableState;
        cableState rxb_priorCableState;
        jointState rxb_currentJointState_Controllable_DOF;//Controllable_DOF 当前 规划 理想关节角
        rxbState rxb_gene_currentJointState;

        sensor_msgs::JointState jointsCurrent;	/* the current joints */

        jointState ForPos_HybridtargetJoint;

        jointState ForPos_Hybrid_targetJoint;
        jointState oloop_targetJoint;//目标 开环 规划 理想关节角
        jointState cloop_targetJoint;
        jointState cloop_currentJoint;
        cableState cloop_currentCablelen;
        cableState cloop_targetCablelen;
        uint cloop_count;
        std::vector<cableState> cable_plan_I;
        ros::Publisher SensorDataPublisher;
        ros::Publisher moveit_PubJoints; // publish当前角度
        ros::Timer timer;
        ros::Timer timer2;
        ros::Publisher velPub;
        ros::Publisher lenPub; //发布理论值
        ros::Publisher JntPub;

        bool pause_flag;
        ros::Subscriber pauseSub_;

        ros::Publisher PlaningJntPos_Pub;
//        ros::Publisher MotVel_Pub;
//        ros::Publisher MotAcc_Pub;
        //ros::Publisher JntEncoderPos_Pub;
        //  ros::Publisher MotorMode_Pub;
        ros::Publisher serial_encoder_pub;
        ros::Subscriber Encoder_zero_sub;
        ros::Subscriber flagSubscriber_; //in the contructor function
        ros::Subscriber recordFlagSubscriber_;
        // ros::Subscriber motorModeSubscriber_;
        ros::Subscriber motorMoveSubscriber_;
        //  ros::Subscriber rxbModeSubscriber_;

        ros::ServiceServer RxbModeServer_;
        ros::ServiceServer MotorModeServer_;
        ros::ServiceServer getRxbPoseServer_;
        ros::Subscriber TrajectorySubscriber_;
        /*  openloop cmd  */
        ros::Subscriber oloop_targetJoint_Sub;
        ros::ServiceServer targetPoseServer_;

        /* closeloop cmd */
        ros::Subscriber cloop_targetJoint_Sub;
        ros::Subscriber ForPos_Hybrid_targetJoint_Sub;
        //   ros::Subscriber matlab_CLoopSubscriber_;
        ros::Publisher serial_pub;
        ros::Subscriber serial_sub;
        ros::Subscriber J1PosPID_sub;
        ros::Subscriber J1PosPIDWriteIn_sub;
        ros::Subscriber ForcePIDWriteIn_sub;

        std_msgs::Float64MultiArray Info_MotPos;
        std_msgs::Float64MultiArray Info_MotVel;
        std_msgs::Float64MultiArray Info_MotAcc;
        std_msgs::Float64MultiArray Info_CableForce;
        std_msgs::Float64MultiArray Info_Controllable_DOF_JntAngle;

        ros::NodeHandle nh_;
        trajectoryServer as_;

        std::list<jointState> RXB_joint_plan;
        std::list<cableState> RXB_Cable_Plan;

        ros::AsyncSpinner spinner;
        bool compensate_check_flag;

        //关节段位置闭环控制量
        double DistanceError[Controllable_DOF_num_all] = {0};
        double dDistanceError[Controllable_DOF_num_all] = {0};
        double iDistanceError[Controllable_DOF_num_all] = {0};
        double segPID_out[Controllable_DOF_num_all] = {0};

        double segPID_kp[Controllable_DOF_num_all] = {8, 8};
        double segPID_ki[Controllable_DOF_num_all] = {0, 0};
        double segPID_kd[Controllable_DOF_num_all] = {2, 2};

        double seg_g_equ_JointAngle[Controllable_DOF_num_all] = {0};
        double segJointAngle_pid_weight[RXBSegment_Joint_num] = {0.14285714 , 0.14285714 , 0.14285714 , 0.14285714 , 0.14285714 , 0.14285714 , 0.14285714};//加起来等于1 不能赋值1/7

        //关节位置闭环控制量
        double Joint_pid_kp_FirstGroup[ModuJnt_num_all] = {12};
        double Joint_pid_ki_FirstGroup[ModuJnt_num_all] = {0};
        double Joint_pid_kd_FirstGroup[ModuJnt_num_all] = {2};
        // 关节速度 e de ie
        double error_joint_FirstGroup[Controllable_DOF_num_all] = {0};
        double derror_joint_FirstGroup[Controllable_DOF_num_all] = {0};
        double ierror_joint_FirstGroup[Controllable_DOF_num_all] = {0};
        double Joint_pid_out_FirstGroup[Controllable_DOF_num_all] = {0};

        double g_equ_JointAngle_FirstGroup[Controllable_DOF_num_all];
        double JointAngle_pid_weight_FirstGroup[RXBSegment_Joint_num/2] = {1/(double(RXBSegment_Joint_num)/2), 1/(double(RXBSegment_Joint_num)/2), 1/(double(RXBSegment_Joint_num)/2), 1/(double(RXBSegment_Joint_num)/2)};//等效关节角加权值，现在的权值相当于求平均值

        double error_joint_SecondGroup[Controllable_DOF_num_all] = {0};
        double derror_joint_SecondGroup[Controllable_DOF_num_all] = {0};
        double ierror_joint_SecondGroup[Controllable_DOF_num_all] = {0};
        double Joint_pid_out_SecondGroup[Controllable_DOF_num_all] = {0};

        double g_equ_JointAngle_SecondGroup[Controllable_DOF_num_all];

        //关节力闭环控制量
        double Force_pid_kp[DriveGroup_num*ModuJnt_num_all] = {10};//每组两个驱动的pid参数 {80 , 60};
        double Force_pid_ki[DriveGroup_num*ModuJnt_num_all] = {0};//{0 , 0};
        double Force_pid_kd[DriveGroup_num*ModuJnt_num_all] = {0};//{0 , 0};
        //
        double error_force[DriveGroup_num*ModuJnt_num_all] = {0};
        double derror_force[DriveGroup_num*ModuJnt_num_all] = {0};
        double ierror_force[DriveGroup_num*ModuJnt_num_all] = {0};
        double Force_pid_out[DriveGroup_num*ModuJnt_num_all] = {0};

        int g_ConstForceCable_ID[DriveGroup_num*ModuJnt_num_all] = {0};//从基座到末端每3根绳索中的恒力控绳索号，从0开始  {0,3}
        double g_expect_ConstCableForce[DriveGroup_num*ModuJnt_num_all] = {20};//N   {50,40}

        double motorVel_ad2_ConstForceCable[DriveGroup_num*ModuJnt_num_all];

        //段闭环方式2 --> 主要段闭环方式
        double seg_error_joint[Controllable_DOF_num_all] ={0};
        double seg_derror_joint[Controllable_DOF_num_all] ={0};
        double seg_ierror_joint[Controllable_DOF_num_all] ={0};
        double seg_pid_out_joint[Controllable_DOF_num_all] = {0};

        double seg_pid_kp_joint[Controllable_DOF_num_all] = {8, 8};
        double seg_pid_ki_joint[Controllable_DOF_num_all] = {0, 0};
        double seg_pid_kd_joint[Controllable_DOF_num_all] = {2, 2};

        double seg_gEqu_JointAngle[Controllable_DOF_num_all];
        double Pos_A_weight[2] = {1.0, 1.0};//{1, 57.3}

        void getSensorData();

        void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
        void timerCallback(const ros::TimerEvent &);

        void rxbCommandFlagCallback(const std_msgs::UInt32::ConstPtr &);
        void recordFlagCallback(const std_msgs::UInt32::ConstPtr &);
        void motorModeCallback(const std_msgs::UInt32::ConstPtr &);
        void motorMoveCallback(const std_msgs::UInt32::ConstPtr &);
        void rxbModeCallback(const std_msgs::UInt32::ConstPtr &);

        void matlab_CLoopCallback(const std_msgs::Float64MultiArray::ConstPtr &);
        void closeLoopJntCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &);
        //void openLoopPoseCmdCallback(const geometry_msgs::Pose::ConstPtr &);
        void openLoopJntCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &);
        void ForPos_HybridJntCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &cmd);

        bool targetPoseCmdCallback(rxb_msgs::RxbPoseCmd::Request &req, rxb_msgs::RxbPoseCmd::Response &res);
        bool motorModeFun(rxb_msgs::MotorMode::Request &req, rxb_msgs::MotorMode::Response &res);
        bool rxbModeFun(rxb_msgs::RxbMode::Request &req, rxb_msgs::RxbMode::Response &res);
        bool getRxbPose(rxb_msgs::RxbPose::Request &req, rxb_msgs::RxbPose::Response &res);
        void rxbTrajectoryFun(const std_msgs::UInt32::ConstPtr &);
        void pauseFlagCallback(const std_msgs::UInt32::ConstPtr& );
        void Encoder_zero_subFun(const std_msgs::UInt32::ConstPtr& data);
        void serial_data_subFun(const rxb_msgs::sensorRead::ConstPtr &data);
        void Sensordata_timerCallback(const ros::TimerEvent &);
        void communicateMatlab();
        void communicateMoveit();
        void writeHW();
        void readHW();
        void init();
        void J1PosPIDCallback(const std_msgs::Float64MultiArray::ConstPtr &datas);
        void J1PosPIDWriteInCallback(const std_msgs::Float64MultiArray::ConstPtr &datas);
        void ForcePIDWriteInCallback(const std_msgs::Float64MultiArray::ConstPtr &datas);
        void JointOloop_Control();
        void Const_CableForce_Control();
        void JointCloop_Control();
        void JointHybrid_Control();
        void TaskSpace_Control();
        void ConstForceCableChoose();

        //机械臂随机运动
        ros::Subscriber randomMotion_sub;
        void randomMotionCallback(const std_msgs::UInt32::ConstPtr &data);
        void randomMotionFunc(void);

        //用于力感知实验
        ros::Subscriber exForceSensor_sub;
        ros::Subscriber forceSenseSub;
        geometry_msgs::Twist exForceMeasured;
        double forceSense[Controllable_DOF_num_all];
        void exForceSensor_callbackFunc(const geometry_msgs::Twist::ConstPtr &data);
        void forceSense_callbackFunc(const rxb_msgs::forceSenseResult::ConstPtr &data);

        //用于导纳控制
        ros::Subscriber admittanceControlSwitch_sub;
        struct State {
            double x;  // 位移
            double v;  // 速度
        };
        bool admittanceControlIsOn = false;
        float M = 1, B = 2, K = 0;
        State acState[Controllable_DOF_num_all];
        void AdmittanceControlSwitch_CallbackFunc(const std_msgs::UInt32::ConstPtr& data);
        void AdmittanceControl(double forceSense[], State state[], float M, float B, float K, double dt);
        State MBKEquation(const State& state, float M, float B, float K, double F);
        ros::Subscriber admittanceControl_seg1Set_sub;
        void AdmittanceControl_Seg1Set_SubFunc(const std_msgs::Float32MultiArray::ConstPtr& data);

        // 用于神经网络训练
        ros::Subscriber nnTrainingNewRcSub;
        ros::Subscriber nnTrainingStartSub;
        ros::Publisher nnTrainingProgressPub;
        void NNTrainingNewRc_CallbackFunc(const std_msgs::UInt8::ConstPtr& data);
        void NNTrainingStart_CallbackFunc(const rxb_msgs::neuralNetworkCommand::ConstPtr& data);
        bool nnTrainingNewRcFlag = false;
        bool nnTrainingStartFlag = false;
        unsigned int rcCount = 0;
        unsigned int extension = 0;

        // 用于初始化机械臂是同步ui界面参数
        ros::Publisher uiUpdatePosePID_pub;

        // 用于任务空间控制
        unsigned int taskSpaceCtrl_mode = 1;
        bool taskSpaceCtrl_firstFlag = true;
        ros::Subscriber taskSpaceCtrl_mode_sub;
        void TaskSpaceCtrl_Mode_SubFunc(const std_msgs::UInt8::ConstPtr& data);
        ros::Subscriber taskSpaceCtrl_move_sub;
        void TaskSpaceCtrl_Move_SubFunc(const std_msgs::Float64MultiArray::ConstPtr& data);
        ros::ServiceClient taskSpaceCtrl_singleSeg_client;
        vector<double> desiredJoint;

        // 补充基础功能
        Eigen::MatrixXd actualT0;
        Eigen::MatrixXd actualT;
        vector<Eigen::VectorXd> BList;
        ros::Publisher actualT_pub;
        void ActualForwardKinematic(const double jointAngle[]);
        class SlidingWindowSmooth{//滑动平滑算法对象，要求能够直接处理向量
        private:
          const string name;
          const unsigned long dim;  // 数据维度
          const unsigned long windowSize = 5;  // 窗口大小
          deque<vector<double>> dq; // 记录数据，进行平滑处理
          vector<double> sum; // 用于提高算法效率

        public:
          vector<double> result;  // 保存处理结果

          SlidingWindowSmooth(const string& name_, unsigned long dim);  //构造函数
          void AddData(const double* const data_);  //加入数据，更新结果
          void Disp(void) const;
        };
        class KalmanFilter{// 卡尔曼滤波算法对象，没有输入，平滑作用，要求能够直接处理向量输入
        private:
          // 不需要更新的常量
          const string name;
          const unsigned long dim;  //数据维度

          Eigen::Matrix3d A; // 状态转移矩阵
          Eigen::Matrix<double, 1, 3> H; // 观测矩阵
          Eigen::Matrix3d Q; // 过程噪声协方差
          Eigen::Matrix<double, 1, 1> R; // 测量噪声协方差

          // 需要更新的变量
          vector<Eigen::Vector3d> x; // 状态向量
          vector<Eigen::Matrix3d> P; // 状态协方差

          void Predict(void); //预测步骤
          void Update(const vector<double>& z_); // 更新步骤

        public:
          vector<Eigen::Vector3d> result;

          KalmanFilter(const string& name_, const unsigned long dim_, const vector<double>& Q_);
          void AddData(const vector<double>& z);
          void Disp(void) const;
        };
        SlidingWindowSmooth jointAngleSmooth;
        KalmanFilter jointAngleKalman;
        MatrixXd bodyJacobian;
        void CalBodyJacobian(const vector<VectorXd>& BList, const double* const alpha);
        VectorXd bodyTwist;
        void CalBodyTwist(const MatrixXd& bodyJacobian, const vector<Vector3d>& jointAngleState);
        ros::Publisher bodyTwist_pub;
        bool trajectoryDemo_flag = false;
        ros::Subscriber trajectoryDemo_sub;
        void TrajectoryDemo_SubFunc(const rxb_msgs::trajectoryDemo::ConstPtr& data);
        void TrajectoryDemo_SegmentalPlan(list<jointState>& jointPoint, const vector<double>& timeGap, list<jointState>& jointPlan);

        /// 在线神经网络训练
        class SlidingWindow{  // 滑动窗口
        private:
            const unsigned long dim;  // 数据维度
            const unsigned long windowSize;  // 窗口大小
            deque<vector<double>> rc; // 记录历史数据
        public:
            SlidingWindow(unsigned int dim_, unsigned int windowSize_); // 构造函数
            void AddData(const vector<double>& data);  //加入数据，更新结果
            vector<double> GetResult(void); // 将滑动窗口中的数据按时间顺序组成向量输出，要求必须填满滑动窗口才能输出，否则输出空数组
        };
        bool neuralNetwork_corCableForce_flag = false;
        ros::Subscriber neuralNetwork_corCableForceStart_sub;
        void NeuralNetwork_CorCableForceStart_SubFunc(const std_msgs::Bool::ConstPtr& data);
        ros::Subscriber neuralNetwork_corCableForceStop_sub;
        void NeuralNetwork_CorCableForceStop_SubFunc(const std_msgs::Bool::ConstPtr& data);
        ros::Publisher neuralNetwork_corCableForceTrain_pub;
        unsigned char neuralNetwork_corCableForce_dataRecord_stateFlag = 0; // 标志状态，0-未采集数据，1-采集训练数据，2-采集验证数据，3-采集测试数据
        unsigned int neuralNetwork_corCableForce_dataRecord_count = 0;  // 记录已采集数据数量，进行可视化
        ros::Subscriber neuralNetwork_corCableForce_dataRecord_start_sub;
        void NeuralNetwork_CorCableForce_DataRecord_Start_SubFunc(const std_msgs::UInt8::ConstPtr& data);
        ros::Subscriber neuralNetwork_corCableForce_dataRecord_stop_sub;
        void NeuralNetwork_CorCableForce_DataRecord_Stop_SubFunc(const std_msgs::UInt8::ConstPtr& data);
        ros::Publisher neuralNetwork_corCableForce_dataRecord_count_pub;
        SlidingWindow neuralNetwork_corCableForce_dataRecord_sw;
        bool neuralNetwork_corCableForce_model_dev_state = false;   // dev启用标志，0-未启用，1-启用
        ros::Subscriber neuralNetwork_corCableForce_model_dev_state_sub;
        void NeuralNetwork_CorCableForce_Model_Dev_State_SubFunc(const std_msgs::Bool::ConstPtr& data);
        ros::ServiceClient neuralNetwork_corCableForce_model_client;
        ros::Publisher neuralNetwork_corCableForce_model_dev_data_pub;
    };

}

#endif
