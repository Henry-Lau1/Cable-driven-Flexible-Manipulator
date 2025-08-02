#ifndef GLOBAL_H
#define GLOBAL_H

#include <list>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

#define DriveGroup_num 1 //每段有几组驱动
#define Gene_Jnt_Num 8 //所有关节数
#define Motor_num_all (DriveGroup_num*3*ModuJnt_num_all)//电机总数，默认一组驱动为3个电机，若为4个需要改动
#define RXBSegment_Joint_num 8 //多级联动段关节数量
#define Controllable_DOF_num_all 2 //可控自由度
#define ModuJnt_num_all 1//总段数
#define Mot_Reduction_Ratio 65.7222 //电机减速比  84.29358
#define Lead 4//导程
#define DB_Ratio 32861.1 //减速箱 DB_Ratio = (500*4)(qc)*Mot_Reduction/Lead 从减速前电机到丝杆滑块的减速比，即滑块每移动1mm减速前电机转动的qc（定滑轮）
#define Joint_CaptureCard_Num 2 //关节采集卡数量
#define CorCable_ForceSensor_Num 18
#define SCorCableForce_Num 6
#define BCorCableForce_Num 12

#define PI 3.1415926535
#define Deg2Rad   0.017453292519943/*---度转弧度----*/
#define Rad2Deg   57.295779513082323/*---弧度转度---*/

#define RxbMode_OLoop_Pos            0x1201
#define RxbMode_CLoop_Pos            0x1202
#define RxbMode_ForPos_Hybrid        0x1206
#define RxbMode_Adjust_Cable         0x1204
#define RxbMode_TaskSpace_Ctrl       0x1207

#define isRxbOnFlag                  0x1000
#define isRxbOffFlag                 0x1001
#define isWorking                    0x1011
#define isNWorking                   0x1012
#define workStatusChange             0x1013

#define isRecordOnFlag               0x1020
#define isRecordOffFlag              0x1021

#define TrajectoryDemo               0x1030

#define MotorVelocityMode            0x1210
#define MotorPositionMode            0x1211

#define resetMotor                   0x1600
#define CableStop_All                0x1500
#define CableLoose_All               0x1300
#define CablePull_All                0x1400

namespace RxbHardwareInterface {

    typedef unsigned char  u8;
    typedef unsigned short int  u16;
    typedef unsigned int  u32;
    typedef float fp32;
    typedef double fp64;

    struct rxbState {
        double j[Gene_Jnt_Num]; //用于保存臂的关节角
        double duration; // 这个state的时间
        double t;
    };

    struct jointState {
        double j[Controllable_DOF_num_all]; //用于保存可控制的关节角
        double vel[Controllable_DOF_num_all];
        double acc[Controllable_DOF_num_all];
        double duration; // 这个state的时间
        double t;
        jointState():t(0){
            memset(j,0,sizeof (j));
            memset(vel,0,sizeof(vel));
            memset(acc,0,sizeof (acc));
        }
    };

    struct cableState {
        double len[Motor_num_all];
        double t;
        double duration;
    };

    //全局变量
    extern const u8 g_Driver_ID[Motor_num_all];
    extern const u8 g_CaptureCard_ID[Joint_CaptureCard_Num];
    extern const fp64 g_DrivenBox[Motor_num_all];//÷g_DrivenBox[] 到绳索端； xg_DrivenBox[] 到电机端
    extern const fp64 g_DrivenBox_V[Motor_num_all];

    extern std::list<jointState> RXB_joint_plan;
    extern std::list<jointState> RXB_joint_plan_tmp;
    extern std::list<cableState> RXB_Cable_Plan;
    extern std::list<cableState> RXB_Cable_Plan_Tmp;
    extern double global_T_Interval_ms;
    extern double global_T_Interval_s;
    extern double global_EncoderDataGet_ms;
    extern double global_EncoderDataGet_s;

    extern double link_len;
    extern double baseLink_len;
    extern double angle_limit[Controllable_DOF_num_all];
    extern float g_CableAngleOnDisk[ModuJnt_num_all];
    extern u8 g_RXBSegment_num; //关节段数
    extern u8 g_RXBSegment_Joint_num; //
    extern u8 g_RXBSegment_PY_num[ModuJnt_num_all]; //关节段内的PY关节数
    extern u8 g_RXBSegment_YP_num[ModuJnt_num_all]; //关节段内的YP关节数
    extern float g_RXBLink_length[ModuJnt_num_all]; //小关节长度
    extern float g_Disk_Distance ;
    extern float g_Cable_Distribute_Diameter[ModuJnt_num_all];
    extern double g_jointAngle[Gene_Jnt_Num];
    extern double g_JointAngle_Zero[Gene_Jnt_Num];
    extern double g_JointAngle_Absolute[Gene_Jnt_Num];

    extern double g_Init_CablePos[Motor_num_all];
    extern double g_cable_pos_feedback[Motor_num_all], g_motor_vel[Motor_num_all], g_motor_cur[Motor_num_all];//注意这三个数次序对应绳索号而不是电机号
    extern double MotDesire_Pos[Motor_num_all];
    extern double g_offset_CablePos[Motor_num_all];

    extern uint g_rxbCommandFlag;
    extern uint g_rxbMode;
    extern uint g_motorMode;
    extern uint g_recordCommandFlag;  //记录数据开关标志
    extern bool g_recordSwitchFlag;

    extern bool g_isRunningFlag;
    extern int g_rxbWorkStatus;
    extern bool g_randomMotionFlag;

    extern double g_PlanjointAngle_nouse[Gene_Jnt_Num]; // deg save sensor data

    extern double g_actCableForce[Motor_num_all];
    extern uint8_t g_actCableID[Motor_num_all];
    extern double g_sCorCableForce[SCorCableForce_Num];
    extern uint8_t g_sCorCableID[SCorCableForce_Num];
    extern double g_bCorCableForce[BCorCableForce_Num];
    extern uint8_t g_bCorCableID[BCorCableForce_Num];

    /* openloop control */
    extern double g_oloopT_s;

    /* closeloop control */
    extern double cloop_cable_v0[Motor_num_all];
    extern double cloop_joint_v0[Controllable_DOF_num_all];
    extern double g_cloopT_s;

    extern double V_max[Motor_num_all];
    extern double V_min[Motor_num_all];
    extern double JntV_max[Controllable_DOF_num_all];
    extern double JntV_min[Controllable_DOF_num_all];
}
#endif
