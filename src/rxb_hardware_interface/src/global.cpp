#include "global.h"

namespace RxbHardwareInterface {
//电机对应驱动号

    const u8 g_Driver_ID[Motor_num_all] = {3, 1, 2};//绳索号对应电机号 例如第2根绳索对应于第4个电机
    const u8 g_CaptureCard_ID[Joint_CaptureCard_Num] = {1, 3};

    //驱动箱减速比 DB_Ratio = (500*4)(qc)*Mot_Reduction/Lead 从减速前电机到丝杆滑块的减速比，即滑块每移动1mm减速前电机转动的qc（定滑轮）
    const fp64 g_DrivenBox[Motor_num_all] = {DB_Ratio, DB_Ratio, DB_Ratio};
    //同样为驱动箱减速比，只不过单位不同 mm/s到RPM
    const fp64 g_DrivenBox_V[Motor_num_all] = {DB_Ratio*60/(4*500), DB_Ratio*60/(4*500), DB_Ratio*60/(4*500)};//{1260*2, 1260*2, 1260*2, 1260*2, 1260*2, 1260*2};

    double global_EncoderDataGet_ms = 3;
    double global_EncoderDataGet_s = global_EncoderDataGet_ms/1000;
    double global_T_Interval_ms = 20;
    double global_T_Interval_s = global_T_Interval_ms / 1000;

    float g_Cable_Distribute_Diameter[ModuJnt_num_all] = {96}; //圆孔分布圆的直径
    float g_CableAngleOnDisk[ModuJnt_num_all] = {30};//%每1段中靠近末端的3个控制绳索中第一个绳索的角度位置,逆时针方向  ，从x轴正方向向底部看去，与y轴之间的夹角{150, 90, 120}
    double link_len = 125; //需要改动（LJH）
    double baseLink_len = 125;
    double angle_limit[Controllable_DOF_num_all] = {15, 15};
    u8 g_RXBSegment_num = 1; //关节da段数
    u8 g_RXBSegment_Joint_num = 8; //单个关节段的小关节数
    u8 g_RXBSegment_PY_num[ModuJnt_num_all] = {2}; //关节段内的PY关节数
    u8 g_RXBSegment_YP_num[ModuJnt_num_all] = {2}; //关节段内的YP关节数
    float g_RXBLink_length[ModuJnt_num_all] = {125}; //小关节长度
    float g_Disk_Distance = 54;
    const fp32 d = g_Disk_Distance / 2; //2d为圆盘间距 mm

    double MotDesire_Pos[Motor_num_all] = {0};
    double g_Init_CablePos[Motor_num_all] = {0};
    double g_cable_pos_feedback[Motor_num_all] = {0};
    double g_offset_CablePos[Motor_num_all] = {0};//规划关节角通过关节空间到绳索空间的理想运动学计算得到的绳长
    double g_motor_vel[Motor_num_all] = {0};
    double g_motor_cur[Motor_num_all] = {0};
    double g_jointAngle[Gene_Jnt_Num] = {0};
    double g_JointAngle_Zero[Gene_Jnt_Num] = {0};//零位关节角
    double g_JointAngle_Absolute[Gene_Jnt_Num] = {0};//角度编码器直接测量得到的关节角
    uint g_rxbCommandFlag = isRxbOnFlag;
    uint g_rxbMode = RxbMode_OLoop_Pos;//默认是开环模式
    uint g_motorMode = MotorPositionMode;
    uint g_recordCommandFlag = isRecordOffFlag; //记录数据开关标志
    bool g_recordSwitchFlag = false;

    bool g_isRunningFlag = false;
    int g_rxbWorkStatus = 1;
    bool g_randomMotionFlag = false;  //机械臂随机运动

    double g_PlanjointAngle_nouse[Gene_Jnt_Num] = {0};

    double g_actCableForce[Motor_num_all] = {0};
    uint8_t g_actCableID[Motor_num_all] = {18, 19, 20};
    double g_sCorCableForce[SCorCableForce_Num] = {0};
    uint8_t g_sCorCableID[SCorCableForce_Num] = {4, 5, 11, 10, 16, 17};
    double g_bCorCableForce[BCorCableForce_Num] = {0};
    uint8_t g_bCorCableID[BCorCableForce_Num] = {0, 2, 1, 3, 7, 9, 6, 8, 12, 14, 13, 15};

    /*  open close  */
    double g_oloopT_s = 90;//单关节走90°使用的时间，单位s 600

    /* closeloop control  */
    double cloop_joint_v0[Controllable_DOF_num_all] = {0};
    double cloop_cable_v0[Motor_num_all] = {0};
    double g_cloopT_s = g_oloopT_s;

    double V_max[Motor_num_all] = {7560, 7560, 7560}; // {7560, 7560, 7560};
    double V_min[Motor_num_all] = {-7560, -7560, -7560}; // {-7560, -7560, -7560};
    double JntV_max[Controllable_DOF_num_all] = {1.00 , 1.00};//°/s
    double JntV_min[Controllable_DOF_num_all] = {-1.00, -1.00};
}


