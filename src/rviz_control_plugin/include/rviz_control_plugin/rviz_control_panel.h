#ifndef RVIZ_CONTROL_PANEL_H
#define RVIZ_CONTROL_PANEL_H

#include <QWidget>
#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <string>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include "rxb_msgs/MotorMode.h"
#include "rxb_msgs/RxbMode.h"
#include "rxb_msgs/RxbPose.h"
#include "rxb_msgs/RxbPoseCmd.h"
#include "rxb_msgs/sensorRead.h"
#include "rxb_msgs/jointControl.h"
#include "rxb_msgs/forceSenseResult.h"
#include "rxb_msgs/neuralNetworkCommand.h"
#include "rxb_msgs/trajectoryDemo.h"
#include "rxb_msgs/neuralNetworkProcess.h"

#include <rviz/panel.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_interface.h>

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

#define TrajectoryDemo               0x1030

#define isJointCommadFlag            0x1100

#define MotorVelocityMode            0x1210
#define MotorPositionMode            0x1211

#define CableLoose_All               0x1300
#define CablePull_All                0x1400
#define CableStop_All                0x1500

#define deg2rad M_PI/180
#define RxbRunning 1
#define RxbStopping 0
#define Gene_Jnt_Num 8
#define Motor_num_all 3
#define TensionSensor_num_all 21
#define CaptureCard_num_all (TensionSensor_num_all-Motor_num_all)
#define Controllable_DOF_num_all 2
#define ModuJnt_num_all 1//段数

namespace Ui {
    class control_panelWidget;
}

namespace rviz_control_plugin {


    class control_panelWidget : public rviz::Panel {

        Q_OBJECT

    public:
        explicit control_panelWidget(QWidget *parent = nullptr);
        ~control_panelWidget();


    private slots:
        void on_TrajectoryDemo8_clicked();

    private slots:
        void on_TrajectoryDemo7_clicked();

    private slots:
        void on_TrajectoryDemo6_clicked();

    private slots:
        void on_TrajectoryDemo5_clicked();

    private slots:
        void on_resetMotor_button_clicked();

    private:
        Ui::control_panelWidget *ui;
        ros::NodeHandle nodehandle_;


        ros::Publisher flagPublisher_;
        ros::Publisher recordFlagPublisher_;

        ros::Publisher supervisoryControlFlagPublisher_;

        std_msgs::UInt32 pauseFlag;
        ros::Publisher pausePublisher_;

        ros::Publisher motorModePublisher_;
        ros::Publisher motorMovePublisher_;
        ros::Publisher rxbModePublisher_;                      //stop button clicked publish
        ros::Publisher rxbTrajectoryPublisher_;

        ros::Publisher openLoop_jointPosCmdPublisher_;      //openLoop joint position button clicked publish,to moveit
        ros::Publisher closeLoop_jointPosCmdPublisher_;      //closeLoop joint position button
        ros::Publisher ForPos_Hybrid_jointPosCmdPublisher_;
        ros::Publisher matlab_jointCommandPublisher_;
        ros::Publisher motorPositionCommandPublisher_;      //motor position button clicked publish
        //  ros::Publisher endPositionCommandPublisher_;     //target position button clicked publish
        ros::Publisher gripperPositionCommandPublisher_;
        ros::Publisher relayLiftCommandPublisher_;

        ros::Publisher matlab_cableForceInfoPublisher_;
        ros::Publisher matlab_jointPositionInfoPublisher_;
        ros::Publisher ForcePIDWriteInPublisher_;

        ros::Subscriber rxbStateSubscriber_;
        ros::Subscriber jointStateSubscriber_;              //joint state recieve subscribe
        ros::Subscriber motorModeSubscriber_;
        ros::Subscriber motorPositionSubscriber_;
        ros::Subscriber motorVelocitySubscriber_;
        ros::Subscriber motorCurrentSubscriber_;
        ros::Subscriber actualStateSubscriber_;
        ros::Subscriber gripperStateSubscriber_;
        ros::Subscriber CableForceSubscriber_;
        ros::Subscriber externalForceCalSubscriber_;
        ros::Subscriber SensorDataSubscriber_;
        ros::Subscriber SensorDataSubscriber_F;
        ros::Subscriber Motor_Run_Subscriber_;
        ros::Subscriber PlaningJntPosSubscriber_;

        ros::ServiceClient RxbModeClient_;
        ros::ServiceClient MotorModeClient_;
        ros::ServiceClient getPoseClient_;
        ros::ServiceClient endPositionCommandClient_;

        //moveit_puber
        ros::Publisher rviz_Jnt_Pub;
        ros::Publisher ZeroFlagPublisher_;
        sensor_msgs::JointState jointsCurrent2moveit;
        ros::Publisher J1posPIDPublisher_;
        ros::Publisher J1posPIDWriteInPublisher_;

        std::ifstream ifs_PID; //用于读取PID文件中的数

        //工作空间内随机运动
        ros::Publisher randomMotionPublisher_;

        //用于多臂联调，柔性臂接收上位机指令进行控制
        ros::Subscriber SupervisoryControl_RxbMode;
        void SupervisoryControl_RxbModeCallback(std_msgs::UInt32);
        ros::Subscriber supervisoryControl_JointControl;
        void SupervisoryControl_JointControlCallback(rxb_msgs::jointControl);

        //用于力感知实验
        ros::Subscriber exForceMeasureSub;
        ros::Subscriber exForceSenseSub;
        ros::Publisher exForceSenseZeroPub;
        void ExForceMeasureSub_CallbackFunc(const geometry_msgs::Twist::ConstPtr& data);
        void ExForceSenseSub_CallbackFunc(const rxb_msgs::forceSenseResult::ConstPtr& data);

        //用于导纳控制
        ros::Publisher admittanceControlSwitchPub;
        ros::Publisher admittanceControl_seg1Set_pub;

        // 用于神经网络训练
        rxb_msgs::neuralNetworkCommand nnc;
        ros::Publisher nnTrainingNewRcPub;
        ros::Publisher nnTrainingStartPub;
        ros::Subscriber nnTrainingProgressSub;
        void NNTrainingProgress_CallbackFunc(const std_msgs::UInt8::ConstPtr& data);

        // 用于初始化机械臂是同步ui界面参数
        ros::Subscriber uiUpdatePosePIDSub;
        void UIUpdatePosePID_CallbackFunc(const std_msgs::Float64MultiArray::ConstPtr& data);

        // 用于任务空间控制
        ros::Publisher taskSpaceCtrl_FKCal_pub;
        ros::Publisher taskSpaceCtrl_mode_pub;
        ros::Publisher taskSpaceCtrl_move_pub;
        ros::Subscriber taskSpaceCtrl_actualT_sub;
        ros::Subscriber taskSpaceCtrl_desiredJoint_sub;
        ros::Subscriber taskSpaceCtrl_desiredT_sub;
        ros::Subscriber taskSpaceCtrl_planJoint_sub;
        ros::Subscriber taskSpaceCtrl_targetT_sub;
        ros::Subscriber taskSpaceCtrl_errorTwist_sub;
        void TaskSpaceCtrl_ActualT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        void TaskSpaceCtrl_DesiredJoint_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        void TaskSpaceCtrl_DesiredT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        void TaskSpaceCtrl_PlanJoint_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        void TaskSpaceCtrl_TargetT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        void TaskSpaceCtrl_ErrorTwist_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);

        // 补充基础功能
        ros::Subscriber monitorWindow_actualT_sub;
        void MonitorWindow_ActualT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        ros::Subscriber monitorWindow_bodyTwist_sub;
        void MonitorWindow_BodyTwist_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data);
        ros::Publisher trajectoryDemo_pub;

        // 在线神经网络训练
        ros::Publisher neuralNetwork_corCableForceStart_pub;
        ros::Publisher neuralNetwork_corCableForceStop_pub;
        ros::Subscriber neuralNetwork_corCableForceProcess_sub;
        void NeuralNetwork_CorCableForceProcess_SubFunc(const rxb_msgs::neuralNetworkProcess::ConstPtr &data);
        unsigned char neuralNetwork_corCableForce_dataRecord_state = 1;   // 记录状态标志为，1-训练数据，2-验证数据，3-测试数据
        ros::Publisher neuralNetwork_corCableForce_dataRecord_start_pub;
        ros::Publisher neuralNetwork_corCableForce_dataRecord_stop_pub;
        ros::Subscriber neuralNetwork_corCableForce_dataRecord_count_sub;
        void NeuralNetwork_CorCableForce_DataRecord_Count_SubFunc(const std_msgs::UInt32::ConstPtr& data);
        ros::Publisher neuralNetwork_corCableForce_model_train_pub;
        ros::Publisher neuralNetwork_corCableForce_model_dev_state_pub;
        ros::Subscriber neuralNetwork_corCableForce_model_dev_data_sub;
        void NeuralNetwork_CorCableForce_Model_Dev_Data_SubFunc(const std_msgs::Float32MultiArray::ConstPtr& data);

        void displayOutputInfos(const std::string &color, const QString &context);    //display type

        void updateJointState(int index);

        void rxbStateCallback(const std_msgs::UInt32::ConstPtr &rxb_state);
        //void jointStateCallback(const std_msgs::Float64MultiArray::ConstPtr &joint_state);    //joint state callback 用sensorDataCallback代替了
        void CableForceCallback(const std_msgs::Float64MultiArray::ConstPtr &cable_force);//cable force
        void motorPositionCallback(const std_msgs::Float64MultiArray::ConstPtr &motor_pos);      //motor state callback
        void motorVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr &motor_vel);
        void motorCurrentCallback(const std_msgs::Float64MultiArray::ConstPtr &motor_cur);
        void motorModeCallback(const std_msgs::UInt32::ConstPtr &motor_mode);
        void l_ExternalForceCallback(const geometry_msgs::Vector3::ConstPtr &l_externalF);
        void Sensor_Encoder_DataCallback(const rxb_msgs::sensorRead::ConstPtr &data);
        void Sensor_CableForce_DataCallback(const rxb_msgs::sensorRead::ConstPtr &data);
        void PlaningJntPos_DataCallback(const std_msgs::Float64MultiArray::ConstPtr &PlaningJntPos);

        void gripperStateCallback(const std_msgs::Int32::ConstPtr &gripper_state);        //gripper state callback
        void Motor_Run(const std_msgs::Float64MultiArray::ConstPtr &motor_pos);


    private Q_SLOTS:

        /*     Rxb Mode         */
        /*
        //Rxb Mode Change button
        void on_rxbModeChange_clicked(int index);       //choose

        //Rxb Cable Tigthen button
        void on_CableTigthenButton_clicked();           //tigthen

        //Rxb Trajectory button
        void on_rxbTrajectory_clicked();               //trajectory

        //Motor Encoder Reset button
        void on_resetMotorEncoderButton_clicked();      //reset

        //Motor Data Recieve button
        void on_recieveMotorDataButton_clicked();       //read motor data

        //Joint Angle Recieve button
        void on_recieveJointAngleButton_clicked();      //read joint angle

        //Cable Force Recieve button
        void on_recieveCableForceButton_clicked();      //read cable force

        */


        /*     Joint   Position        */
        //Position Controller button
        void on_setJointPositionButton_clicked();               //set

        void on_resetJointPositionButton_clicked();             //reset

        void on_stopJointPositionButton_clicked();              //stop

        void on_getnowJointPositionButton_clicked();            //getnow


        /*      Cable    Control        */
        //Motor Controller button
        void on_setCablePositionButton_clicked();               //set

        void on_resetCablePositionButton_clicked();             //reset

        void on_stopCablePositionButton_clicked();              //stop

        void on_getnowCablePositionButton_clicked();            //getnow


        /*      Target  Control         */

        void on_CalculateTargetPositionButton_clicked();

        void on_setTargetPositionButton_clicked();              //set

        //void on_resetTargetPositionButton_clicked();            //reset

        void on_getnowTargetPositionButton_clicked();           //getnow

        void on_setGripperPositionButton_clicked();             //set

        void on_readnowGripperPositionButton_clicked();         //readnow

        void on_getnowGripperPositionButton_clicked();          //getnow

        /*      Rxb    Mode        */


        void on_RxbInitialize_clicked();
        void on_RxbAutoback_clicked();
        void on_RxbShutdown_clicked();
        void on_RxbWorking_clicked();

        void on_RxbMode2_Button_2_clicked();


        void on_RxbMode2_Button_clicked();


        void on_RxbMode1_Button_clicked();

        void on_TrajectoryDemo4_clicked();

        void on_TrajectoryDemo3_clicked();

        void on_TrajectoryDemo2_clicked();

        void on_TrajectoryDemo1_clicked();


        /*    Force  Sensor   */
        //  private slots:
        void on_position_mode_button_clicked();
        void on_velocity_mode_button_clicked();
        void on_externalF_cau_button_clicked();

        void on_PullAllCable_Button_released();

        void on_PullAllCable_Button_pressed();

        void on_LooseAllCable_Button_released();

        void on_LooseAllCable_Button_pressed();

        void on_CablePull_Button_12_released();

        void on_CablePull_Button_12_pressed();

        void on_CableLoose_Button_12_released();

        void on_CableLoose_Button_12_pressed();

        void on_CablePull_Button_11_released();

        void on_CablePull_Button_11_pressed();

        void on_CableLoose_Button_11_released();

        void on_CableLoose_Button_11_pressed();

        void on_CablePull_Button_10_released();

        void on_CablePull_Button_10_pressed();

        void on_CableLoose_Button_10_released();

        void on_CableLoose_Button_10_pressed();

        void on_CablePull_Button_9_released();

        void on_CablePull_Button_9_pressed();

        void on_CableLoose_Button_9_released();

        void on_CableLoose_Button_9_pressed();

        void on_CablePull_Button_8_released();

        void on_CablePull_Button_8_pressed();

        void on_CableLoose_Button_8_released();

        void on_CableLoose_Button_8_pressed();

        void on_CablePull_Button_7_released();

        void on_CablePull_Button_7_pressed();

        void on_CableLoose_Button_7_released();

        void on_CableLoose_Button_7_pressed();

        void on_CablePull_Button_6_released();

        void on_CablePull_Button_6_pressed();

        void on_CableLoose_Button_6_released();

        void on_CableLoose_Button_6_pressed();

        void on_CablePull_Button_5_released();

        void on_CablePull_Button_5_pressed();

        void on_CableLoose_Button_5_released();

        void on_CableLoose_Button_5_pressed();

        void on_CablePull_Button_4_released();

        void on_CablePull_Button_4_pressed();

        void on_CableLoose_Button_4_released();

        void on_CableLoose_Button_4_pressed();

        void on_CablePull_Button_3_released();

        void on_CablePull_Button_3_pressed();

        void on_CableLoose_Button_3_released();

        void on_CableLoose_Button_3_pressed();

        void on_CablePull_Button_2_released();

        void on_CablePull_Button_2_pressed();

        void on_CableLoose_Button_2_released();

        void on_CableLoose_Button_2_pressed();

        void on_CablePull_Button_1_released();

        void on_CablePull_Button_1_pressed();

        void on_CableLoose_Button_1_released();

        void on_CableLoose_Button_1_pressed();

        void on_clearInfoButton_clicked();

        void on_pauseButton_clicked();

        void rviz_pubjoint_2moveit();
        void on_zero_clicked();
        void on_joint1_state_cursorPositionChanged(int arg1, int arg2);
        void on_joint2_state_cursorPositionChanged(int arg1, int arg2);
        void on_CablePull_Button_1_clicked();
        void on_CablePull_Button_13_pressed();
        void on_CablePull_Button_13_released();
        void on_RxbMode4_Button_clicked();
        void on_Cable_Button_1_clicked();
        void on_RxbMode3_Button_clicked();
        void on_J1_pid_set_clicked();
        void on_J2_pid_set_clicked();
        void on_J3_pid_set_clicked();
        void on_J4_pid_set_clicked();
        void on_J5_pid_set_clicked();
        void on_J6_pid_set_clicked();
        void on_Use_J1_all_set_clicked();
        void on_All_PosPID_para_writeIn_clicked();
        void on_CablePull_Button_13_clicked();
        void on_CablePull_Button_13_clicked(bool checked);
        void on_CablePull_Button_14_pressed();
        void on_CablePull_Button_14_released();
        void on_CableLoose_Button_1_clicked();
        void on_CableLoose_Button_1_1_pressed();
        void on_CableLoose_Button_15_pressed();
        void on_CableLoose_Button_15_released();
        void on_CableLoose_Button_16_pressed();
        void on_CableLoose_Button_16_released();
        void on_CablePull_Button_17_pressed();
        void on_CablePull_Button_17_released();
        void on_get_PID_value_from_file_clicked();
        void on_CablePull_Button_14_clicked();
        void on_CablePull_Button_014_pressed();
        void on_CablePull_Button_014_released();
        void on_CableLoose_Button_20_pressed();
        void on_get_PosePID_value_from_file_clicked();
        void on_J1_PosePID_set_2_clicked();
        void on_J2_PosePID_set_clicked();
        void on_J1_PosePID_set_clicked();
        void on_ActualOperation_Button_clicked();
        void on_J1_ForcePID_set_clicked();
        void on_J2_ForcePID_set_clicked();
        void on_RxbMode3_Button_2_clicked();
        void on_CableLoose_Driven2_pressed();
        void on_CableLoose_Driven2_released();
        void on_CableLoose_Driven1_pressed();
        void on_CableLoose_Driven3_pressed();
        void on_CableLoose_Driven4_pressed();
        void on_CableLoose_Driven5_pressed();
        void on_CableLoose_Driven6_pressed();
        void on_CableLoose_Driven7_pressed();
        void on_CableLoose_Driven8_pressed();
        void on_CableLoose_Driven9_pressed();
        void on_CableLoose_Driven1_released();
        void on_CableLoose_Driven3_released();
        void on_CableLoose_Driven4_released();
        void on_CableLoose_Driven5_released();
        void on_CableLoose_Driven6_released();
        void on_CableLoose_Driven7_released();
        void on_CableLoose_Driven8_released();
        void on_CableLoose_Driven9_released();
        void on_CablePull_Driven1_pressed();
        void on_CablePull_Driven2_pressed();
        void on_CablePull_Driven3_pressed();
        void on_CablePull_Driven4_pressed();
        void on_CablePull_Driven5_pressed();
        void on_CablePull_Driven6_pressed();
        void on_CablePull_Driven7_pressed();
        void on_CablePull_Driven8_pressed();
        void on_CablePull_Driven9_pressed();
        void on_CablePull_Driven1_released();
        void on_CablePull_Driven2_released();
        void on_CablePull_Driven3_released();
        void on_CablePull_Driven4_released();
        void on_CablePull_Driven5_released();
        void on_CablePull_Driven6_released();
        void on_CablePull_Driven7_released();
        void on_CablePull_Driven8_released();
        void on_CablePull_Driven9_released();
        void on_CableLoose_Driven10_pressed();
        void on_CableLoose_Driven11_pressed();
        void on_CableLoose_Driven12_pressed();
        void on_CablePull_Driven10_pressed();
        void on_CablePull_Driven11_pressed();
        void on_CablePull_Driven12_pressed();
        void on_CableLoose_Driven10_released();
        void on_CablePull_Driven10_released();
        void on_CableLoose_Driven11_released();
        void on_CablePull_Driven11_released();
        void on_CableLoose_Driven12_released();
        void on_CablePull_Driven12_released();
        void on_zero_2_clicked();
        void on_zero_3_clicked();
        void on_J1_PosePID_set_3_clicked();
        void on_J2_PosePID_set_3_clicked();
        void on_J1_PosePID_set_4_clicked();
        void on_J2_PosePID_set_4_clicked();
        void on_J1_PosePID_set_11_clicked();
        void on_J2_PosePID_set_13_clicked();
        void on_J1_PosePID_set_12_clicked();
        void on_All_ForcePID_para_writeIn_5_clicked();
        void on_get_ForcePID_value_from_file_5_clicked();
        void on_SupervisoryControl_clicked();
        void on_RecordFlag_Button_clicked();
        void on_radioButton_clicked();
        void on_ExForceSense_Zero_clicked();
        void on_radioButton_2_clicked();
        void on_revoluteType_pitch_clicked();
        void on_revoluteType_yaw_clicked();
        void on_revoluteSign_positive_clicked();
        void on_revoluteSign_negative_clicked();
        void on_segmentID_valueChanged(int arg1);
        void on_jointID_valueChanged(int arg1);
        void on_revoluteSign_0_clicked();
        void on_newRecordBtn_clicked();
        void on_startBtn_clicked();
        void on_zero_4_clicked();
        void on_zero_5_clicked();
        void on_zero_6_clicked();
        void on_segment1_zero_clicked();
        void on_segment2_zero_clicked();
        void on_segment3_zero_clicked();
        void on_EmmergencyButton_stateChanged(int arg1);
        void on_ControlMode_Openloop_clicked();
        void on_ControlMode_ForPosHybrid_clicked();
        void on_ControlMode_Closeloop_clicked();
        void on_BasicMode_Openloop_clicked();
        void on_BasicMode_Closeloop_clicked();
        void on_BasicMode_ForPosHybrid_clicked();
        void on_ExtraMode_RecordData_stateChanged(int arg1);
        void on_ExtraMode_RecordData_2_stateChanged(int arg1);
        void on_AdjustCable_Loose1_pressed();
        void on_AdjustCable_Loose2_pressed();
        void on_AdjustCable_Loose3_pressed();
        void on_AdjustCable_Loose1_released();
        void on_AdjustCable_Loose2_released();
        void on_AdjustCable_Loose3_released();
        void on_AdjustCable_Pull1_pressed();
        void on_AdjustCable_Pull2_pressed();
        void on_AdjustCable_Pull3_pressed();
        void on_AdjustCable_Pull1_released();
        void on_AdjustCable_Pull2_released();
        void on_AdjustCable_Pull3_released();
        void on_AdjustCable_Enable_stateChanged(int arg1);
        void on_AdjustCable_Loose4_pressed();
        void on_AdjustCable_Loose4_released();
        void on_AdjustCable_Pull4_pressed();
        void on_AdjustCable_Pull4_released();
        void on_AdjustCable_Loose5_pressed();
        void on_AdjustCable_Loose5_released();
        void on_AdjustCable_Pull5_pressed();
        void on_AdjustCable_Pull5_released();
        void on_AdjustCable_Loose6_pressed();
        void on_AdjustCable_Loose6_released();
        void on_AdjustCable_Pull6_pressed();
        void on_AdjustCable_Pull6_released();
        void on_AdjustCable_Loose7_pressed();
        void on_AdjustCable_Pull7_released();
        void on_AdjustCable_Loose7_released();
        void on_AdjustCable_Pull7_pressed();
        void on_AdjustCable_Loose8_pressed();
        void on_AdjustCable_Loose8_released();
        void on_AdjustCable_Pull8_pressed();
        void on_AdjustCable_Pull8_released();
        void on_AdjustCable_Loose9_pressed();
        void on_AdjustCable_Loose9_released();
        void on_AdjustCable_Pull9_pressed();
        void on_AdjustCable_Pull9_released();
        void on_J3_PosePID_set_clicked();
        void on_ExtraMode_RandomMove_stateChanged(int arg1);
        void on_ExtraMode_DragTeaching_stateChanged(int arg1);
        void on_TaskSpaceControl_EnableBtn_stateChanged(int arg1);
        void on_Calculation_FK_clicked();
        void on_TaskSpaceControl_SingleSegBtn_clicked();
        void on_TaskSpaceControl_MultiSegBtn_clicked();
        void on_TSCtrl_Move_clicked();
        void on_CorCableForce_Start_clicked();
        void on_CorCableForce_Stop_clicked();
        void on_AdmittanceControl_Enable_stateChanged(int arg1);
        void on_AdmittanceControl_Seg1Set_clicked();
        void on_CorCableForce_DataRecord_Start_clicked();
        void on_CorCableForce_DataRecord_TrainSet_clicked();
        void on_CorCableForce_DataRecord_DevSet_clicked();
        void on_CorCableForce_DataRecord_TestSet_clicked();
        void on_CorCableForce_DataRecord_Stop_clicked();
        void on_CorCableForce_Model_Train_clicked();
        void on_CorCableForce_Model_Dev_State_clicked(bool checked);
    };

    extern float  g_plugin_mot_cur[Motor_num_all];
    extern float  g_plugin_mot_vel[Motor_num_all];
    extern double g_plugin_mot_pos[Motor_num_all];
    extern double g_plugin_jnt_pos[Gene_Jnt_Num];
    extern double g_plugin_tar_pos[7];
    extern double g_plugin_gripper_pos;

    extern double g_plugin_cableF[TensionSensor_num_all];
    extern double g_plugin_exForceMeasure[6];
    extern double g_plugin_exForceSense[Controllable_DOF_num_all];

    extern int rxb_mode;
    extern int motor_mode;
    extern bool g_rxb_state;

    extern float  g_jnt_pub2movit[6];
}

#endif
