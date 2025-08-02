#include <rviz_control_plugin/rviz_control_panel.h>
#include "ui_rviz_control_panel.h"
#include <iostream>
#include "../../rxb_hardware_interface/include/rxb_hardware_interface/usbcan.h"


namespace rviz_control_plugin {
    float  g_plugin_mot_cur[Motor_num_all] = {0};
    float  g_plugin_mot_vel[Motor_num_all] = {0};
    double g_plugin_mot_pos[Motor_num_all] = {0};
    double g_plugin_jnt_pos[Gene_Jnt_Num] = {0};

    double g_plugin_tar_pos[7] = {0};
    double g_plugin_gripper_pos = 0;

    float  g_jnt_pub2movit[6]= {0};

    double g_plugin_cableF[TensionSensor_num_all] = {0};

    double g_plugin_exForceMeasure[6] = {0};
    double g_plugin_exForceSense[Controllable_DOF_num_all] = {0};

    int rxb_mode = RxbMode_OLoop_Pos;
    int motor_mode = MotorPositionMode;
    bool g_rxb_state = RxbStopping;

    control_panelWidget::control_panelWidget(QWidget *parent) :
        rviz::Panel(parent),
        ui(new Ui::control_panelWidget) {

        ui->setupUi(this);

        pauseFlag.data = 0;

        pausePublisher_= nodehandle_.advertise<std_msgs::UInt32>("control_plugin/pause_flag", 1);
        flagPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/flag", 1);
        recordFlagPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/record_flag", 1);
        supervisoryControlFlagPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/supervisory_control_flag", 1);
        // motorModePublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/motor_mode", 1);
        motorMovePublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/motor_move", 1);
        rxbTrajectoryPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/trajectory_demo", 1);
        //  rxbModePublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/rxb_mode", 1);

        openLoop_jointPosCmdPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/openLoop_jointPosCmd", 1);
        closeLoop_jointPosCmdPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/closeLoop_jointPosCmd", 1);
        ForPos_Hybrid_jointPosCmdPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/ForPos_Hybrid_jointPosCmd", 1);

        motorPositionCommandPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/all_motor_position_command", 1);
        //endPositionCommandPublisher_ = nodehandle_.advertise<geometry_msgs::Point>("control_plugin/end_position_command", 1);
        gripperPositionCommandPublisher_ = nodehandle_.advertise<std_msgs::Int32>("control_plugin/gripper_position_command", 1);
        relayLiftCommandPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("relay_lift_node/relay_command", 1);

        matlab_cableForceInfoPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("matlab/all_cable_force", 1);
        matlab_jointPositionInfoPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("matlab/all_joint_position", 1);
        matlab_jointCommandPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("matlab/all_joint_position_command", 1);

        rxbStateSubscriber_ = nodehandle_.subscribe("rxbHW/RxbState", 1, &control_panelWidget::rxbStateCallback, this);
        //jointStateSubscriber_ = nodehandle_.subscribe("rxbHW/JntEncoderPos", 1, &control_panelWidget::jointStateCallback, this);//用sensorDataCallback代替了
        // motorModeSubscriber_ = nodehandle_.subscribe("rxbHW/MotorMode", 1, &control_panelWidget::motorModeCallback, this);
        motorPositionSubscriber_ = nodehandle_.subscribe("rxbHW/MotorPos", 1, &control_panelWidget::motorPositionCallback, this);
        // motorVelocitySubscriber_ = nodehandle_.subscribe("rxbHW/MotorVel", 1, &control_panelWidget::motorVelocityCallback, this);
        // motorCurrentSubscriber_ = nodehandle_.subscribe("rxbHW/MotorCur", 1, &control_panelWidget::motorCurrentCallback, this);
//        CableForceSubscriber_ = nodehandle_.subscribe("rxbHW/CableF", 1, &control_panelWidget::CableForceCallback, this);
//        externalForceCalSubscriber_ = nodehandle_.subscribe("matlab/l_ExternalF", 1, &control_panelWidget::l_ExternalForceCallback, this);

        gripperStateSubscriber_ = nodehandle_.subscribe("control_plugin/plugin_gripper_states", 1, &control_panelWidget::gripperStateCallback, this);

        RxbModeClient_ = nodehandle_.serviceClient<rxb_msgs::RxbMode>("control_plugin/RxbMode");
        MotorModeClient_ = nodehandle_.serviceClient<rxb_msgs::MotorMode>("control_plugin/MotorMode");
        endPositionCommandClient_ = nodehandle_.serviceClient<rxb_msgs::RxbPoseCmd>("control_plugin/end_position_command");
        SensorDataSubscriber_ = nodehandle_.subscribe<rxb_msgs::sensorRead>("serialport/Sensor_Encoder_data", 1, &control_panelWidget::Sensor_Encoder_DataCallback, this);//更新界面传感器数值

        SensorDataSubscriber_F = nodehandle_.subscribe<rxb_msgs::sensorRead>("serialport/sensor_data", 1, &control_panelWidget::Sensor_CableForce_DataCallback, this);//更新界面传感器数值
        //moveit publisher
        rviz_Jnt_Pub = nodehandle_.advertise<sensor_msgs::JointState>("/rxb/joint_states", 1);

        ZeroFlagPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/joint_zero_signal", 1);//0位写入
        J1posPIDPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/J1pospid", 1);
        ForcePIDWriteInPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/forcepidWriteIn", 1);
        J1posPIDWriteInPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("control_plugin/J1pospidWriteIn", 1);
        //Motor_Run_Subscriber_ = nodehandle_.subscribe("/cableLen", 1,&control_panelWidget::Motor_Run,this);
        PlaningJntPosSubscriber_ = nodehandle_.subscribe<std_msgs::Float64MultiArray>("rxbHW/PlaningJntPos", 1, &control_panelWidget::PlaningJntPos_DataCallback, this);//显示规划数据

        //机械臂随机运动，用于获得神经网络测试数据
        randomMotionPublisher_ = nodehandle_.advertise<std_msgs::UInt32>("control_plugin/randomMotion", 1);

        //用于多臂联调，柔性臂接收上位机指令进行控制
        SupervisoryControl_RxbMode = nodehandle_.subscribe<std_msgs::UInt32>("SupervisoryControl/RxbMode", 1, &control_panelWidget::SupervisoryControl_RxbModeCallback, this);
        supervisoryControl_JointControl = nodehandle_.subscribe("SupervisoryControl/JointControl", 1, &control_panelWidget::SupervisoryControl_JointControlCallback, this);

        //用于力感知实验
        exForceMeasureSub = nodehandle_.subscribe<geometry_msgs::Twist>("forceSensor/force", 1, &control_panelWidget::ExForceMeasureSub_CallbackFunc, this);
        exForceSenseSub = nodehandle_.subscribe<rxb_msgs::forceSenseResult>("force_sense/result", 1, &control_panelWidget::ExForceSenseSub_CallbackFunc, this);
        exForceSenseZeroPub = nodehandle_.advertise<std_msgs::UInt32>("force_sense/zero", 1);

        //用于导纳控制
        admittanceControlSwitchPub = nodehandle_.advertise<std_msgs::UInt32>("admittance_control/switch", 1);
        admittanceControl_seg1Set_pub = nodehandle_.advertise<std_msgs::Float32MultiArray>("admittanceControl/seg1Set", 1);

        // 用于神经网络训练
        nnc.segmentID = 0;  nnc.jointID = 0;
        nnc.revoluteType = 0; // 0——Pitch;1——Yaw
        nnc.revoluteSign = 0; // 0——None;1——Posiitive;2--Negative
        nnTrainingNewRcPub = nodehandle_.advertise<std_msgs::UInt8>("nnTraining/newRc", 1);
        nnTrainingStartPub = nodehandle_.advertise<rxb_msgs::neuralNetworkCommand>("nnTraining/start", 1);
        nnTrainingProgressSub = nodehandle_.subscribe<std_msgs::UInt8>("nnTraining/progress", 1, &control_panelWidget::NNTrainingProgress_CallbackFunc, this);

        // 用于初始化机械臂是同步ui界面参数
        uiUpdatePosePIDSub = nodehandle_.subscribe<std_msgs::Float64MultiArray>("uiUpdate/posePID", 1, &control_panelWidget::UIUpdatePosePID_CallbackFunc, this);

        // 用于任务空间控制
        taskSpaceCtrl_FKCal_pub = nodehandle_.advertise<std_msgs::Float64MultiArray>("taskSpaceCtrl/FKCal", 1);
        taskSpaceCtrl_mode_pub = nodehandle_.advertise<std_msgs::UInt8>("taskSpaceCtrl/mode", 1);
        taskSpaceCtrl_move_pub = nodehandle_.advertise<std_msgs::Float64MultiArray>("taskSpaceCtrl/move", 1);
        taskSpaceCtrl_actualT_sub = nodehandle_.subscribe("taskSpaceCtrl/actualT", 1, &control_panelWidget::TaskSpaceCtrl_ActualT_SubFunc, this);
        taskSpaceCtrl_desiredJoint_sub = nodehandle_.subscribe("taskSpaceCtrl/desiredJoint", 1, &control_panelWidget::TaskSpaceCtrl_DesiredJoint_SubFunc, this);
        taskSpaceCtrl_desiredT_sub = nodehandle_.subscribe("taskSpaceCtrl/desiredT", 1, &control_panelWidget::TaskSpaceCtrl_DesiredT_SubFunc, this);
        taskSpaceCtrl_planJoint_sub = nodehandle_.subscribe("taskSpaceCtrl/planJoint", 1, &control_panelWidget::TaskSpaceCtrl_PlanJoint_SubFunc, this);
        taskSpaceCtrl_targetT_sub = nodehandle_.subscribe("taskSpaceCtrl/targetT", 1, &control_panelWidget::TaskSpaceCtrl_TargetT_SubFunc, this);
        taskSpaceCtrl_errorTwist_sub = nodehandle_.subscribe("taskSpaceCtrl/errorTwist", 1, &control_panelWidget::TaskSpaceCtrl_ErrorTwist_SubFunc, this);

        // 补充基础功能
        monitorWindow_actualT_sub = nodehandle_.subscribe("monitorWindow/actualT", 1, &control_panelWidget::MonitorWindow_ActualT_SubFunc, this);
        monitorWindow_bodyTwist_sub = nodehandle_.subscribe("monitorWindow/bodyTwist", 1, &control_panelWidget::MonitorWindow_BodyTwist_SubFunc, this);
        trajectoryDemo_pub = nodehandle_.advertise<rxb_msgs::trajectoryDemo>("trajectoryDemo", 1);

        // 在线神经网络训练
        neuralNetwork_corCableForceStart_pub = nodehandle_.advertise<std_msgs::Bool>("neuralNetwork/corCableForce/start", 1);
        neuralNetwork_corCableForceStop_pub = nodehandle_.advertise<std_msgs::Bool>("neuralNetwork/corCableForce/stop", 1);
        neuralNetwork_corCableForceProcess_sub = nodehandle_.subscribe("neuralNetwork/corCableForce/process", 1, &control_panelWidget::NeuralNetwork_CorCableForceProcess_SubFunc, this);
        neuralNetwork_corCableForce_dataRecord_start_pub = nodehandle_.advertise<std_msgs::UInt8>("neuralNetwork/corCableForce/dataRecord/start", 1);
        neuralNetwork_corCableForce_dataRecord_stop_pub = nodehandle_.advertise<std_msgs::UInt8>("neuralNetwork/corCableForce/dataRecord/stop", 1);
        neuralNetwork_corCableForce_dataRecord_count_sub = nodehandle_.subscribe("neuralNetwork/corCableForce/dataRecord/count", 1, &control_panelWidget::NeuralNetwork_CorCableForce_DataRecord_Count_SubFunc, this);
        neuralNetwork_corCableForce_model_train_pub = nodehandle_.advertise<std_msgs::Bool>("neuralNetwork/corCableForce/model/train", 1);
        neuralNetwork_corCableForce_model_dev_state_pub = nodehandle_.advertise<std_msgs::Bool>("neuralNetwork/corCableForce/model/dev/state", 1);
        neuralNetwork_corCableForce_model_dev_data_sub = nodehandle_.subscribe("neuralNetwork/corCableForce/model/dev/data", 1, &control_panelWidget::NeuralNetwork_CorCableForce_Model_Dev_Data_SubFunc, this);
    }

    //用于多臂联调，柔性臂接收上位机指令进行控制
    void control_panelWidget::SupervisoryControl_RxbModeCallback(std_msgs::UInt32 inf){
        rxb_msgs::RxbMode data;

        if(inf.data == 0){
            data.request.mode = RxbMode_OLoop_Pos;
            if (RxbModeClient_.call(data)) {
                rxb_mode = RxbMode_OLoop_Pos;
                displayOutputInfos("black", "set rxb_openloop_mode successfully");
            }
        }else if(inf.data == 1){
            data.request.mode = RxbMode_ForPos_Hybrid;
            if (RxbModeClient_.call(data)) {
                rxb_mode = RxbMode_ForPos_Hybrid;
                displayOutputInfos("black", "set rxb_ForPos_Hybrid_mode successfully");
            }
        }
    }

    //用于力感知实验
    void control_panelWidget::ExForceMeasureSub_CallbackFunc(const geometry_msgs::Twist::ConstPtr& data){
        // 力矩单位为N mm
        g_plugin_exForceMeasure[0] = data->angular.x * 1000;
        g_plugin_exForceMeasure[1] = data->angular.y * 1000;
        g_plugin_exForceMeasure[2] = data->angular.z * 1000;
        // 力单位为N
        g_plugin_exForceMeasure[3] = data->linear.x;
        g_plugin_exForceMeasure[4] = data->linear.y;
        g_plugin_exForceMeasure[5] = data->linear.z;

        updateJointState(5);
    }

    void control_panelWidget::ExForceSenseSub_CallbackFunc(const rxb_msgs::forceSenseResult::ConstPtr &data){
      g_plugin_exForceSense[2*data->ID] = data->force[0];
      g_plugin_exForceSense[2*data->ID+1] = data->force[1];

      updateJointState(7);
    }

    // 用于神经网络训练
    void control_panelWidget::NNTrainingProgress_CallbackFunc(const std_msgs::UInt8::ConstPtr& data){
      ui->recordProgress->setValue(data->data);
    }

    // 用于初始化机械臂是同步ui界面参数
    void control_panelWidget::UIUpdatePosePID_CallbackFunc(const std_msgs::Float64MultiArray::ConstPtr& data){
      for(unsigned long i = 0; i < ModuJnt_num_all*3;){
        std::cout << data->data[i] << " ";
        ui->Pos_P1->setValue(data->data[i++]);
        std::cout << data->data[i] << " ";
        ui->Pos_I1->setValue(data->data[i++]);
        std::cout << data->data[i] << " ";
        ui->Pos_D1->setValue(data->data[i++]);
        std::cout << std::endl;
      }
    }

    // 用于任务空间控制
    void control_panelWidget::TaskSpaceCtrl_ActualT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data) {
        ui->ActualT_1_1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->ActualT_1_2->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->ActualT_1_3->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->ActualT_1_4->setText(QString::number(static_cast<double>(data->data[3]), 'f', 2));
        ui->ActualT_2_1->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
        ui->ActualT_2_2->setText(QString::number(static_cast<double>(data->data[5]), 'f', 2));
        ui->ActualT_2_3->setText(QString::number(static_cast<double>(data->data[6]), 'f', 2));
        ui->ActualT_2_4->setText(QString::number(static_cast<double>(data->data[7]), 'f', 2));
        ui->ActualT_3_1->setText(QString::number(static_cast<double>(data->data[8]), 'f', 2));
        ui->ActualT_3_2->setText(QString::number(static_cast<double>(data->data[9]), 'f', 2));
        ui->ActualT_3_3->setText(QString::number(static_cast<double>(data->data[10]), 'f', 2));
        ui->ActualT_3_4->setText(QString::number(static_cast<double>(data->data[11]), 'f', 2));
    }
    void control_panelWidget::TaskSpaceCtrl_DesiredJoint_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data) {
        ui->DesiredJoint_P1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->DesiredJoint_Y1->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
    }
    void control_panelWidget::TaskSpaceCtrl_DesiredT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data) {
        ui->DesiredT_1_1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->DesiredT_1_2->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->DesiredT_1_3->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->DesiredT_1_4->setText(QString::number(static_cast<double>(data->data[3]), 'f', 2));
        ui->DesiredT_2_1->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
        ui->DesiredT_2_2->setText(QString::number(static_cast<double>(data->data[5]), 'f', 2));
        ui->DesiredT_2_3->setText(QString::number(static_cast<double>(data->data[6]), 'f', 2));
        ui->DesiredT_2_4->setText(QString::number(static_cast<double>(data->data[7]), 'f', 2));
        ui->DesiredT_3_1->setText(QString::number(static_cast<double>(data->data[8]), 'f', 2));
        ui->DesiredT_3_2->setText(QString::number(static_cast<double>(data->data[9]), 'f', 2));
        ui->DesiredT_3_3->setText(QString::number(static_cast<double>(data->data[10]), 'f', 2));
        ui->DesiredT_3_4->setText(QString::number(static_cast<double>(data->data[11]), 'f', 2));
    }
    void control_panelWidget::TaskSpaceCtrl_PlanJoint_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data){
        ui->PlanJoint_P1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->PlanJoint_Y1->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
    }
    void control_panelWidget::TaskSpaceCtrl_TargetT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data) {
        ui->TargetT_1_1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->TargetT_1_2->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->TargetT_1_3->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->TargetT_1_4->setValue(static_cast<double>(data->data[3]));
        ui->TargetT_2_1->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
        ui->TargetT_2_2->setText(QString::number(static_cast<double>(data->data[5]), 'f', 2));
        ui->TargetT_2_3->setText(QString::number(static_cast<double>(data->data[6]), 'f', 2));
        ui->TargetT_2_4->setValue(static_cast<double>(data->data[7]));
        ui->TargetT_3_1->setText(QString::number(static_cast<double>(data->data[8]), 'f', 2));
        ui->TargetT_3_2->setText(QString::number(static_cast<double>(data->data[9]), 'f', 2));
        ui->TargetT_3_3->setText(QString::number(static_cast<double>(data->data[10]), 'f', 2));
        ui->TargetT_3_4->setValue(static_cast<double>(data->data[11]));
    }
    void control_panelWidget::TaskSpaceCtrl_ErrorTwist_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data){
        ui->ErrorTwist_omega1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->ErrorTwist_omega2->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->ErrorTwist_omega3->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->ErrorTwist_vel1->setText(QString::number(static_cast<double>(data->data[3]), 'f', 2));
        ui->ErrorTwist_vel2->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
        ui->ErrorTwist_vel3->setText(QString::number(static_cast<double>(data->data[5]), 'f', 2));
    }

    // 补充基础功能
    void control_panelWidget::MonitorWindow_ActualT_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data){
        ui->ActualT2_1_1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->ActualT2_1_2->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->ActualT2_1_3->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->ActualT2_1_4->setText(QString::number(static_cast<double>(data->data[3]), 'f', 2));
        ui->ActualT2_2_1->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
        ui->ActualT2_2_2->setText(QString::number(static_cast<double>(data->data[5]), 'f', 2));
        ui->ActualT2_2_3->setText(QString::number(static_cast<double>(data->data[6]), 'f', 2));
        ui->ActualT2_2_4->setText(QString::number(static_cast<double>(data->data[7]), 'f', 2));
        ui->ActualT2_3_1->setText(QString::number(static_cast<double>(data->data[8]), 'f', 2));
        ui->ActualT2_3_2->setText(QString::number(static_cast<double>(data->data[9]), 'f', 2));
        ui->ActualT2_3_3->setText(QString::number(static_cast<double>(data->data[10]), 'f', 2));
        ui->ActualT2_3_4->setText(QString::number(static_cast<double>(data->data[11]), 'f', 2));
    }
    void control_panelWidget::MonitorWindow_BodyTwist_SubFunc(const std_msgs::Float32MultiArray::ConstPtr &data){
        ui->Twist_Omega1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->Twist_Omega2->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->Twist_Omega3->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->Twist_Vel1->setText(QString::number(static_cast<double>(data->data[3]), 'f', 2));
        ui->Twist_Vel2->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
        ui->Twist_Vel3->setText(QString::number(static_cast<double>(data->data[5]), 'f', 2));
    }

    // 在线神经网络训练
    void control_panelWidget::NeuralNetwork_CorCableForceProcess_SubFunc(const rxb_msgs::neuralNetworkProcess::ConstPtr &data){
//        ui->CorCableForce_Batch->setText(QString::number(data->batch, 'i', 0));
//        ui->CorCableForce_Loss->setText(QString::number(static_cast<double>(data->loss), 'f', 4));
//        ui->CorCableForce_ActualVal_1->setText(QString::number(static_cast<double>(data->actualVal[0]), 'f', 2));
//        ui->CorCableForce_ActualVal_2->setText(QString::number(static_cast<double>(data->actualVal[1]), 'f', 2));
//        ui->CorCableForce_ActualVal_3->setText(QString::number(static_cast<double>(data->actualVal[2]), 'f', 2));
//        ui->CorCableForce_ActualVal_4->setText(QString::number(static_cast<double>(data->actualVal[3]), 'f', 2));
//        ui->CorCableForce_ActualVal_5->setText(QString::number(static_cast<double>(data->actualVal[4]), 'f', 2));
//        ui->CorCableForce_ActualVal_6->setText(QString::number(static_cast<double>(data->actualVal[5]), 'f', 2));
//        ui->CorCableForce_PredictVal_1->setText(QString::number(static_cast<double>(data->predictVal[0]), 'f', 2));
//        ui->CorCableForce_PredictVal_2->setText(QString::number(static_cast<double>(data->predictVal[1]), 'f', 2));
//        ui->CorCableForce_PredictVal_3->setText(QString::number(static_cast<double>(data->predictVal[2]), 'f', 2));
//        ui->CorCableForce_PredictVal_4->setText(QString::number(static_cast<double>(data->predictVal[3]), 'f', 2));
//        ui->CorCableForce_PredictVal_5->setText(QString::number(static_cast<double>(data->predictVal[4]), 'f', 2));
//        ui->CorCableForce_PredictVal_6->setText(QString::number(static_cast<double>(data->predictVal[5]), 'f', 2));
    }
    void control_panelWidget::NeuralNetwork_CorCableForce_DataRecord_Count_SubFunc(const std_msgs::UInt32::ConstPtr& data){
        ui->CorCableForce_DataRecord_Count->setText(QString::number(data->data, 'i', 0));
    }
    void control_panelWidget::NeuralNetwork_CorCableForce_Model_Dev_Data_SubFunc(const std_msgs::Float32MultiArray::ConstPtr& data){
        ui->CorCableForce_Model_Dev_Act1->setText(QString::number(static_cast<double>(data->data[0]), 'f', 2));
        ui->CorCableForce_Model_Dev_Pred1->setText(QString::number(static_cast<double>(data->data[1]), 'f', 2));
        ui->CorCableForce_Model_Dev_Act2->setText(QString::number(static_cast<double>(data->data[2]), 'f', 2));
        ui->CorCableForce_Model_Dev_Pred2->setText(QString::number(static_cast<double>(data->data[3]), 'f', 2));
        ui->CorCableForce_Model_Dev_Loss->setText(QString::number(static_cast<double>(data->data[4]), 'f', 2));
    }

    void control_panelWidget::SupervisoryControl_JointControlCallback(rxb_msgs::jointControl inf){
        std_msgs::Float64MultiArray joint_position_command;
        joint_position_command.data.resize(Controllable_DOF_num_all);

        //规划关节角存储格式为PYPYPY
        joint_position_command.data[0] = inf.data[0];//ui->Tar_Plan_Seg1_P->value() * deg2rad;
        joint_position_command.data[1] = inf.data[1];
        joint_position_command.data[2] = inf.data[2];
        joint_position_command.data[3] = inf.data[3];
        joint_position_command.data[4] = inf.data[4];
        joint_position_command.data[5] = inf.data[5];

        //事实上，这里的发布者实现的功能是一模一样的，开环控制、力位混合控制和段闭环控制的不同点在于writeHW函数中
        if(rxb_mode == RxbMode_OLoop_Pos && motor_mode == MotorPositionMode) {
            std::cout << "rxb_openloop" << std::endl;
            openLoop_jointPosCmdPublisher_.publish(joint_position_command);
            std::cout << "Publish successfully!" << std::endl;

        }else if(rxb_mode == RxbMode_ForPos_Hybrid) {//rxb_mode == RxbMode_ForPos_Hybrid && motor_mode == MotorVelocityMode
            std::cout << "rxb_ForPos_Hybrid" << std::endl;
            ForPos_Hybrid_jointPosCmdPublisher_.publish(joint_position_command);
        }

        displayOutputInfos("black", "Joint Position Setting! Rxb running");

        for(int i = 0; i <Controllable_DOF_num_all; i++) {
            g_jnt_pub2movit[i]=joint_position_command.data[i];
        }
    }

    //rviz2moveit
    void rviz_control_plugin::control_panelWidget::rviz_pubjoint_2moveit(){
        //初始化/statejoint类型数据
        jointsCurrent2moveit.position.resize(24);
        for(int i = 0; i < Gene_Jnt_Num; i++) {
            jointsCurrent2moveit.name[i] = "joint" +std::to_string(i + 1);
        }

        for(int i = 0; i < ModuJnt_num_all; i++) {
            jointsCurrent2moveit.position[8 * i] = g_jnt_pub2movit[2 * i] ;
            jointsCurrent2moveit.position[8 * i + 3] = g_jnt_pub2movit[2 * i];
            jointsCurrent2moveit.position[8 * i + 4] = g_jnt_pub2movit[2 * i];
            jointsCurrent2moveit.position[8 * i + 7] = g_jnt_pub2movit[2 * i];

            jointsCurrent2moveit.position[8 * i + 1] = g_jnt_pub2movit[2 * i + 1] ;
            jointsCurrent2moveit.position[8 * i + 2] = g_jnt_pub2movit[2 * i + 1] ;
            jointsCurrent2moveit.position[8 * i + 5] = g_jnt_pub2movit[2 * i + 1] ;
            jointsCurrent2moveit.position[8 * i + 6] = g_jnt_pub2movit[2 * i + 1] ;
        }
       rviz_Jnt_Pub.publish(jointsCurrent2moveit);//这里没继续
    };

    control_panelWidget::~control_panelWidget() {
        delete ui;
    }
    void control_panelWidget::Sensor_Encoder_DataCallback(const rxb_msgs::sensorRead::ConstPtr &data) {//接收readHW()函数读取的角度编码器数据

        for(int i = 0; i < Gene_Jnt_Num; i++) {
            g_plugin_jnt_pos[i] = data->joint_angle[i] ;
            //std::cout << "in data callback"  << data->joint_angle[i] << std::endl;
        }
        updateJointState(0);//将变量g_plugin_jnt_pos传递给界面
    }

    void control_panelWidget::Sensor_CableForce_DataCallback(const rxb_msgs::sensorRead::ConstPtr &data) {
        for(int i = 0; i < TensionSensor_num_all; i++) {
            g_plugin_cableF[i] = data->cable_force[i];
        }
        updateJointState(4);
    }
    void control_panelWidget::PlaningJntPos_DataCallback(const std_msgs::Float64MultiArray::ConstPtr &PlaningJntPos)
    {
        ui->joint1_plan_state->setText(QString::number(PlaningJntPos->data[0], 'f', 2));
        ui->joint2_plan_state->setText(QString::number(PlaningJntPos->data[1], 'f', 2));
//        ui->joint3_plan_state->setText(QString::number(PlaningJntPos->data[2], 'f', 2));
//        ui->joint4_plan_state->setText(QString::number(PlaningJntPos->data[3], 'f', 2));
//        ui->joint5_plan_state->setText(QString::number(PlaningJntPos->data[4], 'f', 2));
//        ui->joint6_plan_state->setText(QString::number(PlaningJntPos->data[5], 'f', 2));

    }


    void control_panelWidget::Motor_Run(const std_msgs::Float64MultiArray::ConstPtr &motor_pos){
        std::cout << "Motor_Run" << std::endl;
        for (int i=0;i<Motor_num_all;i++) {
            std::cout << "Motor_Poshou:" << motor_pos->data[i] << std::endl;
        }
    }


    //void control_panelWidget::jointStateCallback(const std_msgs::Float64MultiArray::ConstPtr &joint_state) {
        //for(int i = 0; i < Active_passive_jnt_num_all; i++) {
            //g_plugin_jnt_pos[i] = joint_state->data[i];}


        //updateJointState(0);
    //}

    void control_panelWidget::rxbStateCallback(const std_msgs::UInt32::ConstPtr &rxb_state) {
        uint data = rxb_state->data;
        if(data == 1) {
            g_rxb_state = RxbRunning;
        } else if(data == 0) {
            g_rxb_state = RxbStopping;
        }
    }

//    void control_panelWidget::motorModeCallback(const std_msgs::UInt32::ConstPtr &motor_mode) {
//        uint data = motor_mode->data;
//        if(data == MotorVelocityMode) {
//            ui->DriverMode_Display->setText(tr("Velocity_mode"));
//        } else if(data == MotorPositionMode) {
//            ui->DriverMode_Display->setText(tr("Position_mode"));
//        }
//    }

    void control_panelWidget::motorPositionCallback(const std_msgs::Float64MultiArray::ConstPtr &motor_pos) {
        int j;
        for(int i = 0; i < Motor_num_all; i++) {
           g_plugin_mot_pos[i]   =   motor_pos->data[i];

        }
        updateJointState(1);
    }
//    void control_panelWidget::motorVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr &motor_vel) {
//        int j;
//        for(int i = 0; i < 9; i++) {
//            g_plugin_mot_vel[i]   =   motor_vel->data[i];
//        }
//        //updateJointState(1);
//    }
//    void control_panelWidget::motorCurrentCallback(const std_msgs::Float64MultiArray::ConstPtr &motor_cur) {
//        int j;
//        for(int i = 0; i < 9; i++) {
//            g_plugin_mot_cur[i]   =   motor_cur->data[i];
//        }
//        //updateJointState(1);
//    }

//    void control_panelWidget::CableForceCallback(const std_msgs::Float64MultiArray::ConstPtr &cable_force) {
//        for(int i = 0; i < Motor_num_all; i++) {
//            g_plugin_cableF[i] = cable_force->data[i];
//        }
//        updateJointState(4);
//    }
//    void control_panelWidget::l_ExternalForceCallback(const geometry_msgs::Vector3::ConstPtr &l_externalF) {
//        l_plugin_externalF[0] = l_externalF->x;
//        l_plugin_externalF[1] = l_externalF->y;
//        l_plugin_externalF[2] = l_externalF->z;
//    }



    void control_panelWidget::gripperStateCallback(const std_msgs::Int32::ConstPtr &gripper_state) {
        g_plugin_gripper_pos =  gripper_state->data;
        updateJointState(3);
    }




    /*     plugin   show    current     state*/
    /*  0=joint_current_state   1=cable_current_state   2=endpos_current_state  3=gripper_current_state     */

    void control_panelWidget::updateJointState(int index) {
        //std::cout << "in update" << std::endl;
        switch (index) {
            case 0: { //joint state
                    ui->segment1_pitch1->setText(QString::number(g_plugin_jnt_pos[1], 'f', 2));
                    ui->segment1_yaw1->setText(QString::number(g_plugin_jnt_pos[0], 'f', 2));
                    ui->segment1_pitch2->setText(QString::number(g_plugin_jnt_pos[2], 'f', 2));
                    ui->segment1_yaw2->setText(QString::number(g_plugin_jnt_pos[3], 'f', 2));
                    ui->segment1_pitch3->setText(QString::number(g_plugin_jnt_pos[5], 'f', 2));
                    ui->segment1_yaw3->setText(QString::number(g_plugin_jnt_pos[4], 'f', 2));
                    ui->segment1_pitch4->setText(QString::number(g_plugin_jnt_pos[6], 'f', 2));
                    ui->segment1_yaw4->setText(QString::number(g_plugin_jnt_pos[7], 'f', 2));
                    ui->segment1_pitch_average->setText(QString::number((g_plugin_jnt_pos[1]+g_plugin_jnt_pos[2]+g_plugin_jnt_pos[5]+g_plugin_jnt_pos[6])/4, 'f', 2));
                    ui->segment1_yaw_average->setText(QString::number((g_plugin_jnt_pos[0]+g_plugin_jnt_pos[3]+g_plugin_jnt_pos[4]+g_plugin_jnt_pos[7])/4, 'f', 2));
                    break;
                }
            case 1: { //cable state

                    break;
                }
            case 2: { //target state

                    break;

                }
            case 3: { //gripper state
                    break;
                }
            case 4: {
                    ui->segment1_cable1->setText(QString::number(g_plugin_cableF[CaptureCard_num_all+0], 'f', 2));
                    ui->segment1_cable2->setText(QString::number(g_plugin_cableF[CaptureCard_num_all+1], 'f', 2));
                    ui->segment1_cable3->setText(QString::number(g_plugin_cableF[CaptureCard_num_all+2], 'f', 2));

                    ui->cable1_force_4->setText(QString::number(g_plugin_cableF[4], 'f', 2));
                    ui->cable2_force_4->setText(QString::number(g_plugin_cableF[5], 'f', 2));
                    ui->cable3_force_4->setText(QString::number(g_plugin_cableF[7], 'f', 2));
                    ui->cable4_force_4->setText(QString::number(g_plugin_cableF[6], 'f', 2));
                    ui->cable5_force_4->setText(QString::number(g_plugin_cableF[9], 'f', 2));
                    ui->cable6_force_4->setText(QString::number(g_plugin_cableF[8], 'f', 2));
                    ui->cable7_force_4->setText(QString::number(g_plugin_cableF[16], 'f', 2));
                    ui->cable8_force_4->setText(QString::number(g_plugin_cableF[17], 'f', 2));
                    ui->cable9_force_4->setText(QString::number(g_plugin_cableF[0], 'f', 2));
                    ui->cable10_force_4->setText(QString::number(g_plugin_cableF[1], 'f', 2));
                    ui->cable11_force_4->setText(QString::number(g_plugin_cableF[2], 'f', 2));
                    ui->cable12_force_4->setText(QString::number(g_plugin_cableF[3], 'f', 2));
                    ui->cable13_force_4->setText(QString::number(g_plugin_cableF[11], 'f', 2));
                    ui->cable14_force_4->setText(QString::number(g_plugin_cableF[10], 'f', 2));
                    ui->cable15_force_4->setText(QString::number(g_plugin_cableF[12], 'f', 2));
                    ui->cable16_force_4->setText(QString::number(g_plugin_cableF[13], 'f', 2));
                    ui->cable17_force_4->setText(QString::number(g_plugin_cableF[14], 'f', 2));
                    ui->cable18_force_4->setText(QString::number(g_plugin_cableF[15], 'f', 2));

                    break;
                }
            case 5: {
                    ui->ExForceMeasure_Mx->setText(QString::number(g_plugin_exForceMeasure[0], 'f', 2));
                    ui->ExForceMeasure_My->setText(QString::number(g_plugin_exForceMeasure[1], 'f', 2));
                    ui->ExForceMeasure_Mz->setText(QString::number(g_plugin_exForceMeasure[2], 'f', 2));
                    ui->ExForceMeasure_Fx->setText(QString::number(g_plugin_exForceMeasure[3], 'f', 2));
                    ui->ExForceMeasure_Fy->setText(QString::number(g_plugin_exForceMeasure[4], 'f', 2));
                    ui->ExForceMeasure_Fz->setText(QString::number(g_plugin_exForceMeasure[5], 'f', 2));
                    break;
                }
            case 6: {
                //ui->joint1_plan_state->setText(QString::number(g_plugin_jnt_pos[0], 'f', 2));
                //ui->l_externalF_y->setText(QString::number(l_plugin_externalF[1], 'f', 2));
                //ui->l_externalF_z->setText(QString::number(l_plugin_externalF[2], 'f', 2));
                break;
            }
            case 7: {
                ui->ExForceSense_Fx->setText(QString::number(g_plugin_exForceSense[0], 'f', 2));
                ui->ExForceSense_Fy->setText(QString::number(g_plugin_exForceSense[1], 'f', 2));
                break;
            }
            case 8: {
//                ui->CorCableForce_Batch->setText(QString::number(g_plugin_corCableForce_batch, 'i'));
//                ui->CorCableForce_ActualVal->setText(QString::number(g_plugin_corCableForce_actualVal, 'i'));
//                ui->CorCableForce_Loss->setText(QString::number(g_plugin_corCableForce_loss, 'i'));
//                ui->CorCableForce_PredictVal->setText(QString::number(g_plugin_corCableForce_predictVal, 'i'));
            }
        }
        return ;
    }



    /*  display     configure   */
    void control_panelWidget::displayOutputInfos(const std::string &color, const QString &context) {
        if(color == "red") {
            //ui->InfoOutputs->setTextColor(QColor(255, 0, 0));//InfoOutputs删除[2023.4.10]
            //ui->InfoOutputs->insertPlainText(QString("ERROR : ") + context + QString("\n"));
        }
        if(color == "black") {
            //ui->InfoOutputs->setTextColor(QColor(0, 0, 0));//0 255 0 green
            //ui->InfoOutputs->insertPlainText(QString("INFO : ") + context + QString("\n"));
        }
        if(color == "yellow") {
            //ui->InfoOutputs->setTextColor(QColor(255, 255, 0));
            //ui->InfoOutputs->insertPlainText(QString("WARN : ") + context + QString("\n"));
        }

    }



    /*      Rxb  Mode         */
/*
    void control_panelWidget::on_RxbInitialize_clicked() {
        std_msgs::UInt32 flag_msg;
        flag_msg.data = isRxbOnFlag;
        flagPublisher_.publish(flag_msg);
        displayOutputInfos("black", "Rxb Initilaizing.....");
    }
    */
/* //RxbWorking 按钮删除[2023.4.10]
    void control_panelWidget::on_RxbWorking_clicked() {
        std_msgs::UInt32 flag_msg;
        flag_msg.data = isJointCommadFlag;
        flagPublisher_.publish(flag_msg);

        std_msgs::Float64MultiArray joint_position_command;
        joint_position_command.data.resize(Controllable_DOF_num_all);


        joint_position_command.data[0] =       0 * deg2rad;
        joint_position_command.data[1] =   -14.1 * deg2rad;

        joint_position_command.data[2] =      29 * deg2rad;
        joint_position_command.data[3] =       0 * deg2rad;

        joint_position_command.data[4] =       0 * deg2rad;
        joint_position_command.data[5] =    31.5 * deg2rad;

        //joint_position_command.data[6] =    31.5 * deg2rad;
        //joint_position_command.data[7] =       0 * deg2rad;



        if(rxb_mode == RxbMode_OLoop_Pos) {
            openLoop_jointPosCmdPublisher_.publish(joint_position_command);
        } else if(rxb_mode == RxbMode_CLoop_Pos) {
            closeLoop_jointPosCmdPublisher_.publish(joint_position_command);
        }
        displayOutputInfos("black", "Rxb Working.....");
    }
    */
/*   //RxbAutoback按钮删除[2023.4.10]
    void control_panelWidget::on_RxbAutoback_clicked() {
        std_msgs::UInt32 flag_msg;
        flag_msg.data = isJointCommadFlag;
        flagPublisher_.publish(flag_msg);

        std_msgs::Float64MultiArray joint_position_command;
        joint_position_command.data.resize(Controllable_DOF_num_all);


        joint_position_command.data[0] =    0 * deg2rad;
        joint_position_command.data[1] =    0 * deg2rad;

        joint_position_command.data[2] =    0 * deg2rad;
        joint_position_command.data[3] =    0 * deg2rad;

        joint_position_command.data[4] =    0 * deg2rad;
        joint_position_command.data[5] =    0 * deg2rad;

        //joint_position_command.data[6] =    0 * deg2rad;
        //joint_position_command.data[7] =    0 * deg2rad;


        if(rxb_mode == RxbMode_OLoop_Pos) {
            openLoop_jointPosCmdPublisher_.publish(joint_position_command);
        } else if(rxb_mode == RxbMode_CLoop_Pos) {
            closeLoop_jointPosCmdPublisher_.publish(joint_position_command);
        }
        displayOutputInfos("black", "Rxb Back to Initialization.....");
    }
*/
    void control_panelWidget::on_RxbShutdown_clicked() {
        std_msgs::UInt32 flag_msg;
        flag_msg.data = isRxbOffFlag;
        flagPublisher_.publish(flag_msg);
    }

    void control_panelWidget::on_RxbMode1_Button_clicked() {
        rxb_msgs::RxbMode data;
        data.request.mode = RxbMode_OLoop_Pos;
        if (RxbModeClient_.call(data)) {
            rxb_mode = RxbMode_OLoop_Pos;
            displayOutputInfos("black", "set rxb_openloop_mode successfully");
        }

    }
/*
//原来的闭环模式，没什么用，删掉了
    void control_panelWidget::on_RxbMode2_Button_clicked() {
        rxb_msgs::RxbMode data;
        data.request.mode = RxbMode_CLoop_Pos;
        if (RxbModeClient_.call(data)) {
            rxb_mode = RxbMode_CLoop_Pos;
            displayOutputInfos("black", "set rxb_closeloop_mode successfully");
        }
    }
    */

    void control_panelWidget::on_RxbMode2_Button_2_clicked() {
        rxb_msgs::RxbMode data;
        data.request.mode = RxbMode_ForPos_Hybrid;        
        if (RxbModeClient_.call(data)) {
            rxb_mode = RxbMode_ForPos_Hybrid;
            displayOutputInfos("black", "set rxb_ForPos_Hybrid_mode successfully");
        }
    }

    void control_panelWidget::on_TrajectoryDemo1_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 1;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo2_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 2;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo3_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 3;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo4_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 4;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo5_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 5;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo6_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 6;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo7_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 7;
        trajectoryDemo_pub.publish(data);
    }

    void control_panelWidget::on_TrajectoryDemo8_clicked() {
        rxb_msgs::trajectoryDemo data;
        if(ui->TrajectoryDemo_Segmental->isChecked()){
            data.mode = 0;
        }else{
            data.mode = 1;
        }
        data.ID = 8;
        trajectoryDemo_pub.publish(data);
//        std_msgs::UInt32 traj;
//        traj.data = TrajectoryDemo + 8;
//        rxbTrajectoryPublisher_.publish(traj);
    }

    /*  joint   control */
    /*  set     stop    reset      getnow   */
    void control_panelWidget::on_setJointPositionButton_clicked() { //position control  ->set
        std_msgs::Float64MultiArray joint_position_command;
        joint_position_command.data.resize(Controllable_DOF_num_all);

        //规划关节角存储格式为PYPYPY
        joint_position_command.data[0] = ui->Tar_Plan_Seg1_P->value() * deg2rad;
        joint_position_command.data[1] = ui->Tar_Plan_Seg1_Y->value() * deg2rad;

        //事实上，这里的发布者实现的功能是一模一样的，开环控制、力位混合控制和段闭环控制的不同点在于writeHW函数中
        if(rxb_mode == RxbMode_OLoop_Pos && motor_mode == MotorPositionMode) {
            std::cout << "rxb_openloop" << std::endl;
            openLoop_jointPosCmdPublisher_.publish(joint_position_command);
            std::cout << "Publish successfully!" << std::endl;
        }else if(rxb_mode == RxbMode_CLoop_Pos && motor_mode == MotorVelocityMode){
            std::cout << "rxb_ForPos_Hybrid" << std::endl;
            closeLoop_jointPosCmdPublisher_.publish(joint_position_command);
        }else if(rxb_mode == RxbMode_ForPos_Hybrid && motor_mode == MotorVelocityMode) {
            std::cout << "rxb_ForPos_Hybrid" << std::endl;
            ForPos_Hybrid_jointPosCmdPublisher_.publish(joint_position_command);
        }

        displayOutputInfos("black", "Joint Position Setting! Rxb running");

        for(int i = 0; i <Controllable_DOF_num_all; i++) {
            g_jnt_pub2movit[i]=joint_position_command.data[i];
        }
    }


    void control_panelWidget::on_stopJointPositionButton_clicked() {
//        std_msgs::UInt32 flag_msg;
//        flag_msg.data = isRxbOffFlag;
//        flagPublisher_.publish(flag_msg);
//        displayOutputInfos("yellow", "Joint Position Stopping! Rxb stoppiing!!");
        std_msgs::UInt32 flag;
        flag.data = 1;
         pausePublisher_.publish(flag);
    }


    void control_panelWidget::on_resetJointPositionButton_clicked() { //position control  ->reset
        ui->Tar_Plan_Seg1_P->setValue(0);
        ui->Tar_Plan_Seg1_Y->setValue(0);
        ui->Tar_Plan_Seg2_P->setValue(0);
        ui->Tar_Plan_Seg2_Y->setValue(0);
        ui->Tar_Plan_Seg3_P->setValue(0);
        ui->Tar_Plan_Seg3_Y->setValue(0);
        //ui->joint7_position->setValue(0);
        //ui->joint8_position->setValue(0);
    }


    void control_panelWidget::on_getnowJointPositionButton_clicked() {
        ui->Tar_Plan_Seg1_P->setValue((g_plugin_jnt_pos[3] + g_plugin_jnt_pos[4])/2);//插眼
        ui->Tar_Plan_Seg1_Y->setValue((g_plugin_jnt_pos[2] + g_plugin_jnt_pos[5])/2);
//        ui->Tar_Plan_Seg2_P->setValue((g_plugin_jnt_pos[11] + g_plugin_jnt_pos[12])/2);
//        ui->Tar_Plan_Seg2_Y->setValue((g_plugin_jnt_pos[10] + g_plugin_jnt_pos[13])/2);
//        ui->Tar_Plan_Seg3_P->setValue((g_plugin_jnt_pos[19] + g_plugin_jnt_pos[20])/2);
//        ui->Tar_Plan_Seg3_Y->setValue((g_plugin_jnt_pos[18] + g_plugin_jnt_pos[21])/2);

        //ui->joint7_position->setValue(g_plugin_jnt_pos[6]);
        //ui->joint8_position->setValue(g_plugin_jnt_pos[7]);
    }



    /*  cable   control */
    /*  set     stop    reset      getnow   */
    void control_panelWidget::on_setCablePositionButton_clicked() { //position control  ->reset
        std_msgs::UInt32 flag_msg;
        flag_msg.data = isRxbOnFlag;

        std_msgs::Float64MultiArray cable_position_command;
        cable_position_command.data.resize(Motor_num_all);

        cable_position_command.data[0] = ui->cable1_position->value();
        cable_position_command.data[1] = ui->cable2_position->value();
        cable_position_command.data[2] = ui->cable3_position->value();

        cable_position_command.data[3] = ui->cable4_position->value();
        cable_position_command.data[4] = ui->cable5_position->value();
        cable_position_command.data[5] = ui->cable6_position->value();

        cable_position_command.data[6] = ui->cable7_position->value();
        cable_position_command.data[7] = ui->cable8_position->value();
        cable_position_command.data[8] = ui->cable9_position->value();

        cable_position_command.data[9] = ui->cable10_position->value();
        //cable_position_command.data[10] = ui->cable11_position->value();
        //cable_position_command.data[11] = ui->cable12_position->value();



        motorPositionCommandPublisher_.publish(cable_position_command);

        flagPublisher_.publish(flag_msg);
        displayOutputInfos("black", "Cable Position Setting! Rxb running!!");

    }


    void control_panelWidget::on_stopCablePositionButton_clicked() {
//        std_msgs::UInt32 flag_msg;
//        flag_msg.data = isRxbOffFlag;
//        flagPublisher_.publish(flag_msg);
//        displayOutputInfos("yellow", "Cable Position Stopping! Rxb stoppiing!!");
    }

    void control_panelWidget::on_resetCablePositionButton_clicked() {
        ui->cable1_position->setValue(0);
        ui->cable2_position->setValue(0);
        ui->cable3_position->setValue(0);

        ui->cable4_position->setValue(0);
        ui->cable5_position->setValue(0);
        ui->cable6_position->setValue(0);

        ui->cable7_position->setValue(0);
        ui->cable8_position->setValue(0);
        ui->cable9_position->setValue(0);

        ui->cable10_position->setValue(0);
        //ui->cable11_position->setValue(0);
        //ui->cable12_position->setValue(0);


    }

    void control_panelWidget::on_getnowCablePositionButton_clicked() {
        ui->cable1_position->setValue(g_plugin_mot_pos[0]);
        ui->cable2_position->setValue(g_plugin_mot_pos[1]);
        ui->cable3_position->setValue(g_plugin_mot_pos[2]);

//        ui->cable4_position->setValue(g_plugin_mot_pos[3]);
//        ui->cable5_position->setValue(g_plugin_mot_pos[4]);
//        ui->cable6_position->setValue(g_plugin_mot_pos[5]);

//        ui->cable7_position->setValue(g_plugin_mot_pos[6]);
//        ui->cable8_position->setValue(g_plugin_mot_pos[7]);
//        ui->cable9_position->setValue(g_plugin_mot_pos[8]);

        //ui->cable10_position->setValue(g_plugin_mot_pos[9]);
        //ui->cable11_position->setValue(g_plugin_mot_pos[10]);
        //ui->cable12_position->setValue(g_plugin_mot_pos[11]);
    }

    void control_panelWidget::on_CalculateTargetPositionButton_clicked() {
        rxb_msgs::RxbPose data;
        geometry_msgs::Pose pose;
        data.request.cmd = true;
        getPoseClient_.call(data);
        if(data.response.success == true) {
            pose = data.response.pose;
            g_plugin_tar_pos[0] = pose.position.x;
            g_plugin_tar_pos[1] = pose.position.y;
            g_plugin_tar_pos[2] = pose.position.z;
            g_plugin_tar_pos[3] = pose.orientation.x;
            g_plugin_tar_pos[4] = pose.orientation.y;
            g_plugin_tar_pos[5] = pose.orientation.z;
            g_plugin_tar_pos[6] = pose.orientation.w;
            updateJointState(2);
        }


    }

    /*     Force    Senor        */
/*
    void control_panelWidget::on_externalF_cau_button_clicked() {
        std_msgs::Float64MultiArray all_cable_force, all_joint_position;
        all_cable_force.data.resize(12);
        all_joint_position.data.resize(8);
        for(int i = 0; i < Motor_num_all; i++) {
            all_cable_force.data[i] = g_plugin_cableF[i];
        }
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            all_joint_position.data[i] = g_plugin_jnt_pos[i];
        }
        matlab_cableForceInfoPublisher_.publish(all_cable_force);
        matlab_jointPositionInfoPublisher_.publish(all_joint_position);
    }
    */

/*
    void rviz_control_plugin::control_panelWidget::on_position_mode_button_clicked() {
        rxb_msgs::MotorMode motorMode;
        if(g_rxb_state != RxbRunning && rxb_mode == RxbMode_OLoop_Pos) {
            if(motor_mode == MotorPositionMode ) {
                motorMode.request.mode = MotorVelocityMode;
                motor_mode = MotorVelocityMode;
                if(MotorModeClient_.call(motorMode)) {
                    displayOutputInfos("black", "set motor_Velocity_mode successfully");
                    ui->DriverMode_Display->setText("Velocity_Mode");
                }
            } else if(motor_mode == MotorVelocityMode ) {
                motorMode.request.mode = MotorPositionMode;
                motor_mode = MotorPositionMode;
                if(MotorModeClient_.call(motorMode)) {
                    displayOutputInfos("black", "set motor_Position_mode successfully");
                    ui->DriverMode_Display->setText("Position_Mode");
                }
            }
        } else {
            displayOutputInfos("red", "rxb is running or not openloop_mode!");
        }

    }
    */
    /*
    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_1_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 1;
        motorMovePublisher_.publish(rxb_move);
    }
    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_1_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 1;
        motorMovePublisher_.publish(rxb_move);
    }
    */

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_13_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 1;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_2_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 2;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_2_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 2;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_2_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 2;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_2_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 2;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_3_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 3;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_3_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 3;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_3_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 3;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_3_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 3;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_4_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 4;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_4_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 4;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_4_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 4;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_4_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 4;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_5_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 5;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_5_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 5;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_5_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 5;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_5_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 5;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_6_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 6;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_6_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 6;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_6_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 6;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_6_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 6;
        motorMovePublisher_.publish(rxb_move);
    }
/*
    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_7_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 7;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_7_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 7;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_7_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 7;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_7_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 7;
        motorMovePublisher_.publish(rxb_move);
    }
*/
    /*
    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_8_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 8;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_8_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 8;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_8_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 8;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_8_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 8;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_9_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 9;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_9_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 9;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_9_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 9;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_9_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 9;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_10_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 10;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_10_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 10;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_10_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 10;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_10_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 10;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_11_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 11;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_11_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 11;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_11_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 11;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_11_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 11;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_12_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All + 12;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_12_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 12;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_12_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All + 12;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_CablePull_Button_12_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All + 12;
        motorMovePublisher_.publish(rxb_move);
    }
    */
    void rviz_control_plugin::control_panelWidget::on_LooseAllCable_Button_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableLoose_All ;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_LooseAllCable_Button_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All ;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_PullAllCable_Button_pressed() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CablePull_All ;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_PullAllCable_Button_released() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = CableStop_All ;
        motorMovePublisher_.publish(rxb_move);
    }

    void rviz_control_plugin::control_panelWidget::on_clearInfoButton_clicked() {
        //ui->InfoOutputs->clear();
    }
    /*
    void rviz_control_plugin::control_panelWidget::on_resetMotor_button_clicked() {
        std_msgs::UInt32 rxb_move;
        rxb_move.data = resetMotor ;
        motorMovePublisher_.publish(rxb_move);
        motor_mode = MotorPositionMode;
        ui->DriverMode_Display->setText("Position_Mode");
        displayOutputInfos("black", "reset motors successfully.");
    }
    */
    void rviz_control_plugin::control_panelWidget::on_pauseButton_clicked(){
      std_msgs::UInt32 flag;
      flag.data = 1;
       pausePublisher_.publish(flag);
    }


void control_panelWidget::on_CableLoose_Driven2_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 2;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven2_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 2;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven1_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 1;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven3_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 3;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven4_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 4;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven5_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 5;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven6_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 6;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven7_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 7;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven8_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 8;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven9_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 9;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven1_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 1;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven3_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 3;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven4_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 4;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven5_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 5;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven6_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 6;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven7_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 7;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven8_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 8;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven9_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 9;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven1_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 1;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven2_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 2;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven3_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 3;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven4_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 4;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven5_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 5;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven6_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 6;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven7_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 7;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven8_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 8;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven9_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 9;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven1_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 1;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven2_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 2;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven3_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 3;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven4_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 4;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven5_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 5;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven6_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 6;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven7_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 7;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven8_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 8;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven9_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 9;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven10_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 10;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven11_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 11;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven12_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 12;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven10_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 10;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven11_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 11;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven12_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 12;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven10_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 10;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven10_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 10;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven11_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 11;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven11_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 11;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CableLoose_Driven12_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 12;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_CablePull_Driven12_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 12;
    motorMovePublisher_.publish(rxb_move);
}


void control_panelWidget::on_zero_2_clicked()
{
    std_msgs::UInt32 flag;
    flag.data = 2;
    ZeroFlagPublisher_.publish(flag);
}


void control_panelWidget::on_zero_3_clicked()
{
    std_msgs::UInt32 flag;
    flag.data = 3;
    ZeroFlagPublisher_.publish(flag);
}


void control_panelWidget::on_J1_PosePID_set_3_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(5);
    datas.data[0] = ui->Pos_P3->value();
    datas.data[1] = ui->Pos_I3->value();
    datas.data[2] = ui->Pos_D3->value();
    datas.data[3] = 0;//0代表位置pid
    datas.data[4] = 3;//1代表第一个可控关节
    J1posPIDPublisher_.publish(datas);
}

void control_panelWidget::on_J1_PosePID_set_11_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(5);
    datas.data[0] = ui->Force_P1->value();
    datas.data[1] = ui->Force_I1->value();
    datas.data[2] = ui->Force_D1->value();
    datas.data[3] = 1;//代表力pid
    datas.data[4] = 1;//第一组驱动
    J1posPIDPublisher_.publish(datas);
}


void control_panelWidget::on_J2_PosePID_set_13_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(5);
    datas.data[0] = ui->Force_P2->value();
    datas.data[1] = ui->Force_I2->value();
    datas.data[2] = ui->Force_D2->value();
    datas.data[3] = 1;//代表力pid
    datas.data[4] = 2;//第一组驱动
    J1posPIDPublisher_.publish(datas);
}


void control_panelWidget::on_J1_PosePID_set_12_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(5);
    datas.data[0] = ui->Force_P3->value();
    datas.data[1] = ui->Force_I3->value();
    datas.data[2] = ui->Force_D3->value();
    datas.data[3] = 1;//代表力pid
    datas.data[4] = 3;//第一组驱动
    J1posPIDPublisher_.publish(datas);
}


void control_panelWidget::on_All_ForcePID_para_writeIn_5_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(ModuJnt_num_all*3);
    datas.data[0] = ui->Force_P1->value();
    datas.data[1] = ui->Force_I1->value();
    datas.data[2] = ui->Force_D1->value();

//    datas.data[3] = ui->Force_P2->value();
//    datas.data[4] = ui->Force_I2->value();
//    datas.data[5] = ui->Force_D2->value();

//    datas.data[6] = ui->Force_P3->value();
//    datas.data[7] = ui->Force_I3->value();
//    datas.data[8] = ui->Force_D3->value();

    ForcePIDWriteInPublisher_.publish(datas);
}


void control_panelWidget::on_get_ForcePID_value_from_file_5_clicked()
{
    //函数作用：点击“get PID value from file”后将PID参数改为文件中的值
    //参数定义
    double idata;
    double para_kp[ModuJnt_num_all];
    double para_ki[ModuJnt_num_all];
    double para_kd[ModuJnt_num_all];
    int i = 0;

    //获取文件中的值
    ifs_PID.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/ForcePIDSaved.txt", std::ios::in);//读PID数
    if (!ifs_PID.is_open()) {
        std::cout << "文件打开失败" << std::endl;
        RxbHardwareInterface::can_close();
        ros::shutdown();
    }
    while (ifs_PID >> idata) {//从文件中读取关节角
        para_kp[i] = idata;
        ifs_PID >> idata;
        para_ki[i] = idata;
        ifs_PID >> idata;
        para_kd[i++] = idata;
    }
    ifs_PID.close();
    std::cout << "Total i:" << i << std::endl;

    //修改UI上的值
    ui->Force_P1->setValue(para_kp[0]);
    ui->Force_P2->setValue(para_kp[1]);
    ui->Force_P3->setValue(para_kp[2]);

    ui->Force_I1->setValue(para_ki[0]);
    ui->Force_I2->setValue(para_ki[1]);
    ui->Force_I3->setValue(para_ki[2]);

    ui->Force_D1->setValue(para_kd[0]);
    ui->Force_D2->setValue(para_kd[1]);
    ui->Force_D3->setValue(para_kd[2]);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_control_plugin::control_panelWidget, rviz::Panel)


void rviz_control_plugin::control_panelWidget::on_zero_clicked()
{
    std_msgs::UInt32 flag;
    flag.data = 1;
    ZeroFlagPublisher_.publish(flag);
}

void rviz_control_plugin::control_panelWidget::on_RxbMode4_Button_clicked()
{
    //if (RxbHardwareInterface::g_rxbWorkStatus){
        rxb_msgs::RxbMode data;
        data.request.mode = RxbMode_Adjust_Cable;
        if (RxbModeClient_.call(data)) {
            rxb_mode = RxbMode_Adjust_Cable;
            displayOutputInfos("black", "set rxb_Adjust_Cable_mode successfully");
        }
    //}
}
/*
void rviz_control_plugin::control_panelWidget::on_Cable_Button_1_clicked()
{
    rxb_msgs::RxbMode data;
    data.request.mode = RxbMode_Reset;
    if (RxbModeClient_.call(data)) {
        rxb_mode = RxbMode_Reset;
        displayOutputInfos("black", "reset successfully");
    }
}
*/
void rviz_control_plugin::control_panelWidget::on_RxbMode3_Button_clicked()
{
        rxb_msgs::RxbMode data;
        data.request.mode = RxbMode_ForPos_Hybrid;
        if (RxbModeClient_.call(data)) {
            rxb_mode = RxbMode_ForPos_Hybrid;
            displayOutputInfos("black", "set rxb_ForPos_Hybrid_mode successfully");
        }
}


void rviz_control_plugin::control_panelWidget::on_J1_PosePID_set_clicked()
{
      std_msgs::Float64MultiArray datas;
      datas.data.resize(5);
      datas.data[0] = ui->Pos_P1->value();
      datas.data[1] = ui->Pos_I1->value();
      datas.data[2] = ui->Pos_D1->value();
      datas.data[3] = 0;//0代表位置pid
      datas.data[4] = 1;//1代表第一个可控关节
      J1posPIDPublisher_.publish(datas);
}

void rviz_control_plugin::control_panelWidget::on_J2_PosePID_set_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(5);
    datas.data[0] = ui->Pos_P2->value();
    datas.data[1] = ui->Pos_I2->value();
    datas.data[2] = ui->Pos_D2->value();
    datas.data[3] = 0;//0代表位置pid
    datas.data[4] = 2;//1代表第一个可控关节
    //std::cout << "  " << datas.data[0] << "    " << datas.data[1] << "   "  <<  datas.data[2] << "   " <<  datas.data[3] << std::endl;
    J1posPIDPublisher_.publish(datas);
}
/*
void rviz_control_plugin::control_panelWidget::on_J3_pid_set_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(4);
    datas.data[0] = ui->Pos_P3->value();
    datas.data[1] = ui->Pos_I3->value();
    datas.data[2] = ui->Pos_D3->value();
    datas.data[3] = 3;//1代表第一个可控关节
    J1posPIDPublisher_.publish(datas);
}
*/
/*
void rviz_control_plugin::control_panelWidget::on_J4_pid_set_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(4);
    datas.data[0] = ui->Pos_P4->value();
    datas.data[1] = ui->Pos_I4->value();
    datas.data[2] = ui->Pos_D4->value();
    datas.data[3] = 4;//1代表第一个可控关节
    J1posPIDPublisher_.publish(datas);
}
*/
/*
void rviz_control_plugin::control_panelWidget::on_J5_pid_set_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(4);
    datas.data[0] = ui->Pos_P5->value();
    datas.data[1] = ui->Pos_I5->value();
    datas.data[2] = ui->Pos_D5->value();
    datas.data[3] = 5;//1代表第一个可控关节
    J1posPIDPublisher_.publish(datas);
}
*/
/*
void rviz_control_plugin::control_panelWidget::on_J6_pid_set_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(4);
    datas.data[0] = ui->Pos_P6->value();
    datas.data[1] = ui->Pos_I6->value();
    datas.data[2] = ui->Pos_D6->value();
    datas.data[3] = 6;//1代表第一个可控关节
    J1posPIDPublisher_.publish(datas);
}
*/
/*
void rviz_control_plugin::control_panelWidget::on_Use_J1_all_set_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(4);
    datas.data[0] = ui->Pos_P1->value();
    datas.data[1] = ui->Pos_I1->value();
    datas.data[2] = ui->Pos_D1->value();
    datas.data[3] = 7;//1代表第一个可控关节
    J1posPIDPublisher_.publish(datas);
}
*/
void rviz_control_plugin::control_panelWidget::on_get_PosePID_value_from_file_clicked()
{
    //函数作用：点击“get PID value from file”后将PID参数改为文件中的值
    //参数定义
    double idata;
    double para_kp[Controllable_DOF_num_all];
    double para_ki[Controllable_DOF_num_all];
    double para_kd[Controllable_DOF_num_all];
    int i = 0;

    //获取文件中的值
    ifs_PID.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/PosPIDSaved.txt", std::ios::in);//读PID数
    if (!ifs_PID.is_open()) {
        std::cout << "文件打开失败" << std::endl;
        RxbHardwareInterface::can_close();
        ros::shutdown();
    }
    while (ifs_PID >> idata) {//从文件中读取关节角
        para_kp[i] = idata;
        ifs_PID >> idata;
        para_ki[i] = idata;
        ifs_PID >> idata;
        para_kd[i++] = idata;
    }
    ifs_PID.close();
    std::cout << "Total i:" << i << std::endl;

    //修改UI上的值
    ui->Pos_P1->setValue(para_kp[0]);
    ui->Pos_I1->setValue(para_ki[0]);
    ui->Pos_D1->setValue(para_kd[0]);
}


void rviz_control_plugin::control_panelWidget::on_All_PosPID_para_writeIn_clicked()
{
    std_msgs::Float64MultiArray datas;
    datas.data.resize(ModuJnt_num_all*3);
    datas.data[0] = ui->Pos_P1->value();
    datas.data[1] = ui->Pos_I1->value();
    datas.data[2] = ui->Pos_D1->value();

//    datas.data[3] = ui->Pos_P2->value();
//    datas.data[4] = ui->Pos_I2->value();
//    datas.data[5] = ui->Pos_D2->value();

//    datas.data[6] = ui->Pos_P3->value();
//    datas.data[7] = ui->Pos_I3->value();
//    datas.data[8] = ui->Pos_D3->value();

    J1posPIDWriteInPublisher_.publish(datas);
}

void rviz_control_plugin::control_panelWidget::on_CablePull_Button_014_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 1;
    motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_CablePull_Button_014_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 1;
    motorMovePublisher_.publish(rxb_move);
}


void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_15_pressed()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 1;
    motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_15_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 1;
    motorMovePublisher_.publish(rxb_move);

}
/*
void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_16_pressed()
{

    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableLoose_All + 7;
    motorMovePublisher_.publish(rxb_move);
}


void rviz_control_plugin::control_panelWidget::on_CableLoose_Button_16_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 7;
    motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_CablePull_Button_17_pressed()
{

    std_msgs::UInt32 rxb_move;
    rxb_move.data = CablePull_All + 7;
    motorMovePublisher_.publish(rxb_move);
}


void rviz_control_plugin::control_panelWidget::on_CablePull_Button_17_released()
{
    std_msgs::UInt32 rxb_move;
    rxb_move.data = CableStop_All + 7;
    motorMovePublisher_.publish(rxb_move);
}
*/



void rviz_control_plugin::control_panelWidget::on_ActualOperation_Button_clicked()
{
    std_msgs::UInt32 flag_msg;
    flag_msg.data = workStatusChange;
    flagPublisher_.publish(flag_msg);

    //ui->RxbMode1_Button->setChecked(1);
}

// void rviz_control_plugin::control_panelWidget::SupervisoryControl_RXBMode_Callback(void){
//     ;
// }

void rviz_control_plugin::control_panelWidget::on_RecordFlag_Button_clicked()
{
    std_msgs::UInt32 flag_msg;
    flag_msg.data = 1;
    recordFlagPublisher_.publish(flag_msg);
}

void rviz_control_plugin::control_panelWidget::on_radioButton_clicked()
{
    std_msgs::UInt32 msg;
    msg.data = 1;
    randomMotionPublisher_.publish(msg);
}

void rviz_control_plugin::control_panelWidget::on_ExForceSense_Zero_clicked()
{
    std_msgs::UInt32 flag;
    flag.data = 1;
    exForceSenseZeroPub.publish(flag);
}

void rviz_control_plugin::control_panelWidget::on_radioButton_2_clicked()
{
    std_msgs::UInt32 flag;
    flag.data = 1;
    admittanceControlSwitchPub.publish(flag);
}

void rviz_control_plugin::control_panelWidget::on_revoluteType_pitch_clicked()
{
    nnc.revoluteType = 0;
}

void rviz_control_plugin::control_panelWidget::on_revoluteType_yaw_clicked()
{
    nnc.revoluteType = 1;
}

void rviz_control_plugin::control_panelWidget::on_revoluteSign_positive_clicked()
{
    nnc.revoluteSign = 1;
}

void rviz_control_plugin::control_panelWidget::on_revoluteSign_negative_clicked()
{
    nnc.revoluteSign = 2;
}

void rviz_control_plugin::control_panelWidget::on_segmentID_valueChanged(int arg1)
{
    nnc.segmentID = static_cast<unsigned char>(arg1);
}

void rviz_control_plugin::control_panelWidget::on_jointID_valueChanged(int arg1)
{
    nnc.jointID = static_cast<unsigned char>(arg1);
}

void rviz_control_plugin::control_panelWidget::on_revoluteSign_0_clicked()
{
    nnc.revoluteSign = 0;
}


void rviz_control_plugin::control_panelWidget::on_newRecordBtn_clicked()
{
  std_msgs::UInt8 data;
  data.data = 1;
  nnTrainingNewRcPub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_startBtn_clicked()
{
    nnTrainingStartPub.publish(nnc);
}

void rviz_control_plugin::control_panelWidget::on_segment1_zero_clicked()
{
  std_msgs::UInt32 flag;
  flag.data = 1;
  ZeroFlagPublisher_.publish(flag);
}

void rviz_control_plugin::control_panelWidget::on_segment2_zero_clicked()
{
  std_msgs::UInt32 flag;
  flag.data = 2;
  ZeroFlagPublisher_.publish(flag);
}

void rviz_control_plugin::control_panelWidget::on_segment3_zero_clicked()
{
  std_msgs::UInt32 flag;
  flag.data = 3;
  ZeroFlagPublisher_.publish(flag);
}

void rviz_control_plugin::control_panelWidget::on_EmmergencyButton_stateChanged(int arg1)
{
  std_msgs::UInt32 flag_msg;
  flag_msg.data = workStatusChange;
  flagPublisher_.publish(flag_msg);
}

void rviz_control_plugin::control_panelWidget::on_BasicMode_Openloop_clicked()
{
  rxb_msgs::RxbMode data;
  data.request.mode = RxbMode_OLoop_Pos;
  if (RxbModeClient_.call(data)) {
      rxb_mode = RxbMode_OLoop_Pos;
      motor_mode = MotorPositionMode;
      displayOutputInfos("black", "set rxb_openloop_mode successfully");
  }
}

void rviz_control_plugin::control_panelWidget::on_BasicMode_Closeloop_clicked()
{
  rxb_msgs::RxbMode data;
  data.request.mode = RxbMode_CLoop_Pos;
  if (RxbModeClient_.call(data)) {
      rxb_mode = RxbMode_CLoop_Pos;
      motor_mode = MotorVelocityMode;
      displayOutputInfos("black", "set RxbMode_CLoop_Pos successfully");
  }
}

void rviz_control_plugin::control_panelWidget::on_BasicMode_ForPosHybrid_clicked()
{
  rxb_msgs::RxbMode data;
  data.request.mode = RxbMode_ForPos_Hybrid;
  if (RxbModeClient_.call(data)) {
      rxb_mode = RxbMode_ForPos_Hybrid;
      motor_mode = MotorVelocityMode;
      displayOutputInfos("black", "set rxb_ForPos_Hybrid_mode successfully");
  }
}

void rviz_control_plugin::control_panelWidget::on_ExtraMode_RecordData_stateChanged(int arg1)
{
  std_msgs::UInt32 flag_msg;
  flag_msg.data = 1;
  recordFlagPublisher_.publish(flag_msg);
}

void rviz_control_plugin::control_panelWidget::on_ExtraMode_RecordData_2_stateChanged(int arg1)
{
  std_msgs::UInt32 msg;
  msg.data = 1;
  randomMotionPublisher_.publish(msg);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose1_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 1;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose2_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 2;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose3_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 3;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose1_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 1;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose2_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 2;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose3_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 3;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull1_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 1;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull2_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 2;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull3_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 3;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull1_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 1;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull2_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 2;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull3_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 3;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Enable_stateChanged(int arg1)
{
  rxb_msgs::RxbMode data;
  if(arg1 == Qt::Checked){
    data.request.mode = RxbMode_Adjust_Cable;
    if (RxbModeClient_.call(data)) {
        rxb_mode = RxbMode_Adjust_Cable;
        displayOutputInfos("black", "set RxbMode_Adjust_Cable successfully");
        ui->BasicMode_Openloop->setDisabled(true);
        ui->BasicMode_Closeloop->setDisabled(true);
        ui->BasicMode_ForPosHybrid->setDisabled(true);
    }
  }else if(arg1 == Qt::Unchecked){
    data.request.mode = RxbMode_OLoop_Pos;
    if (RxbModeClient_.call(data)) {
        rxb_mode = RxbMode_OLoop_Pos;
        displayOutputInfos("black", "set RxbMode_OLoop_Pos successfully");
        ui->BasicMode_Openloop->setEnabled(true);
        ui->BasicMode_Closeloop->setEnabled(true);
        ui->BasicMode_ForPosHybrid->setEnabled(true);
        ui->BasicMode_Openloop->setChecked(true);
    }
  }
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose4_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 4;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose4_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 4;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull4_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 4;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull4_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 4;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose5_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 5;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose5_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 5;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull5_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 5;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull5_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 5;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose6_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 6;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose6_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 6;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull6_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 6;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull6_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 6;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose7_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 7;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose7_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 7;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull7_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 7;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull7_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 7;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose8_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 8;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose8_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 8;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull8_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 8;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull8_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 8;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose9_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableLoose_All + 9;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Loose9_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 9;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull9_pressed()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CablePull_All + 9;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_AdjustCable_Pull9_released()
{
  std_msgs::UInt32 rxb_move;
  rxb_move.data = CableStop_All + 9;
  motorMovePublisher_.publish(rxb_move);
}

void rviz_control_plugin::control_panelWidget::on_J3_PosePID_set_clicked()
{
  std_msgs::Float64MultiArray datas;
  datas.data.resize(5);
  datas.data[0] = ui->Pos_P2->value();
  datas.data[1] = ui->Pos_I2->value();
  datas.data[2] = ui->Pos_D2->value();
  datas.data[3] = 0;//0代表位置pid
  datas.data[4] = 3;//1代表第一个可控关节
  //std::cout << "  " << datas.data[0] << "    " << datas.data[1] << "   "  <<  datas.data[2] << "   " <<  datas.data[3] << std::endl;
  J1posPIDPublisher_.publish(datas);
}

void rviz_control_plugin::control_panelWidget::on_ExtraMode_RandomMove_stateChanged(int arg1)
{
  std_msgs::UInt32 msg;
  msg.data = 1;
  randomMotionPublisher_.publish(msg);

  if(arg1 == Qt::Checked){
    ui->BasicMode_Openloop->setDisabled(true);
    ui->BasicMode_Closeloop->setDisabled(true);
    ui->BasicMode_ForPosHybrid->setDisabled(true);
  }else{
    ui->BasicMode_Openloop->setEnabled(true);
    ui->BasicMode_Closeloop->setEnabled(true);
    ui->BasicMode_ForPosHybrid->setEnabled(true);
  }
}

void rviz_control_plugin::control_panelWidget::on_ExtraMode_DragTeaching_stateChanged(int arg1)
{
  std_msgs::UInt32 flag;
  flag.data = 1;
  admittanceControlSwitchPub.publish(flag);

  if(arg1 == Qt::Checked){
    ui->BasicMode_Openloop->setDisabled(true);
    ui->BasicMode_Closeloop->setDisabled(true);
    ui->BasicMode_ForPosHybrid->setDisabled(true);
  }else{
    ui->BasicMode_Openloop->setEnabled(true);
    ui->BasicMode_Closeloop->setEnabled(true);
    ui->BasicMode_ForPosHybrid->setEnabled(true);
  }
}

void rviz_control_plugin::control_panelWidget::on_TaskSpaceControl_EnableBtn_stateChanged(int arg1)
{
    rxb_msgs::RxbMode data;
    if(arg1 == Qt::Checked){
      data.request.mode = RxbMode_TaskSpace_Ctrl;
      if (RxbModeClient_.call(data)) {
          rxb_mode = RxbMode_TaskSpace_Ctrl;
          displayOutputInfos("black", "set RxbMode_TaskSpace_Ctrl successfully");
          ui->BasicMode_Openloop->setDisabled(true);
          ui->BasicMode_Closeloop->setDisabled(true);
          ui->BasicMode_ForPosHybrid->setDisabled(true);
      }
    }else if(arg1 == Qt::Unchecked){
      data.request.mode = RxbMode_OLoop_Pos;
//      data.request.mode = RxbMode_ForPos_Hybrid;
      if (RxbModeClient_.call(data)) {
          rxb_mode = RxbMode_OLoop_Pos;
//          rxb_mode = RxbMode_ForPos_Hybrid;

          displayOutputInfos("black", "set RxbMode_OLoop_Pos successfully");
          ui->BasicMode_Openloop->setEnabled(true);
          ui->BasicMode_Closeloop->setEnabled(true);
          ui->BasicMode_ForPosHybrid->setEnabled(true);
          ui->BasicMode_Openloop->setChecked(true);
      }
    }
}

void rviz_control_plugin::control_panelWidget::on_Calculation_FK_clicked()
{
    // 创建消息对象
    std_msgs::Float64MultiArray msg_FK_cal_q_tgt;

    // 将数据复制到msg向量中
    msg_FK_cal_q_tgt.data.clear();
    msg_FK_cal_q_tgt.data.push_back(ui->TargetJoint_P1->value());
    msg_FK_cal_q_tgt.data.push_back(ui->TargetJoint_Y1->value());

    msg_FK_cal_q_tgt.data.push_back(ui->TargetJoint_P2->value());
    msg_FK_cal_q_tgt.data.push_back(ui->TargetJoint_Y2->value());

    msg_FK_cal_q_tgt.data.push_back(ui->TargetJoint_P3->value());
    msg_FK_cal_q_tgt.data.push_back(ui->TargetJoint_Y3->value());

    // 发布消息
    taskSpaceCtrl_FKCal_pub.publish(msg_FK_cal_q_tgt);
}

void rviz_control_plugin::control_panelWidget::on_TaskSpaceControl_SingleSegBtn_clicked()
{
    std_msgs::UInt8 data;
    data.data = 1;
    taskSpaceCtrl_mode_pub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_TaskSpaceControl_MultiSegBtn_clicked()
{
    std_msgs::UInt8 data;
    data.data = 2;
    taskSpaceCtrl_mode_pub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_TSCtrl_Move_clicked()
{
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ui->TargetJoint_P1->value());
    msg.data.push_back(ui->TargetJoint_Y1->value());
    msg.data.push_back(ui->TargetJoint_P2->value());
    msg.data.push_back(ui->TargetJoint_Y2->value());
    msg.data.push_back(ui->TargetJoint_P3->value());
    msg.data.push_back(ui->TargetJoint_Y3->value());
    taskSpaceCtrl_move_pub.publish(msg);
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_Start_clicked()
{
    std_msgs::Bool msg;
    msg.data = true;

    neuralNetwork_corCableForceStart_pub.publish(msg);
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_Stop_clicked()
{
    std_msgs::Bool msg;
    msg.data = true;

    neuralNetwork_corCableForceStop_pub.publish(msg);
}

void rviz_control_plugin::control_panelWidget::on_AdmittanceControl_Enable_stateChanged(int arg1)
{
    std_msgs::UInt32 flag;
    flag.data = 1;
    admittanceControlSwitchPub.publish(flag);

    if(arg1 == Qt::Checked){
        ui->BasicMode_Openloop->setDisabled(true);
        ui->BasicMode_Closeloop->setDisabled(true);
        ui->BasicMode_ForPosHybrid->setDisabled(true);
    }else{
        ui->BasicMode_Openloop->setEnabled(true);
        ui->BasicMode_Closeloop->setEnabled(true);
        ui->BasicMode_ForPosHybrid->setEnabled(true);
    }
}

void rviz_control_plugin::control_panelWidget::on_AdmittanceControl_Seg1Set_clicked()
{
    std_msgs::Float32MultiArray data;
    data.data.push_back(static_cast<float>(ui->AdmittanceControl_Seg1M->value()));
    data.data.push_back(static_cast<float>(ui->AdmittanceControl_Seg1B->value()));
    data.data.push_back(static_cast<float>(ui->AdmittanceControl_Seg1K->value()));
    admittanceControl_seg1Set_pub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_DataRecord_Start_clicked()
{
    std_msgs::UInt8 data;
    data.data = neuralNetwork_corCableForce_dataRecord_state;
    neuralNetwork_corCableForce_dataRecord_start_pub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_DataRecord_TrainSet_clicked()
{
    neuralNetwork_corCableForce_dataRecord_state = 1;
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_DataRecord_DevSet_clicked()
{
    neuralNetwork_corCableForce_dataRecord_state = 2;
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_DataRecord_TestSet_clicked()
{
    neuralNetwork_corCableForce_dataRecord_state = 3;
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_DataRecord_Stop_clicked()
{
    std_msgs::UInt8 data;
    data.data = 0;
    neuralNetwork_corCableForce_dataRecord_stop_pub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_Model_Train_clicked()
{
    std_msgs::Bool data;
    data.data = true;
    neuralNetwork_corCableForce_model_train_pub.publish(data);
}

void rviz_control_plugin::control_panelWidget::on_CorCableForce_Model_Dev_State_clicked(bool checked)
{
    std_msgs::Bool data;
    data.data = checked;
    neuralNetwork_corCableForce_model_dev_state_pub.publish(data);
}
