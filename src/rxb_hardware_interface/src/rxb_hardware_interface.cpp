#include "rxb_hardware_interface.h"
#include "motor_driver.h"
#include "usbcan.h"
#include <std_msgs/Float64MultiArray.h>
#include "global.h"
#include "math.h"
#include "rxb_msgs/serialCommu.h"



namespace RxbHardwareInterface {

    RxbHardwareInterface::RxbHardwareInterface(ros::NodeHandle n): nh_(n), spinner(20),
        as_(nh_, "rxb/rxb_joint_controller/follow_joint_trajectory", boost::bind(&RxbHardwareInterface::executeTrajectory,  this, _1), false),
        jointAngleSmooth("jointAngleSmooth", Gene_Jnt_Num),
        jointAngleKalman("jointAngleKalman", Gene_Jnt_Num, {1, 2, 4}),
        neuralNetwork_corCableForce_dataRecord_sw(2, 20)
    {
        as_.start();
        flagSubscriber_ = nh_.subscribe<std_msgs::UInt32>("control_plugin/flag", 1, &RxbHardwareInterface::rxbCommandFlagCallback, this);//标志rxb的状态，开启、关闭、实际操控（机械臂是否运动）
        recordFlagSubscriber_ = nh_.subscribe<std_msgs::UInt32>("control_plugin/record_flag", 1, &RxbHardwareInterface::recordFlagCallback, this);//
    }


    void RxbHardwareInterface::executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
//        std_msgs::Float64MultiArray jointsRXB[16];
//        jointState js;
//        jointState joints1;
//        jointState jointsn;
//        double lastDuration = 0.0;
//        int nrofPoints;
//        float dtmp;
//        int nrOfJoints = goal->trajectory.points.size();
//        cout << nrOfJoints << endl;
//        RXB_joint_plan_tmp.clear();
//        if(g_motorMode == MotorVelocityMode) {
//            g_motorMode = MotorPositionMode;
//            Motor_Driver_Reset(1, 0);
//            Motor_Driver_Reset(0, 0);
//            usleep(200000);
//            Motor_Driver_Mode_Choice(0, 0, Velocity_Position_Mode);
//            Motor_Driver_Mode_Choice(1, 0, Velocity_Position_Mode);
//        }
//        if(g_rxbMode = RxbMode_OLoop_Pos) {
//            RXB_Cable_Plan.clear();
//            RXB_joint_plan.clear();
//            //       cout << "output joint:" << endl;
//            for(int i = 0; i < nrOfJoints; i++) {
//                for(int k = 0; k < Controllable_DOF_num_all; k++) {
//                    js.j[k] = goal->trajectory.points[i].positions[k] *  Rad2Deg;
//                }
//                dtmp = goal->trajectory.points[i].time_from_start.toSec();
//                js.t = dtmp * timeCoe;
//                js.duration = dtmp - lastDuration;
//                lastDuration = dtmp;
//                RXB_joint_plan_tmp.push_back(js); //记得这里要改变
//                if(i == 0) {
//                    joints1 = js;
//                }
//                if(i == nrOfJoints - 1) {
//                    jointsn = js;
//                }
////                for(int k = 0; k < Controllable_DOF_num_all; k++) {
////                    cout << js.j[k] << endl;
////                }
//            }
//            //      cout << " " << endl;
//            jointPlan2CablePlan(RXB_joint_plan_tmp, RXB_Cable_Plan_Tmp);
//            cable_Interpolation(RXB_Cable_Plan_Tmp, RXB_Cable_Plan, nrofPoints);
//            RXB_Cable_Plan_Tmp.clear();
//            joints_SimpleInterpolation(nrofPoints, joints1, jointsn, RXB_joint_plan);

//        }
//        //接下来把tmp里的jointstate样条插值了，再push_back进入RXB_Motor_plan里，再清零。
//        as_.setSucceeded();
//        ROS_INFO("get the calculated cableLen Data!!");
    }

    void RxbHardwareInterface::openLoopJntCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &goal) {//点到点规划
        std::cout << "Start planning!" << std::endl;
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            oloop_targetJoint.j[i] = goal->data[i] * Rad2Deg;
        }
        cout << "Enter openLoopJntCmdCallback" << endl;
        RXB_Cable_Plan.clear();
        RXB_joint_plan.clear();
        if(g_motorMode == MotorVelocityMode) {
            g_motorMode = MotorPositionMode;
            Motor_Driver_Reset(0, 0);//复位驱动器
            usleep(500000);
            Motor_Driver_Mode_Choice(0, 0, Velocity_Position_Mode);//将驱动器运动模式设置为速度位置模式
            usleep(500000);
        }
        rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;//从当前角度开始规划
        p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, oloop_targetJoint, RXB_joint_plan);
        jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
    }

    bool RxbHardwareInterface::targetPoseCmdCallback(rxb_msgs::RxbPoseCmd::Request &req, rxb_msgs::RxbPoseCmd::Response &res) {
        Eigen::Matrix4d T_end;
        T_end = Quatern2RotMat(req.pose);
        double theta[Controllable_DOF_num_all];
        int flag;
        cout << "Enter targetPoseCmdCallback" << endl;
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            theta[i] = rxb_currentJointState_Controllable_DOF.j[i];
        }
        flag = inverse_kinematics(T_end, theta, angle_limit);//deg
        res.success = flag;
        if(flag == 1) { //inverse_kinematics calculate successfully
            if(g_rxbMode == RxbMode_OLoop_Pos) {
                for(int i = 0; i < Controllable_DOF_num_all; i++) {
                    oloop_targetJoint.j[i] = theta[i];
                }
                RXB_Cable_Plan.clear();
                RXB_joint_plan.clear();
                if(g_motorMode == MotorVelocityMode) {
                    g_motorMode = MotorPositionMode;
                    Motor_Driver_Reset(1, 0);
                    Motor_Driver_Reset(0, 0);
                    usleep(500000);
                    Motor_Driver_Mode_Choice(0, 0, Velocity_Position_Mode);
                    Motor_Driver_Mode_Choice(1, 0, Velocity_Position_Mode);
                    usleep(500000);
                }
                rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;
                p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, oloop_targetJoint, RXB_joint_plan);
                jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
            }
        }
        return true;

    }

    void RxbHardwareInterface::pauseFlagCallback(const std_msgs::UInt32::ConstPtr& data){
      if(data->data==1){
          pause_flag = pause_flag==true?false:true; //pause_flag = !pause_flag; 111
          if(pause_flag){
              if(g_rxbMode == RxbMode_ForPos_Hybrid)
              {
                  Motor_Driver_Velocity_Mode(0, 0, 2500,  0);
                  Motor_Driver_Velocity_Mode(0, 1, 2500,  0);
              }

          }
      }
    }


    void RxbHardwareInterface::init() {
        can_init(0x00, 0x14);//启动线程，如果接收缓冲区内有数据则接收
        Motor_Driver_Reset(0, 0);//对第0组的所有驱动器初始化
        usleep(500000);
        Motor_Driver_Mode_Choice(0, 0, Velocity_Position_Mode);//驱动器速度位置模式选择
        usleep(500000);
        Motor_Driver_Config(0, 0, 5, 0);//命令0组所有驱动器向上位机传输电机相关信息
        usleep(1000);
        g_motorMode = MotorPositionMode;//驱动器速度位置模式对应于电机位置模式

        pause_flag = false;//暂停标志，为真时使定时器回调函数timerCallback中的函数体不进行

        timer = nh_.createTimer(ros::Duration(global_T_Interval_s), &RxbHardwareInterface::timerCallback, this, false);//20ms，获得力传感器数据，在上位机界面更新力传感器和角度编码器数据，硬件驱动
        timer2 = nh_.createTimer(ros::Duration(global_EncoderDataGet_s), &RxbHardwareInterface::Sensordata_timerCallback, this, false);//3ms，要求角度编码器传回数据，每次要求1个采集卡传回，2个采集卡就是6ms的周期

        /// 旧功能（已删减）
        PlaningJntPos_Pub = nh_.advertise<std_msgs::Float64MultiArray>("rxbHW/PlaningJntPos", 1);//返回面板关节规划值
        RxbModeServer_ = nh_.advertiseService("control_plugin/RxbMode", &RxbHardwareInterface::rxbModeFun, this);//更改rxb运行模式,开环，闭环，调绳
        TrajectorySubscriber_ = nh_.subscribe<std_msgs::UInt32>("control_plugin/trajectory_demo", 1, &RxbHardwareInterface::rxbTrajectoryFun, this);//1-8txt生成轨迹
        motorMoveSubscriber_ = nh_.subscribe<std_msgs::UInt32>("control_plugin/motor_move", 1, &RxbHardwareInterface::motorMoveCallback, this);//直接控制电机运动 松、拉、停、复位
        // 三种关节控制
        cloop_targetJoint_Sub = nh_.subscribe<std_msgs::Float64MultiArray>("control_plugin/closeLoop_jointPosCmd", 1, &RxbHardwareInterface::closeLoopJntCmdCallback, this);
        ForPos_Hybrid_targetJoint_Sub = nh_.subscribe<std_msgs::Float64MultiArray>("control_plugin/ForPos_Hybrid_jointPosCmd", 1, &RxbHardwareInterface::ForPos_HybridJntCmdCallback, this);
        oloop_targetJoint_Sub = nh_.subscribe("control_plugin/openLoop_jointPosCmd", 1, &RxbHardwareInterface::openLoopJntCmdCallback, this);
        pauseSub_ = nh_.subscribe<std_msgs::UInt32>("control_plugin/pause_flag", 1, &RxbHardwareInterface::pauseFlagCallback, this);  //订阅者 暂停按钮
        serial_pub = nh_.advertise<rxb_msgs::serialCommu>("serialport/commu", 1);//通知串口发数据(力传感器)数据过来
        serial_sub = nh_.subscribe<rxb_msgs::sensorRead>("serialport/sensor_data", 1, &RxbHardwareInterface::serial_data_subFun, this);//接收串口发送过来的数据（力传感器数据）
        serial_encoder_pub = nh_.advertise<rxb_msgs::sensorRead>("serialport/Sensor_Encoder_data", 1);//将编码器数据传递给上位机界面
        moveit_PubJoints = nh_.advertise<sensor_msgs::JointState>("/rxb/joint_states", 1);//let moveit model move 发布者
        Encoder_zero_sub = nh_.subscribe("control_plugin/joint_zero_signal", 1, &RxbHardwareInterface::Encoder_zero_subFun, this);
        J1PosPID_sub = nh_.subscribe<std_msgs::Float64MultiArray>("control_plugin/J1pospid", 1, &RxbHardwareInterface::J1PosPIDCallback, this);
        J1PosPIDWriteIn_sub = nh_.subscribe<std_msgs::Float64MultiArray>("control_plugin/J1pospidWriteIn", 1, &RxbHardwareInterface::J1PosPIDWriteInCallback, this);
        ForcePIDWriteIn_sub = nh_.subscribe<std_msgs::Float64MultiArray>("control_plugin/forcepidWriteIn", 1, &RxbHardwareInterface::ForcePIDWriteInCallback, this);

        //机械臂随机运动，用于获得神经网络训练数据
        randomMotion_sub = nh_.subscribe<std_msgs::UInt32>("control_plugin/randomMotion", 1, &RxbHardwareInterface::randomMotionCallback, this);

        //用于力感知实验
        exForceSensor_sub = nh_.subscribe<geometry_msgs::Twist>("forceSensor/force", 1, &RxbHardwareInterface::exForceSensor_callbackFunc, this);
        forceSenseSub = nh_.subscribe<rxb_msgs::forceSenseResult>("force_sense/result", 1, &RxbHardwareInterface::forceSense_callbackFunc, this);

        //用于导纳控制
        admittanceControlSwitch_sub = nh_.subscribe<std_msgs::UInt32>("admittance_control/switch", 1, &RxbHardwareInterface::AdmittanceControlSwitch_CallbackFunc, this);
        admittanceControl_seg1Set_sub = nh_.subscribe<std_msgs::Float32MultiArray>("admittanceControl/seg1Set", 1, &RxbHardwareInterface::AdmittanceControl_Seg1Set_SubFunc, this);

        // 用于神经网络训练
        nnTrainingNewRcSub = nh_.subscribe<std_msgs::UInt8>("nnTraining/newRc", 1, &RxbHardwareInterface::NNTrainingNewRc_CallbackFunc, this);
        nnTrainingStartSub = nh_.subscribe<rxb_msgs::neuralNetworkCommand>("nnTraining/start", 1, &RxbHardwareInterface::NNTrainingStart_CallbackFunc, this);
        nnTrainingProgressPub = nh_.advertise<std_msgs::UInt8>("nnTraining/progress", 1);

        // 用于初始化机械臂是同步ui界面参数
        uiUpdatePosePID_pub = nh_.advertise<std_msgs::Float64MultiArray>("uiUpdate/posePID", 1);

        // 用于任务空间控制
        taskSpaceCtrl_mode_sub = nh_.subscribe<std_msgs::UInt8>("taskSpaceCtrl/mode", 1, &RxbHardwareInterface::TaskSpaceCtrl_Mode_SubFunc, this);
        taskSpaceCtrl_move_sub = nh_.subscribe<std_msgs::Float64MultiArray>("taskSpaceCtrl/move", 1, &RxbHardwareInterface::TaskSpaceCtrl_Move_SubFunc, this);
        taskSpaceCtrl_singleSeg_client = nh_.serviceClient<rxb_msgs::taskSpaceCtrl_singleSeg>("taskSpaceCtrl/singleSeg");
        desiredJoint = vector<double>(Controllable_DOF_num_all, 0);

        // 补充基础功能
        actualT0 = (Eigen::MatrixXd(4, 4) <<  1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 1, baseLink_len+Gene_Jnt_Num/2*link_len,
                                              0, 0, 0, 1).finished();
        actualT = actualT0;
        BList = vector<Eigen::VectorXd>(Gene_Jnt_Num, Eigen::VectorXd(6, 1));
        for(unsigned long m = 0; m < 2*g_RXBSegment_num; m++){
            BList[4*m+1 - 1] << 1, 0, 0, 0, (2*static_cast<long>(m) - 4*g_RXBSegment_num)*link_len, 0;
            BList[4*m+2 - 1] << 0, 1, 0, (4*g_RXBSegment_num - 2*static_cast<long>(m))*link_len, 0, 0;
            BList[4*m+3 - 1] << 0, 1, 0, ((4*g_RXBSegment_num-1) - 2*static_cast<long>(m))*link_len, 0, 0;
            BList[4*m+4 - 1] << 1, 0, 0, 0, (2*static_cast<long>(m) - (4*g_RXBSegment_num-1))*link_len, 0;
        }
        actualT_pub = nh_.advertise<std_msgs::Float32MultiArray>("monitorWindow/actualT", 1);
        bodyJacobian = MatrixXd(6, Gene_Jnt_Num);
        bodyJacobian.setZero();
        bodyTwist_pub = nh_.advertise<std_msgs::Float32MultiArray>("monitorWindow/bodyTwist", 1);
        trajectoryDemo_sub = nh_.subscribe("trajectoryDemo", 1, &RxbHardwareInterface::TrajectoryDemo_SubFunc, this);

        // 在线神经网络训练
        neuralNetwork_corCableForceStart_sub = nh_.subscribe<std_msgs::Bool>("neuralNetwork/corCableForce/start", 1, &RxbHardwareInterface::NeuralNetwork_CorCableForceStart_SubFunc, this);
        neuralNetwork_corCableForceStop_sub = nh_.subscribe<std_msgs::Bool>("neuralNetwork/corCableForce/stop", 1, &RxbHardwareInterface::NeuralNetwork_CorCableForceStop_SubFunc, this);
        neuralNetwork_corCableForceTrain_pub = nh_.advertise<std_msgs::Float32MultiArray>("neuralNetwork/corCableForce/train", 1);
        neuralNetwork_corCableForce_dataRecord_start_sub = nh_.subscribe("neuralNetwork/corCableForce/dataRecord/start", 1, &RxbHardwareInterface::NeuralNetwork_CorCableForce_DataRecord_Start_SubFunc, this);
        neuralNetwork_corCableForce_dataRecord_stop_sub = nh_.subscribe("neuralNetwork/corCableForce/dataRecord/stop", 1, &RxbHardwareInterface::NeuralNetwork_CorCableForce_DataRecord_Stop_SubFunc, this);
        neuralNetwork_corCableForce_dataRecord_count_pub = nh_.advertise<std_msgs::UInt32>("neuralNetwork/corCableForce/dataRecord/count", 1);
        neuralNetwork_corCableForce_model_dev_state_sub = nh_.subscribe("neuralNetwork/corCableForce/model/dev/state", 1, &RxbHardwareInterface::NeuralNetwork_CorCableForce_Model_Dev_State_SubFunc, this);
        neuralNetwork_corCableForce_model_client = nh_.serviceClient<rxb_msgs::corCableForceNN>("neuralNetwork/corCableForce/model/call");
        neuralNetwork_corCableForce_model_dev_data_pub = nh_.advertise<std_msgs::Float32MultiArray>("neuralNetwork/corCableForce/model/dev/data", 1);

        //力传感器获得初值
        getSensorData();

        //--------------规划数据--------------
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            rxb_setJointState.j[i] = 0;
        }
        rxb_setJointState.t = 0;
        rxb_setJointState.duration = 0;
        for(int i = 0; i < Motor_num_all; i++) {
            rxb_setCableState.len[i] = 0;
        }
        rxb_setCableState.t = 0;
        rxb_setCableState.duration = 0;
        //--------------规划数据--------------


        //PID参数加载
        double idata;
        int i = 0;
        unsigned long j = 0;
        std_msgs::Float64MultiArray posePID;  //更新UI界面的PID
        posePID.data.resize(3*ModuJnt_num_all);
        ifs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/PosPIDSaved.txt", ios::in);//读PID数据
        if (!ifs.is_open()) {
            cout << "文件打开失败" << endl;
            can_close();
            ros::shutdown();
        }
        while (ifs >> idata) {
            Joint_pid_kp_FirstGroup[i] = idata;
            posePID.data[j++] = idata;
            ifs >> idata;
            Joint_pid_ki_FirstGroup[i] = idata;
            posePID.data[j++] = idata;
            ifs >> idata;
            Joint_pid_kd_FirstGroup[i++] = idata;
            posePID.data[j++] = idata;
        }
        ifs.close();
        uiUpdatePosePID_pub.publish(posePID);

       //断电前规划数据加载，加载到变量 rxb_currentJointState_Controllable_DOF
        ifs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/joint_saved.txt", ios::in);
        if (!ifs.is_open()) {
            cout << "文件打开失败" << endl;
            can_close();
            ros::shutdown();
        }
        int t = 0;
        while (ifs >> idata) {//从文件中读取关节角
            rxb_currentJointState_Controllable_DOF.j[t++] = idata;
            cout << "（Before Power outage）rxb_currentJointState_Controllable_DOF [" << t << "] : " << rxb_currentJointState_Controllable_DOF.j[t-1] << endl;
        }
        rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;
        ifs.close();

        for(int i = 0; i < Controllable_DOF_num_all; i++){
          rxb_setJointState.j[i] = rxb_currentJointState_Controllable_DOF.j[i]; // 避免闭环模式默认回到零点，导致电机转速过快
        }

        //绳索初始数据加载
        RXB_JntAngle_To_Cable_Length_new(rxb_currentJointState_Controllable_DOF.j, g_offset_CablePos);
        for(int i = 0; i < Motor_num_all; i++) {
            rxb_priorCableState.len[i] = g_offset_CablePos[i];
            cout << "g_offset_CablePos [" << i << "] : " << g_offset_CablePos[i] << endl;
        }
        rxb_priorCableState.t = 0;
        rxb_priorCableState.duration = 0;

        //断电前角度编码器零位读取
        ifs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/jointAngle_Zero.txt", ios::in);
        if (!ifs.is_open()) {
            cout << "文件打开失败" << endl;
            can_close();
            ros::shutdown();
        }
        j = 0;
        while (ifs >> idata) {
            g_JointAngle_Zero[j++] = idata;
            cout << "（Before Power outage）g_JointAngle_Zero [" << j << "] : " << g_JointAngle_Zero[j-1] << endl;
        }
        ifs.close();

        //初始化向moveit发送的数据
        jointsCurrent.name.resize(Gene_Jnt_Num);
        jointsCurrent.position.resize(Gene_Jnt_Num);
        for(int i = 0; i < Gene_Jnt_Num; i++) {
            jointsCurrent.name[i] = "Joint_" + to_string(i + 1);
        }

        // 等待ui界面，更新ui界面
        ros::Rate rate(1);
        while(ros::ok() && uiUpdatePosePID_pub.getNumSubscribers()==0){
          cout << "Wait for UI!" << endl;
          rate.sleep();
        }
        uiUpdatePosePID_pub.publish(posePID);

        spinner.start();
    }

    //rxb flag: on  off  ....
    void RxbHardwareInterface::rxbCommandFlagCallback(const std_msgs::UInt32::ConstPtr &flag) {
        uint data = flag->data;
        if(data == isRxbOnFlag) {
            g_rxbCommandFlag = isRxbOnFlag;
        } else if(data == isRxbOffFlag) {
            g_rxbCommandFlag = isRxbOffFlag;
        }
        else if (data == isWorking) {
            g_rxbWorkStatus = 1;
            cout << "g_rxbWorkStatus:" << g_rxbWorkStatus << endl;
        }
        else if (data == isNWorking) {
            g_rxbWorkStatus = 0;
            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();
            cout << "g_rxbWorkStatus:" << g_rxbWorkStatus << endl;
        }
        else if (data == workStatusChange){
            g_rxbWorkStatus = (g_rxbWorkStatus==1) ? 0 : 1;
            cout << "g_rxbWorkStatus:" << g_rxbWorkStatus << endl;
            if(g_rxbWorkStatus == 0){
                RXB_Cable_Plan.clear();
                RXB_joint_plan.clear();
            }

        }
    }

    void RxbHardwareInterface::recordFlagCallback(const std_msgs::UInt32::ConstPtr &flag) {
        uint data = flag->data;

        if (data){
            g_recordCommandFlag = (g_recordCommandFlag == isRecordOffFlag) ? isRecordOnFlag : isRecordOffFlag;
        }
    }

    //motor mode control: velocity  or  position
    bool RxbHardwareInterface::motorModeFun(rxb_msgs::MotorMode::Request &req, rxb_msgs::MotorMode::Response &res) {// 暂无接口
        uint data = req.mode;
        res.result = data;
        if(g_isRunningFlag == false) {
            Motor_Driver_Reset(1, 0);
            Motor_Driver_Reset(0, 0);
            usleep(500000);
            if(data == MotorVelocityMode) {
                g_motorMode = MotorVelocityMode;
                Motor_Driver_Mode_Choice(0, 0, Velocity_Mode); //0组的所有驱动器 都进入速度模式
                Motor_Driver_Mode_Choice(1, 0, Velocity_Mode);
                usleep(500000);
                ROS_INFO("Motors are changed to Velocity_Mode");
            } else if(data == MotorPositionMode) {
                g_motorMode = MotorPositionMode;
                Motor_Driver_Mode_Choice(0, 0, Velocity_Position_Mode);
                Motor_Driver_Mode_Choice(1, 0, Velocity_Position_Mode);
                usleep(500000);
                ROS_INFO("Motors are changed to Position_Mode");


                for(int i=0;i<Controllable_DOF_num_all;i++){
                        rxb_currentJointState_Controllable_DOF.j[i] = g_PlanjointAngle_nouse[i];
                }
                rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;

                RXB_JntAngle_To_Cable_Length_new(rxb_currentJointState_Controllable_DOF.j, g_offset_CablePos);//必要的
                for(int i = 0; i < Motor_num_all; i++) {
                    rxb_priorCableState.len[i] = g_offset_CablePos[i];
                }
                rxb_priorCableState.t = 0;
                rxb_priorCableState.duration = 0;
                RXB_Cable_Plan.clear();
                RXB_joint_plan.clear();


            }
        }

        return true;
    }


    void RxbHardwareInterface::motorMoveCallback(const std_msgs::UInt32::ConstPtr &flag) {

        int data = static_cast<int>(flag->data);
        int type = 0;//0 pull  1 loose 3 stop 4 reset
        u8 Group_Num, Driver_Num;
        int coh;
        int vel = 3000;
        int num;

        //确定模式——0拉绳、1松绳、3停止、4重置电机
        if(data-CableLoose_All >= 0 && data-CableLoose_All <= Motor_num_all){
          type = 1;
          num = data - CableLoose_All;
        }else if(data-CablePull_All >= 0 && data-CablePull_All <= Motor_num_all){
          type = 0;
          num = data - CablePull_All;
        }else{
          type = 3;
          num = data - CableStop_All;
        }

        //确定控制的驱动器
        if(num != 0) {
            Group_Num = g_Driver_ID[num-1] >> 4 & 0x0f;
            Driver_Num = g_Driver_ID[num-1] - 15 * Group_Num;
        }else{
          Group_Num = 0;
          Driver_Num = 0;
        }

        if(g_rxbMode == RxbMode_Adjust_Cable) {
            if(type == 1) { //1 loose
                coh = 1;
                Motor_Driver_Velocity_Mode(Group_Num, Driver_Num, 5000, coh * vel);
            } else if(type == 0) { //0 pull
                coh = -1;
                Motor_Driver_Velocity_Mode(Group_Num, Driver_Num, 5000, coh * vel);
            }else if(type == 3) {
                Motor_Driver_Velocity_Mode(Group_Num, Driver_Num, 5000, 0);
            }
        }
    }

    void RxbHardwareInterface::rxbTrajectoryFun(const std_msgs::UInt32::ConstPtr &traj) {

        uint data = traj->data;
        double idata;
        bool enter_time = false, usage_flag = false;
        int count = 0, points_num = 0;
        jointState point;
        list<jointState>::iterator itea;
        ifstream iffs;
        int num = static_cast<int>(data - TrajectoryDemo);// 1 2 3 4
        switch(num) {
            case 1:
                ROS_INFO("Loading the trajectory1..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory1.txt", ios::in);
                break;
            case 2:
                ROS_INFO("Loading the trajectory2..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory2.txt", ios::in);
                break;
            case 3:
                ROS_INFO("Loading the trajectory3..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory3.txt", ios::in);
                break;
            case 4:
                ROS_INFO("Loading the trajectory4..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory4.txt", ios::in);
                break;
            case 5:
                ROS_INFO("Loading the trajectory5..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory5.txt", ios::in);
                break;
            case 6:
                ROS_INFO("Loading the trajectory6..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory6.txt", ios::in);
                break;
            case 7:
                ROS_INFO("Loading the trajectory7..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory7.txt", ios::in);
                break;
            case 8:
                ROS_INFO("Loading the trajectory8..........");
                iffs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory8.txt", ios::in);
                break;
            default:
                break;
        }
        if (!iffs.is_open()) {
            cout << "traj file cannot open" << endl;
            return;
        }

        std::list<jointState> RXB_joint_plan_tmp;
        RXB_joint_plan_tmp.clear();
        while (iffs >> idata) {
            if (idata == 999999999 && enter_time == false) {  //999999999表示后面的数据为时间tick
                cout << "find the 999999999" << endl;
                enter_time = true;
                usage_flag = true;
                count = 0;
                itea = RXB_joint_plan_tmp.begin();  //重新定位到表格开头的地方
                continue;
            }
            if(count >= 0 && count < Controllable_DOF_num_all && enter_time == false) { //记录轨迹点
                point.j[count] = idata;
            }
            if (enter_time == true) { //记录时间tick
                if (count >= points_num) {
                    break;
                }
                itea ->t = idata;
                itea++;
            }
            count++;
            if (count == Controllable_DOF_num_all && enter_time == false) { //每记录一组轨迹点
                count = 0;
                point.t = 0;
                RXB_joint_plan_tmp.push_back(point);
                points_num++; //轨迹点数量
            }
        }
        iffs.close();

        if(usage_flag == false) {
            RXB_joint_plan_tmp.clear();
            ROS_ERROR("no '999999999' in the trajectory file");
            return;
        }else {
            if(RXB_joint_plan_tmp.size() > 0) { //当 RXB_joint_plan_tmp 记录到数据
                /*
                itea = RXB_joint_plan_tmp.begin();
                point = *itea;

                rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;
                p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, point, RXB_joint_plan);

                double t_bg = RXB_joint_plan_tmp.begin()->t;
                for(itea = RXB_joint_plan_tmp.begin(); itea != RXB_joint_plan_tmp.end(); itea++) {
                    itea->t += (point.t - t_bg);
                }

                for(auto it = RXB_joint_plan_tmp.begin(); it != RXB_joint_plan_tmp.end(); it++){
                    for(int a = 0; a < 2; a++){
                        cout << "dian: " << it->j[a] << "   ";
                    }
                    cout << it->t;
                    cout << endl;
                }
                cout << points_num << endl;

                joints_Interpolation(RXB_joint_plan_tmp, RXB_joint_plan, points_num);
                jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
                rxb_setCableState.t = 0;
                */
                itea = RXB_joint_plan_tmp.begin();
                point = *itea;

                rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;
                p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, point, RXB_joint_plan); //会根据最大关节角变化计算目标点时刻，并记录到 point


                double t_bg = RXB_joint_plan_tmp.begin()->t;
                for(itea = RXB_joint_plan_tmp.begin(); itea != RXB_joint_plan_tmp.end(); itea++) {  //需要加上移动到轨迹规划初始点的时间
                    itea->t += (point.t - t_bg);

                }
                joints_Interpolation(RXB_joint_plan_tmp, RXB_joint_plan, points_num);



                jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);

            } else {
                ROS_ERROR("no data available data in the trajectory file");
            }
        }




    }
//choose rxb working mode: open_loop close_loop or force_position_hybrid
    bool RxbHardwareInterface::rxbModeFun(rxb_msgs::RxbMode::Request &req, rxb_msgs::RxbMode::Response &res) {
        if(req.mode == RxbMode_OLoop_Pos) {
            ROS_INFO("rxb openloop mode");
            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();
            if(g_motorMode != MotorPositionMode){
                while(g_isRunningFlag); // 避免出现电机重置，仍有电机控制指令发送
                g_motorMode = MotorPositionMode;
                Motor_Driver_Reset(0, 0);
                usleep(500000);//死延时
                Motor_Driver_Mode_Choice(0, 0, Velocity_Position_Mode);
                usleep(500000);//死延时
                RXB_JntAngle_To_Cable_Length_new(rxb_currentJointState_Controllable_DOF.j, g_offset_CablePos);
            }
            g_rxbMode = RxbMode_OLoop_Pos;
            res.result = static_cast<unsigned short>(g_rxbMode);
        }else if(req.mode == RxbMode_CLoop_Pos){
            ROS_INFO("rxb closeloop mode");
            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();
            for(int i = 0; i < Controllable_DOF_num_all; i++){
              error_joint_FirstGroup[i] = 0;
              derror_joint_FirstGroup[i] = 0;
              ierror_joint_FirstGroup[i] = 0;
            }

            if(g_motorMode != MotorVelocityMode){
                while(g_isRunningFlag);
                g_motorMode = MotorVelocityMode;
                Motor_Driver_Reset(0, 0);
                usleep(500000);
                Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
                usleep(500000);
            }

            jointState currentJointState;
            currentJointState.j[0] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[1] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[2] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[5] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[6]);
            currentJointState.j[1] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[0] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[3] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[4] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[7]);
            currentJointState.t = currentJointState.duration = 0;
            p2p_jointTrajectoryPlan(currentJointState, rxb_currentJointState_Controllable_DOF, RXB_joint_plan);
            jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);

            g_rxbMode = RxbMode_CLoop_Pos;
            res.result = static_cast<unsigned short>(g_rxbMode);
        }else if(req.mode == RxbMode_ForPos_Hybrid) {
            ROS_INFO("rxb Force&Pos mode");
            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();
            for(int i = 0; i < Controllable_DOF_num_all; i++){
              error_joint_FirstGroup[i] = 0;
              derror_joint_FirstGroup[i] = 0;
              ierror_joint_FirstGroup[i] = 0;
            }

            if(g_motorMode != MotorVelocityMode){
                while(g_isRunningFlag);
                g_motorMode = MotorVelocityMode;
                Motor_Driver_Reset(0, 0);
                usleep(500000);
                Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
                usleep(500000);
            }

            jointState currentJointState;
            currentJointState.j[0] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[1] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[2] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[5] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[6]);
            currentJointState.j[1] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[0] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[3] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[4] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[7]);
            currentJointState.t = currentJointState.duration = 0;
            p2p_jointTrajectoryPlan(currentJointState, rxb_currentJointState_Controllable_DOF, RXB_joint_plan);
            jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);

            g_rxbMode = RxbMode_ForPos_Hybrid;
            res.result = static_cast<unsigned short>(g_rxbMode);
        }else if(req.mode == RxbMode_Adjust_Cable) {
            ROS_INFO("rxb AdjustCable mode");
            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();
            if(g_motorMode != MotorVelocityMode){
                while(g_isRunningFlag);
                g_motorMode = MotorVelocityMode;
                Motor_Driver_Reset(0, 0);
                usleep(500000);
                Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
                usleep(500000);
            }
            g_rxbMode = RxbMode_Adjust_Cable;
            res.result = static_cast<unsigned short>(g_rxbMode);
        }else if(req.mode == RxbMode_TaskSpace_Ctrl){
            ROS_INFO("rxb TaskSpaceCtrl mode");
            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();
            for(int i = 0; i < Controllable_DOF_num_all; i++){
              error_joint_FirstGroup[i] = 0;
              derror_joint_FirstGroup[i] = 0;
              ierror_joint_FirstGroup[i] = 0;
            }

            if(g_motorMode != MotorVelocityMode){
                while(g_isRunningFlag);
                g_motorMode = MotorVelocityMode;
                Motor_Driver_Reset(0, 0);
                usleep(500000);
                Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
                usleep(500000);
            }

            jointState currentJointState;
            currentJointState.j[0] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[1] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[2] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[5] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[6]);
            currentJointState.j[1] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[0] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[3] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[4] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[7]);
            currentJointState.t = currentJointState.duration = 0;
            p2p_jointTrajectoryPlan(currentJointState, rxb_currentJointState_Controllable_DOF, RXB_joint_plan);
            jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);

            g_rxbMode = RxbMode_TaskSpace_Ctrl;
            res.result = static_cast<unsigned short>(g_rxbMode);
        }
        return true;
    }

    void RxbHardwareInterface::ForPos_HybridJntCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &cmd){
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            ForPos_Hybrid_targetJoint.j[i] = cmd->data[static_cast<unsigned long>(i)] * Rad2Deg;
        }

        RXB_Cable_Plan.clear();
        RXB_joint_plan.clear();

        if(g_motorMode == MotorPositionMode) {
            g_motorMode = MotorVelocityMode;
            Motor_Driver_Reset(0, 0);
            usleep(500000);
            Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
            usleep(500000);
            ROS_INFO_STREAM("\033[41;37m Motor in Position Mode, enter to Velocity Mode successfuly \033[0m");

        }
        ROS_INFO_STREAM("\033[41;37m Running!\033[0m");
        rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;//从当前角度开始规划
        p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, ForPos_Hybrid_targetJoint, RXB_joint_plan);
        jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
//        cout << "RXB_Cable_Plan.size:" << RXB_Cable_Plan.size() << endl;
//        cout << "RXB_joint_plan.size:" << RXB_Cable_Plan.size() << endl;
    }


    void RxbHardwareInterface::closeLoopJntCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &cmd) {
      jointState targetJointAngle;
      for(int i = 0; i < Controllable_DOF_num_all; i++) {
          targetJointAngle.j[i] = cmd->data[i] * Rad2Deg;
      }

      RXB_Cable_Plan.clear();
      RXB_joint_plan.clear();

      if(g_motorMode == MotorPositionMode) {
          g_motorMode = MotorVelocityMode;
          Motor_Driver_Reset(0, 0);
          usleep(500000);
          Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
          usleep(500000);
          ROS_INFO_STREAM("\033[41;37m Motor in Position Mode, enter to Velocity Mode successfuly \033[0m");
      }
      ROS_INFO_STREAM("\033[41;37m Running!\033[0m");
      rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;//从当前角度开始规划
      p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, targetJointAngle, RXB_joint_plan);
      jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
      cout << "RXB_Cable_Plan.size:" << RXB_Cable_Plan.size() << endl;
      cout << "RXB_joint_plan.size:" << RXB_Cable_Plan.size() << endl;
    }

    bool RxbHardwareInterface::getRxbPose(rxb_msgs::RxbPose::Request &req, rxb_msgs::RxbPose::Response &res) {
        bool cmd = req.cmd;
        std::vector<Eigen::Matrix4d> Ti;
        Eigen::Matrix4d T_end;
        geometry_msgs::Pose pose;
        double theta[Controllable_DOF_num_all];//6
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            theta[i] = rxb_currentJointState_Controllable_DOF.j[i];
        }
        if(cmd == true) {
            fkd_kinematics(theta, Ti);
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++) {
                    T_end(i, j) = Ti[Controllable_DOF_num_all - 1](i, j);
                }
            res.success = false;
            res.pose = RotMat2Quatern(T_end);
            res.success = true;
            return true;
        } else {
            res.success = false;
            return false;
        }

    }


    void RxbHardwareInterface::readHW() {
        std_msgs::UInt32 rxb_state, motor_mode;
        rxb_state.data = g_isRunningFlag;//获取柔性臂当前状态信息（运行还是停止）
        motor_mode.data = g_motorMode;//获取当前电机模式（位置模式还是速度模式）

        getSensorData();//力传感器数据获取

        //数编码器数据传递给界面进程
        rxb_msgs::sensorRead sensorData;
        sensorData.cable_force.resize(Motor_num_all);
        sensorData.joint_angle.resize(Gene_Jnt_Num);
        for(int i = 0; i < Gene_Jnt_Num; i++) {
            sensorData.joint_angle[i] = g_jointAngle[i];//关节角在函数 SensorDataReveive 中更新，意味着关节角实际上从can总线的接收时间间隔为3ms，但是在上位机界面上的更新则和力传感器的时间间隔一样（20ms）
        }
        serial_encoder_pub.publish(sensorData);//更新界面角度编码器的数值

        // 向ui输出末端齐次变换矩阵
        ActualForwardKinematic(g_jointAngle);

        // 向ui输出末端速度旋量
        jointAngleSmooth.AddData(g_jointAngle);
        jointAngleKalman.AddData(jointAngleSmooth.result);
        CalBodyJacobian(BList, g_jointAngle);
        CalBodyTwist(bodyJacobian, jointAngleKalman.result);
    }

    void RxbHardwareInterface::communicateMoveit() {
        for(int i = 0; i < Gene_Jnt_Num; i++) {
            rxb_gene_currentJointState.j[i] = g_jointAngle[i];
            jointsCurrent.position[i] = rxb_gene_currentJointState.j[i] * Deg2Rad;
        }
        rxb_gene_currentJointState.t = rxb_setJointState.t;
        jointsCurrent.header.stamp = ros::Time::now();
        moveit_PubJoints.publish(jointsCurrent);  // ROS communication works in Radian
    }


    void RxbHardwareInterface::communicateMatlab() {

    }


    void RxbHardwareInterface::writeHW() {
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);

        static bool firstFlag = 1;  //启动数据记录后的第一次循环
        static uint timeTickCount = 0;

        //机械臂随机运动
        if(g_randomMotionFlag && RXB_Cable_Plan.size()==0 && RXB_joint_plan.size()==0){
            randomMotionFunc();
        }

        // 用于神经网络训练
        if(nnTrainingNewRcFlag && nnTrainingStartFlag && ++rcCount%5==0){
          ofstream oFile;
          oFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/nnTrainingData.txt", ios::app);
          // 只考虑Yaw转动
          oFile << g_jointAngle[0] << ' ' << g_jointAngle[3] << ' ' << g_jointAngle[4] << ' ' << g_jointAngle[7] << ' ';
          oFile << extension;
          oFile << endl;
          oFile.close();

          std_msgs::UInt8 pubData;
          pubData.data = static_cast<unsigned char>(rcCount*100/(5*250));
          nnTrainingProgressPub.publish(pubData);
          if(rcCount == 5*250){  //记录100秒的数据，总共1000个数据
            nnTrainingStartFlag = false;
          }
        }

        // 在线神经网络训练
        neuralNetwork_corCableForce_dataRecord_sw.AddData({g_jointAngle[1], g_jointAngle[2]});
        if(neuralNetwork_corCableForce_dataRecord_stateFlag){
            vector<double> input = neuralNetwork_corCableForce_dataRecord_sw.GetResult();
            if(input.size() != 0){
                fstream file;
                switch(neuralNetwork_corCableForce_dataRecord_stateFlag){
                case 1:
                    file.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/trainSet.txt", ios::app);
                    break;
                case 2:
                    file.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/devSet.txt", ios::app);
                    break;
                case 3:
                    file.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/testSet.txt", ios::app);
                    break;
                }
                for(unsigned long i = 0; i < input.size(); i++){
                    file << input[i] << " ";
                }
                file << g_sCorCableForce[0] << " " << g_sCorCableForce[1] << endl;
                file.close();

                neuralNetwork_corCableForce_dataRecord_count++;
                std_msgs::UInt32 count;
                count.data = neuralNetwork_corCableForce_dataRecord_count;
                neuralNetwork_corCableForce_dataRecord_count_pub.publish(count);
            }
        }
        if(neuralNetwork_corCableForce_model_dev_state){
            vector<double> input = neuralNetwork_corCableForce_dataRecord_sw.GetResult();
            if(input.size() != 0){
                rxb_msgs::corCableForceNN srv;
                for(unsigned long i = 0; i < input.size(); i++){
                    srv.request.jointAngle.push_back(static_cast<float>(input[i]));
                }
                srv.request.actCableForce.push_back(static_cast<float>(g_sCorCableForce[0]));
                srv.request.actCableForce.push_back(static_cast<float>(g_sCorCableForce[1]));

                if(neuralNetwork_corCableForce_model_client.call(srv)){
                    std_msgs::Float32MultiArray data;
                    for(unsigned long i = 0; i < srv.response.predCableForce.size(); i++){
                        data.data.push_back(srv.request.actCableForce[i]);
                        data.data.push_back(srv.response.predCableForce[i]);
                    }
                    data.data.push_back(srv.response.loss);
                    neuralNetwork_corCableForce_model_dev_data_pub.publish(data);
                }
            }
        }

        if(g_rxbMode == RxbMode_OLoop_Pos && g_motorMode == MotorPositionMode) {
            JointOloop_Control();

            if(g_recordCommandFlag == isRecordOnFlag){
                //数据分析
                ofstream ofile;
                if(firstFlag){
                    ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_Oloop.txt",ios::out);
                    firstFlag = 0;
                }else{
                    ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_Oloop.txt",ios::app);
                }
                ofile << 1900 + p->tm_year << "/" << 1 + p->tm_mon << "/" << p->tm_mday<< " " << p->tm_hour << ":" << p->tm_min << ":" <<p->tm_sec << ' ' ;
                ofile << "timeTick" << timeTickCount << " " << timeTickCount*global_T_Interval_ms << " ";
                ofile << "Plan_Joint" << " ";//规划关节角
                for (int i=0;i<Controllable_DOF_num_all;i++) {
                    ofile << rxb_currentJointState_Controllable_DOF.j[i] << ' ';
                }
                ofile << "Gen_Joint" << ' ';//角度编码器返回实际各关节角 YPPYYPPY
                for (int i=0;i<Gene_Jnt_Num;i++) {
                    ofile << g_jointAngle[i] << ' ';
                }
                ofile << "ActCable_Force" << " "; //驱动绳拉力
                for(int i = 0; i<Motor_num_all; i++){
                    ofile << g_actCableForce[i] << ' ';
                }
                ofile << "SCorCableForce" << " "; // 小8联动绳拉力，从前往后，由正到负
                for (int i=0;i<SCorCableForce_Num;i++) {
                    ofile << g_sCorCableForce[i] << ' ';
                }
                ofile << "BCorCableForce" << " "; // 大8联动绳拉力，从前往后，由正到负，同联动绳连续
                for (int i=0;i<BCorCableForce_Num;i++) {
                    ofile << g_bCorCableForce[i] << ' ';
                }
                ofile << "ExForceMeasured" << " ";
                ofile << exForceMeasured.angular.x << " ";
                ofile << exForceMeasured.angular.y << " ";
                ofile << exForceMeasured.angular.z << " ";
                ofile << exForceMeasured.linear.x << " ";
                ofile << exForceMeasured.linear.y << " ";
                ofile << exForceMeasured.linear.z << " ";
                ofile << "ExForceSensed" << " ";
                for(int i = 0; i < Controllable_DOF_num_all; i++){
                  ofile << forceSense[i] << " ";
                }
                ofile << endl;
                ofile.close();
                timeTickCount++;
            }else{
                firstFlag = 1;
                timeTickCount = 0;
            }
        }else if(g_rxbMode == RxbMode_CLoop_Pos && g_motorMode == MotorVelocityMode) {
          //关节闭环控制
          JointCloop_Control();

          if(g_recordCommandFlag == isRecordOnFlag){
            //数据分析
            ofstream ofile;
            if(firstFlag){
                ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_Cloop.txt",ios::out);
                firstFlag = 0;
            }else{
                ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_Cloop.txt",ios::app);
            }
            ofile << 1900 + p->tm_year << "/" << 1 + p->tm_mon << "/" << p->tm_mday<< " " << p->tm_hour << ":" << p->tm_min << ":" <<p->tm_sec << ' ' ;
            ofile << "timeTick" << timeTickCount << " " << timeTickCount*global_T_Interval_ms << " ";
            ofile << "Plan_Joint" << " ";//规划关节角
            for (int i=0;i<Controllable_DOF_num_all;i++) {
                ofile << rxb_currentJointState_Controllable_DOF.j[i] << ' ';
            }
            ofile << "Gen_Joint" << ' ';//角度编码器返回实际各关节角 YPPYYPPY
            for (int i=0;i<Gene_Jnt_Num;i++) {
                ofile << g_jointAngle[i] << ' ';
            }
            ofile << "ActCable_Force" << " "; //驱动绳拉力
            for(int i = 0; i<Motor_num_all; i++){
                ofile << g_actCableForce[i] << ' ';
            }
            ofile << "SCorCableForce" << " "; // 小8联动绳拉力，从前往后，由正到负
            for (int i=0;i<SCorCableForce_Num;i++) {
                ofile << g_sCorCableForce[i] << ' ';
            }
            ofile << "BCorCableForce" << " "; // 大8联动绳拉力，从前往后，由正到负，同联动绳连续
            for (int i=0;i<BCorCableForce_Num;i++) {
                ofile << g_bCorCableForce[i] << ' ';
            }
            ofile << "ExForceMeasured" << " ";
            ofile << exForceMeasured.angular.x << " ";
            ofile << exForceMeasured.angular.y << " ";
            ofile << exForceMeasured.angular.z << " ";
            ofile << exForceMeasured.linear.x << " ";
            ofile << exForceMeasured.linear.y << " ";
            ofile << exForceMeasured.linear.z << " ";
            ofile << "ExForceSensed" << " ";
            for(int i = 0; i < Controllable_DOF_num_all; i++){
              ofile << forceSense[i] << " ";
            }
            ofile << endl;
            ofile.close();
            timeTickCount++;
          }else{
              firstFlag = 1;
              timeTickCount = 0;
          }
        }else if(g_rxbMode == RxbMode_ForPos_Hybrid && g_motorMode == MotorVelocityMode) {
            //力控绳索选择
            ConstForceCableChoose();
            //关节闭环控制
            JointHybrid_Control();
            //绳索恒力控制
            Const_CableForce_Control();

            if(g_recordCommandFlag == isRecordOnFlag){
                //数据分析
                ofstream ofile;
                if(firstFlag){
                    ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_Hybrid.txt",ios::out);
                    firstFlag = 0;
                }else{
                    ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_Hybrid.txt",ios::app);
                }
                ofile << 1900 + p->tm_year << "/" << 1 + p->tm_mon << "/" << p->tm_mday<< " " << p->tm_hour << ":" << p->tm_min << ":" <<p->tm_sec << ' ' ;
                ofile << "timeTick" << timeTickCount << " " << timeTickCount*global_T_Interval_ms << " ";
                ofile << "Plan_Joint" << " ";//规划关节角
                for (int i=0;i<Controllable_DOF_num_all;i++) {
                    ofile << rxb_currentJointState_Controllable_DOF.j[i] << ' ';
                }
                ofile << "Gen_Joint" << ' ';//角度编码器返回实际各关节角 YPPYYPPY
                for (int i=0;i<Gene_Jnt_Num;i++) {
                    ofile << g_jointAngle[i] << ' ';
                }
                ofile << "ActCable_Force" << " "; //驱动绳拉力
                for(int i = 0; i<Motor_num_all; i++){
                    ofile << g_actCableForce[i] << ' ';
                }
                ofile << "SCorCableForce" << " "; // 小8联动绳拉力，从前往后，由正到负
                for (int i=0;i<SCorCableForce_Num;i++) {
                    ofile << g_sCorCableForce[i] << ' ';
                }
                ofile << "BCorCableForce" << " "; // 大8联动绳拉力，从前往后，由正到负，同联动绳连续
                for (int i=0;i<BCorCableForce_Num;i++) {
                    ofile << g_bCorCableForce[i] << ' ';
                }
                ofile << "ExForceMeasured" << " ";
                ofile << exForceMeasured.angular.x << " ";
                ofile << exForceMeasured.angular.y << " ";
                ofile << exForceMeasured.angular.z << " ";
                ofile << exForceMeasured.linear.x << " ";
                ofile << exForceMeasured.linear.y << " ";
                ofile << exForceMeasured.linear.z << " ";
                ofile << "ExForceSensed" << " ";
                for(int i = 0; i < Controllable_DOF_num_all; i++){
                  ofile << forceSense[i] << " ";
                }
                ofile << endl;
                ofile.close();
                timeTickCount++;
            }else{
                firstFlag = 1;
                timeTickCount = 0;
            }
        }else if(g_rxbMode == RxbMode_Adjust_Cable && g_motorMode == MotorVelocityMode){

        }else if(g_rxbMode == RxbMode_TaskSpace_Ctrl && g_motorMode == MotorVelocityMode){
            ConstForceCableChoose();
            TaskSpace_Control();
            Const_CableForce_Control();

            if(g_recordCommandFlag == isRecordOnFlag){
              //数据分析
              ofstream ofile;
              if(firstFlag){
                  ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_TaskSpaceCtrl.txt",ios::out);
                  firstFlag = 0;
              }else{
                  ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/qrealdata_TaskSpaceCtrl.txt",ios::app);
              }
              ofile << 1900 + p->tm_year << "/" << 1 + p->tm_mon << "/" << p->tm_mday<< " " << p->tm_hour << ":" << p->tm_min << ":" <<p->tm_sec << ' ' ;
              ofile << "timeTick" << timeTickCount << " " << timeTickCount*global_T_Interval_ms << " ";
              ofile << "Desired_Joint" << " ";
              for (unsigned long i=0;i<Controllable_DOF_num_all;i++) {
                  ofile << desiredJoint[i] << ' ';
              }
              ofile << "Plan_Joint" << " ";//规划关节角
              for (int i=0;i<Controllable_DOF_num_all;i++) {
                  ofile << rxb_currentJointState_Controllable_DOF.j[i] << ' ';
              }
              ofile << "Gen_Joint" << ' ';//角度编码器返回实际各关节角 YPPYYPPY
              for (int i=0;i<Gene_Jnt_Num;i++) {
                  ofile << g_jointAngle[i] << ' ';
              }
              ofile << "ActCable_Force" << " "; //驱动绳拉力
              for(int i = 0; i<Motor_num_all; i++){
                  ofile << g_actCableForce[i] << ' ';
              }
              ofile << "SCorCableForce" << " "; // 小8联动绳拉力，从前往后，由正到负
              for (int i=0;i<SCorCableForce_Num;i++) {
                  ofile << g_sCorCableForce[i] << ' ';
              }
              ofile << "BCorCableForce" << " "; // 大8联动绳拉力，从前往后，由正到负，同联动绳连续
              for (int i=0;i<BCorCableForce_Num;i++) {
                  ofile << g_bCorCableForce[i] << ' ';
              }
              ofile << "ExForceMeasured" << " ";
              ofile << exForceMeasured.angular.x << " ";
              ofile << exForceMeasured.angular.y << " ";
              ofile << exForceMeasured.angular.z << " ";
              ofile << exForceMeasured.linear.x << " ";
              ofile << exForceMeasured.linear.y << " ";
              ofile << exForceMeasured.linear.z << " ";
              ofile << "ExForceSensed" << " ";
              for(int i = 0; i < Controllable_DOF_num_all; i++){
                ofile << forceSense[i] << " ";
              }
              ofile << endl;
              ofile.close();
              timeTickCount++;
            }else{
                firstFlag = 1;
                timeTickCount = 0;
            }
        }
        g_isRunningFlag = 0;
    }

    void RxbHardwareInterface::timerCallback(const ros::TimerEvent &) {
        if(pause_flag==false){// && g_InitForceFlag==true
          readHW();//传感器数据获取 g_cableForce  g_JointAngle
          communicateMoveit();//实时仿真画面
          writeHW();//将数据保存至文件，后续进行数据分析；硬件控制
        }
    }

    /*void RxbHardwareInterface::Sensordata_timerCallback(const ros::TimerEvent &) {

        //cout <<"in Sensordata_timercallback" << endl;

        int temp = (consti%2)+1;
        consti ++;//一个初始值为0的全局变量，只在此处出现，那为什么要定义成全局变量呢？
        if(Encoder_zero_index == true && temp == Encoder_zero_cal){
            SensorDataGet(temp,3);
            cout << temp << endl;
            Encoder_zero_cal ++;
            if(Encoder_zero_cal == 3){
                Encoder_zero_index = false;
                Encoder_zero_cal = 0;
            }
        }
        else {
            SensorDataGet(temp,1);//第几段（1，2，3），功能码（1，2，3）
            SensorDataGet(temp,2);
            g_InitJointAngleFlag = true;
        }
    }*/

    void RxbHardwareInterface::Sensordata_timerCallback(const ros::TimerEvent &) {
        static int i = 0;
        i++;//第一次读取第一段的关节角

        SensorDataGet(i);

        if (i >= 2){
            i=0;
        }
    }

    void RxbHardwareInterface::mainLoop() {
        ROS_INFO("Starting Program Main Loop");
        ROS_INFO("......Waitting for rxb init...........");
        init();

        while(ros::ok()) {

        }
        cout << "main_func exit" << endl;

        can_close();
        ros::shutdown();
    }

    void RxbHardwareInterface::serial_data_subFun(const rxb_msgs::sensorRead::ConstPtr &data) {
        for(int i = 0; i < Motor_num_all; i++){
            g_actCableForce[i] = data->cable_force[g_actCableID[i]];
        }
        for(int i = 0; i < SCorCableForce_Num; i++){
            g_sCorCableForce[i] = data->cable_force[g_sCorCableID[i]];
        }
        for(int i = 0; i < BCorCableForce_Num; i++){
            g_bCorCableForce[i] = data->cable_force[g_bCorCableID[i]];
        }
    }

    void RxbHardwareInterface::getSensorData() {
        rxb_msgs::serialCommu sp_data;
        sp_data.data.resize(12);//调整容器的大小，在这里即调整sp_data.data的容纳的元素数量，sp_data.data是一个元素为char类型的数组
        sp_data.data[0] = 'S';
        sp_data.data[1] = 'A';
        for(int i = 0; i < 10; i++) {
            sp_data.data[i + 2] = 0x66;
        }
        serial_pub.publish(sp_data);
        //cout << "send sensor cmd" << endl;

    }
    void RxbHardwareInterface::Encoder_zero_subFun(const std_msgs::UInt32::ConstPtr& data) {
        if(data->data){
            ofstream ofile;
            ofile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/jointAngle_Zero.txt", ios::out);//因为臂段的角度编码器不会随着断电清零，需要将柔性臂零位的实际关节角记录下来，来计算相对关节角
            if(!ofile.is_open()){
                std::cout << "Fail to open the file!";
            }else{
                for (unsigned int i = g_RXBSegment_Joint_num*(data->data-1); i < Gene_Jnt_Num && i < g_RXBSegment_Joint_num*data->data; i++){
                    g_JointAngle_Zero[i] = g_JointAngle_Absolute[i];//当前程序臂段角度编码器归零
                }
                for(int i = 0; i < Gene_Jnt_Num; i++){
                    ofile << g_JointAngle_Zero[i] << std::endl;
                }
            }
            ofile.close();
        }

    }

    void RxbHardwareInterface::J1PosPIDCallback(const std_msgs::Float64MultiArray::ConstPtr &datas) {
        if(static_cast<int>(datas->data[3]) == 0)//位置pid
        {
            Joint_pid_kp_FirstGroup[ int(datas->data[4]) -1 ] = datas->data[0];
            Joint_pid_ki_FirstGroup[ int(datas->data[4]) -1 ] = datas->data[1];
            Joint_pid_kd_FirstGroup[ int(datas->data[4]) -1 ] = datas->data[2];
            cout << "setting pos PID successful" << endl;
        }else if(static_cast<int>(datas->data[3]) == 1){//力pid
            Force_pid_kp[ int(datas->data[4]) -1 ] = datas->data[0];
            Force_pid_ki[ int(datas->data[4]) -1 ] = datas->data[1];
            Force_pid_kd[ int(datas->data[4]) -1 ] = datas->data[2];
            cout << "setting Force PID successful"  << endl;
        }
    }

    void RxbHardwareInterface::J1PosPIDWriteInCallback(const std_msgs::Float64MultiArray::ConstPtr &datas) {
        ofs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/PosPIDSaved.txt", ios::out);
        for(int i = 0; i < ModuJnt_num_all; i++) {
            ofs << datas->data[i*3] << endl;
            ofs << datas->data[i*3+1] << endl;
            ofs << datas->data[i*3+2] << endl;

        }

        ofs.close();
        cout << "setting pos PID write successful" << endl;
    }

    void RxbHardwareInterface::ForcePIDWriteInCallback(const std_msgs::Float64MultiArray::ConstPtr &datas) {

        ofs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/ForcePIDSaved.txt", ios::out);
        for(int i = 0; i < g_RXBSegment_num; i++) {
            ofs << datas->data[i*3] << endl;
            ofs << datas->data[i*3+1] << endl;
            ofs << datas->data[i*3+2] << endl;

        }

        ofs.close();
        cout << "setting force PID write successful" << endl;
    }

    //关节开环控制
    void RxbHardwareInterface::JointOloop_Control() {
        u8 Group_Num, Driver_Num;
        fp32 Cable_Vel_Cacu;
        fp32 CableMove_Plan;//相对移动距离

        int Cable_num = 0;
        std_msgs::Float64MultiArray Planing_jnt_ang;

        Planing_jnt_ang.data.resize(Controllable_DOF_num_all);
        double CableAdjust_coefficient[Motor_num_all] = {1,1,1};  //绳索补偿系数 {1， 1， 1， 1， 1， 1}
        if(RXB_Cable_Plan.size() > 0 && RXB_joint_plan.size() > 0) {//预定轨迹走没走完
            if(trajectoryDemo_flag && g_recordCommandFlag == isRecordOffFlag){
                g_recordCommandFlag = isRecordOnFlag;
                g_recordSwitchFlag = true;
            }

            g_isRunningFlag = 1;

            //关节数据
            rxb_setJointState = RXB_joint_plan.front();
            RXB_joint_plan.pop_front();

            ofs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/joint_saved.txt", ios::out);
            for(int i = 0; i < Controllable_DOF_num_all; i++) {
                rxb_currentJointState_Controllable_DOF.j[i] = rxb_setJointState.j[i];
                Planing_jnt_ang.data[i] = rxb_setJointState.j[i];
                ofs << rxb_currentJointState_Controllable_DOF.j[i] << endl;//保存的是当前规划的轨迹点角度值
            }
            rxb_currentJointState_Controllable_DOF.t = rxb_setJointState.t;
            ofs.close();
            PlaningJntPos_Pub.publish(Planing_jnt_ang);//返回面板关节角规划值

            //绳索数据
            rxb_priorCableState = rxb_setCableState;//更新上一时刻的绳索数据，注意，不同于没有使用的rxb_priorJointState，这个变量在柔性臂初始化时有加载上次运行最后的记录值
            rxb_setCableState = RXB_Cable_Plan.front();
            RXB_Cable_Plan.pop_front();
            for(int i = 0; i < Motor_num_all; i++) {//i对应绳索号不对应电机号
                Cable_num = i + 1;//绳索编号，第一段绳索编号分别为1、2、3，第二段分别为4、5、6，以此类推
                Group_Num = g_Driver_ID[Cable_num - 1] >> 4 & 0x0f;//对应驱动器组数
                Driver_Num = g_Driver_ID[Cable_num - 1] - 15 * Group_Num;//对应驱动器号数

                computerCableVel(Cable_num, rxb_setCableState, rxb_priorCableState, Cable_Vel_Cacu);//这一时刻的规划值减去上一时刻的规划值，除以控制时间（20ms）
                CableMove_Plan = (rxb_setCableState.len[Cable_num - 1] - g_offset_CablePos[Cable_num - 1]);//绳索应当移动的距离，用目标位置的绳长将去初始位置的绳长
                if(g_rxbWorkStatus==1){//开启则会驱动电机，关闭可用于调试
                    Motor_Driver_Velocity_Position_Mode(Group_Num, Driver_Num, 5000, (short)(fabs(Cable_Vel_Cacu * g_DrivenBox_V[Cable_num-1] * CableAdjust_coefficient[Cable_num-1])), (long)(CableMove_Plan * g_DrivenBox[Cable_num - 1] * CableAdjust_coefficient[Cable_num-1]) );
                }
            }
        }else{
            if(trajectoryDemo_flag){
                if(g_recordSwitchFlag){
                   g_recordCommandFlag = isRecordOffFlag;
                   g_recordSwitchFlag = false;
                }
                trajectoryDemo_flag = false;
            }
        }
    }

    //关节位置力位混合控制（多组驱动,关节限速，绳索限速）
    void RxbHardwareInterface::JointCloop_Control() {

        u8 Group_Num, Driver_Num;
        int Cable_num = 0;
        double cableL_vel_FirstGroup[DriveGroup_num*3*ModuJnt_num_all];
        double cableL_vel[DriveGroup_num*3*ModuJnt_num_all];
        double motor_vel[Motor_num_all];
        std_msgs::Float64MultiArray Planing_jnt_ang;
        Planing_jnt_ang.data.resize(Controllable_DOF_num_all);

        vector<Eigen::Matrix4d> Ti_ideal, Ti_actual, Ti;
        Eigen::Matrix4d T_end, T_actual_end, T_ideal_end;
        Eigen::Vector3d dPos, dPos_old, dA, dA_old;

        g_isRunningFlag = 1;
        //对每个可控自由度，得到等效关节角，顺序PYPYPY
        g_equ_JointAngle_FirstGroup[0] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[1] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[2] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[5] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[6]);
        g_equ_JointAngle_FirstGroup[1] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[0] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[3] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[4] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[7]);
        for (int i = 0; i < Controllable_DOF_num_all; i++){
            cout << "g_equ_jointAngle[" << i << "] : " << g_equ_JointAngle_FirstGroup[i] << endl;
        }

        //策略：第一组驱动（末端）使用同向角平均值作为反馈值，第二组驱动（基座）使用靠近基座的角作为反馈值
        if(RXB_joint_plan.size() > 0 && RXB_Cable_Plan.size()>0){
            if(trajectoryDemo_flag && g_recordCommandFlag == isRecordOffFlag){
                g_recordCommandFlag = isRecordOnFlag;
                g_recordSwitchFlag = true;
            }

            rxb_setJointState = RXB_joint_plan.front();
            RXB_joint_plan.pop_front();
            RXB_Cable_Plan.pop_front();
        }else{
            if(trajectoryDemo_flag){
                if(g_recordSwitchFlag){
                   g_recordCommandFlag = isRecordOffFlag;
                   g_recordSwitchFlag = false;
                }
                trajectoryDemo_flag = false;
            }
        }
        pos_jointq = rxb_setJointState;
        admi_jointq = pos_jointq;

        if(admittanceControlIsOn){
          AdmittanceControl(forceSense, acState, M, B, K, global_T_Interval_s);
          for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
            pos_jointq.j[i] += acState[i].x;
          }
        }

        cout << "Input planning joint angle is:  " ;
        for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
          cout << admi_jointq.j[i] << "  ";
        }
        cout << endl;
        cout << "Output planning joint angle is:  " ;
        for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
          cout << pos_jointq.j[i] << "  ";
        }
        cout << endl;

        //规划数据记录
        ofs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/joint_saved.txt", ios::out);
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            rxb_currentJointState_Controllable_DOF.j[i] = pos_jointq.j[i];
            Planing_jnt_ang.data[static_cast<unsigned long>(i)] = pos_jointq.j[i];
            ofs << pos_jointq.j[i] << endl;//保存的是当前规划的轨迹点角度值
        }
        rxb_currentJointState_Controllable_DOF.t = pos_jointq.t;
        ofs.close();
        PlaningJntPos_Pub.publish(Planing_jnt_ang);

        //闭环控制与开环控制不同,只要进入了力位混合控制后，柔性臂会直接开始
        for(int i = 0; i < Controllable_DOF_num_all; ++i) {
            //第一组驱动的pidout计算
            derror_joint_FirstGroup[i] = (pos_jointq.j[i] - g_equ_JointAngle_FirstGroup[i]) - error_joint_FirstGroup[i];//delta_deg
            error_joint_FirstGroup[i] = pos_jointq.j[i] - g_equ_JointAngle_FirstGroup[i];//deg
            ierror_joint_FirstGroup[i] = ierror_joint_FirstGroup[i] + error_joint_FirstGroup[i];
            //可控关节角角速度
            Joint_pid_out_FirstGroup[i] = Joint_pid_kp_FirstGroup[i/2] * error_joint_FirstGroup[i] / 10.0 + Joint_pid_kd_FirstGroup[i/2] * derror_joint_FirstGroup[i] / 10.0 + Joint_pid_ki_FirstGroup[i/2] * ierror_joint_FirstGroup[i] / 10.0 + pos_jointq.vel[i];
            cout << "Joint_pid_kp_FirstGroup[" << i << "] : " << Joint_pid_kp_FirstGroup[i/2] << endl;
            cout << "Joint_pid_ki_FirstGroup[" << i << "] : " << Joint_pid_ki_FirstGroup[i/2] << endl;
            cout << "Joint_pid_kd_FirstGroup[" << i << "] : " << Joint_pid_kd_FirstGroup[i/2] << endl;
            cout << "Joint_pid_out_FirstGroup[" << i << "] : " << Joint_pid_out_FirstGroup[i] << endl;
        }

        RXB_JntState_To_CableVel(g_equ_JointAngle_FirstGroup, Joint_pid_out_FirstGroup, cableL_vel_FirstGroup);//pid_out to pos_jointq.vel cable_vel : mm/s

        double max_motor_vel = DBL_MIN;
        for(int i = 0; i < Motor_num_all; i++){
          cableL_vel[i] = cableL_vel_FirstGroup[i];
          motor_vel[i] = cableL_vel[i] * g_DrivenBox_V[i];
          max_motor_vel = max(fabs(max_motor_vel), fabs(motor_vel[i]));
        }

        for(int i = 0; i < Motor_num_all; i++) {//绳索号  Motor_num_all
            Cable_num = i + 1;
            Group_Num = g_Driver_ID[Cable_num - 1] >> 4 & 0x0f;
            Driver_Num = g_Driver_ID[Cable_num - 1] - 15 * Group_Num;

            if(fabs(max_motor_vel) > fabs(V_max[0])){
              motor_vel[i] /= fabs(max_motor_vel/V_max[0]);
            }
            cout << "Motorspeed-[Cable " << Cable_num << "]:   "<< motor_vel[i] << endl;
            if(g_rxbWorkStatus==1)
                Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,static_cast<long>(motor_vel[i]));
            else
                Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,0);
        }
    }

    //关节位置力位混合控制（多组驱动,关节限速，绳索限速）
    void RxbHardwareInterface::JointHybrid_Control() {

        u8 Group_Num, Driver_Num;
        int Cable_num = 0;
        double cableL_vel_FirstGroup[DriveGroup_num*3*ModuJnt_num_all];
        double cableL_vel[DriveGroup_num*3*ModuJnt_num_all];
        double motor_vel[Motor_num_all];
        std_msgs::Float64MultiArray Planing_jnt_ang;
        Planing_jnt_ang.data.resize(Controllable_DOF_num_all);

        vector<Eigen::Matrix4d> Ti_ideal, Ti_actual, Ti;
        Eigen::Matrix4d T_end, T_actual_end, T_ideal_end;
        Eigen::Vector3d dPos, dPos_old, dA, dA_old;

        g_isRunningFlag = 1;
        //对每个可控自由度，得到等效关节角，顺序PYPYPY
        g_equ_JointAngle_FirstGroup[0] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[1] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[2] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[5] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[6]);
        g_equ_JointAngle_FirstGroup[1] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[0] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[3] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[4] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[7]);

        //策略：第一组驱动（末端）使用同向角平均值作为反馈值，第二组驱动（基座）使用靠近基座的角作为反馈值
        if(RXB_joint_plan.size() > 0 && RXB_Cable_Plan.size()>0){
            if(trajectoryDemo_flag && g_recordCommandFlag == isRecordOffFlag){
                g_recordCommandFlag = isRecordOnFlag;
                g_recordSwitchFlag = true;
            }

            rxb_setJointState = RXB_joint_plan.front();
            RXB_joint_plan.pop_front();
            RXB_Cable_Plan.pop_front();
        }else{
            if(trajectoryDemo_flag){
                if(g_recordSwitchFlag){
                   g_recordCommandFlag = isRecordOffFlag;
                   g_recordSwitchFlag = false;
                }
                trajectoryDemo_flag = false;
            }
        }
        pos_jointq = rxb_setJointState;
        admi_jointq = pos_jointq;

        if(admittanceControlIsOn){
          AdmittanceControl(forceSense, acState, M, B, K, global_T_Interval_s);
          for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
            pos_jointq.j[i] += acState[i].x;
          }
        }

        //规划数据记录
        ofs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/joint_saved.txt", ios::out);
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            rxb_currentJointState_Controllable_DOF.j[i] = pos_jointq.j[i];
            Planing_jnt_ang.data[static_cast<unsigned long>(i)] = pos_jointq.j[i];
            ofs << pos_jointq.j[i] << endl;//保存的是当前规划的轨迹点角度值
        }
        rxb_currentJointState_Controllable_DOF.t = pos_jointq.t;
        ofs.close();
        PlaningJntPos_Pub.publish(Planing_jnt_ang);

        //力位混合控制与开环控制不同,只要进入了力位混合控制后，柔性臂会直接开始
        for(int i = 0; i < Controllable_DOF_num_all; ++i) {//力位混合控制与开环控制不同,只要进入了力位混合控制后，柔性臂会直接开始
            //第一组驱动的pidout计算
            derror_joint_FirstGroup[i] = (pos_jointq.j[i] - g_equ_JointAngle_FirstGroup[i]) - error_joint_FirstGroup[i];//delta_deg
            error_joint_FirstGroup[i] = pos_jointq.j[i] - g_equ_JointAngle_FirstGroup[i];//deg
            ierror_joint_FirstGroup[i] = ierror_joint_FirstGroup[i] + error_joint_FirstGroup[i];

            //可控关节角角速度
            Joint_pid_out_FirstGroup[i] = Joint_pid_kp_FirstGroup[i/2] * error_joint_FirstGroup[i] / 10.0 + Joint_pid_kd_FirstGroup[i/2] * derror_joint_FirstGroup[i] / 10.0 + Joint_pid_ki_FirstGroup[i/2] * ierror_joint_FirstGroup[i] / 10.0 + pos_jointq.vel[i];

//            cout << "Joint_pid_kp_FirstGroup[" << i << "] : " << Joint_pid_kp_FirstGroup[i/2] << endl;
//            cout << "Joint_pid_ki_FirstGroup[" << i << "] : " << Joint_pid_ki_FirstGroup[i/2] << endl;
//            cout << "Joint_pid_kd_FirstGroup[" << i << "] : " << Joint_pid_kd_FirstGroup[i/2] << endl;
//            cout << "Joint_pid_out_FirstGroup[" << i << "] : " << Joint_pid_out_FirstGroup[i] << endl;
        }

        RXB_JntState_To_CableVel(g_equ_JointAngle_FirstGroup, Joint_pid_out_FirstGroup, cableL_vel_FirstGroup);//pid_out to pos_jointq.vel cable_vel : mm/s

        double max_motor_vel = DBL_MIN;
        for(int i = 0; i < Motor_num_all; i++){
          cableL_vel[i] = cableL_vel_FirstGroup[i];
          motor_vel[i] = cableL_vel[i] * g_DrivenBox_V[i];
          max_motor_vel = max(fabs(max_motor_vel), fabs(motor_vel[i]));
        }

        for(int i = 0; i < Motor_num_all; i++) {//绳索号  Motor_num_all
            Cable_num = i + 1;
            Group_Num = g_Driver_ID[Cable_num - 1] >> 4 & 0x0f;
            Driver_Num = g_Driver_ID[Cable_num - 1] - 15 * Group_Num;

            if(fabs(max_motor_vel) > fabs(V_max[0])){
              motor_vel[i] /= fabs(max_motor_vel/V_max[0]);
            }
//            cout << "Motorspeed-[Cable " << Cable_num << "]:   "<< motor_vel[i] << endl;
            if(i != g_ConstForceCable_ID[0]/* && i != g_ConstForceCable_ID[1] && i != g_ConstForceCable_ID[2]*/){//不是恒力控绳索 i != g_ConstForceCable_ID[0] && i != g_ConstForceCable_ID[1]
                if(g_rxbWorkStatus==1)
                    Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,static_cast<long>(motor_vel[i]));
                else
                    Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,0);
            }
        }
        for(int i=0; i < DriveGroup_num*ModuJnt_num_all; i++){//DriveGroup_num*ModuJnt_num_all
            motorVel_ad2_ConstForceCable[i] = motor_vel[g_ConstForceCable_ID[i]];
        }

        cout << "------------------------------------------" << endl;
        static auto timePoint1 = std::chrono::high_resolution_clock::now();
        static auto timePoint2 = std::chrono::high_resolution_clock::now();
        timePoint2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint2 - timePoint1);
        std::cout << "Time interval: " << duration.count() << " milliseconds" << std::endl;
        timePoint1 = timePoint2;
        for (int i = 0; i < Controllable_DOF_num_all; i++){
            cout << "g_equ_jointAngle[" << i << "] : " << g_equ_JointAngle_FirstGroup[i] << endl;
        }
        cout << "Input planning joint angle is:  " ;
        for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
          cout << admi_jointq.j[i] << "  ";
        }
        cout << endl;
        cout << "Output planning joint angle is:  " ;
        for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
          cout << pos_jointq.j[i] << "  ";
        }
        cout << endl;
        for(int i = 0; i < Controllable_DOF_num_all; ++i) {//力位混合控制与开环控制不同,只要进入了力位混合控制后，柔性臂会直接开始
            cout << "Joint_pid_kp_FirstGroup[" << i << "] : " << Joint_pid_kp_FirstGroup[i/2] << endl;
            cout << "Joint_pid_ki_FirstGroup[" << i << "] : " << Joint_pid_ki_FirstGroup[i/2] << endl;
            cout << "Joint_pid_kd_FirstGroup[" << i << "] : " << Joint_pid_kd_FirstGroup[i/2] << endl;
            cout << "Joint_pid_out_FirstGroup[" << i << "] : " << Joint_pid_out_FirstGroup[i] << endl;
        }
        for(int i = 0; i < Motor_num_all; i++) {//绳索号  Motor_num_all
            cout << "Motorspeed-[Cable " << Cable_num << "]:   "<< motor_vel[i] << endl;
        }
    }

    //力控绳索恒力控制
    void RxbHardwareInterface::Const_CableForce_Control() {
        u8 Group_Num, Driver_Num;
        int Cable_num = 0;
        double motor_vel[ModuJnt_num_all*DriveGroup_num];

        for(int i = 0; i < ModuJnt_num_all*DriveGroup_num ; i++) { //ModuJnt_num_all*DriveGroup_num
            Cable_num = g_ConstForceCable_ID[i] + 1;//绳索号，从1开始
            Group_Num = g_Driver_ID[Cable_num - 1] >> 4 & 0x0f;
            Driver_Num = g_Driver_ID[Cable_num - 1] - 15 * Group_Num;

            //这里默认采用P控制，即I和D参数都为零，这是因为最小拉力绳索在不停选择
            derror_force[i] = (g_expect_ConstCableForce[i] - g_actCableForce[Cable_num-1]) - error_force[i];//delta N
            error_force[i] = g_expect_ConstCableForce[i] - g_actCableForce[Cable_num-1];//N
            ierror_force[i] = ierror_force[i] + error_force[i];//N
            Force_pid_out[i] = Force_pid_kp[i]*error_force[i]/100 + Force_pid_kd[i]*derror_force[i]/100 + Force_pid_ki[i]*ierror_force[i]/100;
            motor_vel[i] = -1*Force_pid_out[i]*g_DrivenBox_V[i];//绳索速度转换成电机速度
            //速度负为拉，正为松

            cout << "error_force = " << error_force[i] << "\tForce_pid_kp = " << Force_pid_kp[i] << "\tForce_pid_out = " << Force_pid_out[i] << endl;

            //绳索限速
            if(motor_vel[i] > V_max[i]){
                motor_vel[i] = V_max[i];
            }else if (motor_vel[i] < V_min[i]) {
                motor_vel[i] = V_min[i];
            }
            cout << "Motorspeed-[Cable " << Cable_num << "]:   "<< motor_vel[i] << endl;
            if(g_rxbWorkStatus==1)
                Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,static_cast<long>(motor_vel[i] + motorVel_ad2_ConstForceCable[i]));//+ cableLvel_ad2_ConstForceCable[i]*g_DrivenBox_V[i]
            else
                Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,0);//+ cableLvel_ad2_ConstForceCable[i]*g_DrivenBox_V[i]

        }
    }

    //力控绳索选择
    void RxbHardwareInterface::ConstForceCableChoose() {

        double CableForceMin[ModuJnt_num_all*DriveGroup_num];//ModuJnt_num_all*DriveGroup_num
        for(int i = 0; i < ModuJnt_num_all*DriveGroup_num ; i++){//ModuJnt_num_all*DriveGroup_num
            for(int j=0 ; j<3 ; j++){
                if(j == 0){
                    CableForceMin[i] = g_actCableForce[i*3+j];
                    g_ConstForceCable_ID[i] = i*3+j;
                }else if(g_actCableForce[i*3+j] < CableForceMin[i]){
                    CableForceMin[i] = g_actCableForce[i*3+j];
                    g_ConstForceCable_ID[i] = i*3+j;
                }
            }
        }
    }

    /// --------------------------------------------------------------------------------------------
    /// 函数功能：任务空间控制
    /// 返回类型：
    ///     void
    void RxbHardwareInterface::TaskSpace_Control(void){
        u8 Group_Num, Driver_Num;
        int Cable_num = 0;
        double cableL_vel_FirstGroup[DriveGroup_num*3*ModuJnt_num_all];
        double cableL_vel[DriveGroup_num*3*ModuJnt_num_all];
        double motor_vel[Motor_num_all];
        std_msgs::Float64MultiArray Planing_jnt_ang;
        Planing_jnt_ang.data.resize(Controllable_DOF_num_all);

        vector<Eigen::Matrix4d> Ti_ideal, Ti_actual, Ti;
        Eigen::Matrix4d T_end, T_actual_end, T_ideal_end;
        Eigen::Vector3d dPos, dPos_old, dA, dA_old;

        g_isRunningFlag = 1;
        //对每个可控自由度，得到等效关节角，顺序PYPYPY
        g_equ_JointAngle_FirstGroup[0] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[1] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[2] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[5] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[6]);
        g_equ_JointAngle_FirstGroup[1] = (JointAngle_pid_weight_FirstGroup[0]*g_jointAngle[0] + JointAngle_pid_weight_FirstGroup[1]*g_jointAngle[3] + JointAngle_pid_weight_FirstGroup[2]*g_jointAngle[4] + JointAngle_pid_weight_FirstGroup[3]*g_jointAngle[7]);
        for (int i = 0; i < Controllable_DOF_num_all; i++){
            cout << "g_equ_jointAngle[" << i << "] : " << g_equ_JointAngle_FirstGroup[i] << endl;
        }

        //策略：第一组驱动（末端）使用同向角平均值作为反馈值，第二组驱动（基座）使用靠近基座的角作为反馈值
        if(RXB_joint_plan.size() > 0 && RXB_Cable_Plan.size()>0){
            if(trajectoryDemo_flag && g_recordCommandFlag == isRecordOffFlag){
                g_recordCommandFlag = isRecordOnFlag;
                g_recordSwitchFlag = true;
            }

            rxb_setJointState = RXB_joint_plan.front();
            RXB_joint_plan.pop_front();
            RXB_Cable_Plan.pop_front();
        }else{
            if(trajectoryDemo_flag){
                if(g_recordSwitchFlag){
                   g_recordCommandFlag = isRecordOffFlag;
                   g_recordSwitchFlag = false;
                }
                trajectoryDemo_flag = false;
            }
        }
        for(int i = 0; i < Controllable_DOF_num_all; i++){
            desiredJoint[static_cast<unsigned long>(i)] = rxb_setJointState.j[i];
        }
        pos_jointq.t = rxb_setJointState.t;
        pos_jointq.duration = rxb_setJointState.duration;

//        if(admittanceControlIsOn){
//          double M = 1, B = 2, K = 0;

//          AdmittanceControl(forceSense, acState, M, B, K, global_T_Interval_s);
//          for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
//            pos_jointq.j[i] += acState[i].x;
//          }
//        }

//        cout << "Input planning joint angle is:  " ;
//        for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
//          cout << admi_jointq.j[i] << "  ";
//        }
//        cout << endl;
//        cout << "Output planning joint angle is:  " ;
//        for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
//          cout << pos_jointq.j[i] << "  ";
//        }
//        cout << endl;

        // 段空间闭环
        rxb_msgs::taskSpaceCtrl_singleSeg srv;
        for(int i = 0; i < Controllable_DOF_num_all; i++){
            srv.request.desiredJoint.push_back(static_cast<float>(desiredJoint[i]));
            srv.request.planJoint.push_back(static_cast<float>(rxb_currentJointState_Controllable_DOF.j[i]));
        }
        if(!taskSpaceCtrl_singleSeg_client.call(srv) || !srv.response.success){
            return;
        }else{
            for(int i = 0; i < Controllable_DOF_num_all; i++){
                pos_jointq.j[i] = static_cast<double>(srv.response.compensateJoint[static_cast<unsigned long>(i)]);
            }
        }

        //规划数据记录
        ofs.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/joint_saved.txt", ios::out);
        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            rxb_currentJointState_Controllable_DOF.j[i] = pos_jointq.j[i];
            Planing_jnt_ang.data[static_cast<unsigned long>(i)] = pos_jointq.j[i];
            ofs << pos_jointq.j[i] << endl;//保存的是当前规划的轨迹点角度值
        }
        rxb_currentJointState_Controllable_DOF.t = pos_jointq.t;
        ofs.close();
        PlaningJntPos_Pub.publish(Planing_jnt_ang);

        //闭环控制与开环控制不同,只要进入了力位混合控制后，柔性臂会直接开始
        for(int i = 0; i < Controllable_DOF_num_all; ++i) {
            //第一组驱动的pidout计算
            derror_joint_FirstGroup[i] = (pos_jointq.j[i] - g_equ_JointAngle_FirstGroup[i]) - error_joint_FirstGroup[i];//delta_deg
            error_joint_FirstGroup[i] = pos_jointq.j[i] - g_equ_JointAngle_FirstGroup[i];//deg
            ierror_joint_FirstGroup[i] = ierror_joint_FirstGroup[i] + error_joint_FirstGroup[i];
            //可控关节角角速度
            Joint_pid_out_FirstGroup[i] = Joint_pid_kp_FirstGroup[i/2] * error_joint_FirstGroup[i] / 10.0 + Joint_pid_kd_FirstGroup[i/2] * derror_joint_FirstGroup[i] / 10.0 + Joint_pid_ki_FirstGroup[i/2] * ierror_joint_FirstGroup[i] / 10.0 + pos_jointq.vel[i];
            cout << "Joint_pid_kp_FirstGroup[" << i << "] : " << Joint_pid_kp_FirstGroup[i/2] << endl;
            cout << "Joint_pid_ki_FirstGroup[" << i << "] : " << Joint_pid_ki_FirstGroup[i/2] << endl;
            cout << "Joint_pid_kd_FirstGroup[" << i << "] : " << Joint_pid_kd_FirstGroup[i/2] << endl;
            cout << "Joint_pid_out_FirstGroup[" << i << "] : " << Joint_pid_out_FirstGroup[i] << endl;
        }

        RXB_JntState_To_CableVel(g_equ_JointAngle_FirstGroup, Joint_pid_out_FirstGroup, cableL_vel_FirstGroup);//pid_out to pos_jointq.vel cable_vel : mm/s

        double max_motor_vel = DBL_MIN;
        for(int i = 0; i < Motor_num_all; i++){
            cableL_vel[i] = cableL_vel_FirstGroup[i];
            motor_vel[i] = cableL_vel[i] * g_DrivenBox_V[i];
            max_motor_vel = max(fabs(max_motor_vel), fabs(motor_vel[i]));
        }

        for(int i = 0; i < Motor_num_all; i++) {//绳索号  Motor_num_all
            Cable_num = i + 1;
            Group_Num = g_Driver_ID[Cable_num - 1] >> 4 & 0x0f;
            Driver_Num = g_Driver_ID[Cable_num - 1] - 15 * Group_Num;

            if(fabs(max_motor_vel) > fabs(V_max[0])){
              motor_vel[i] /= fabs(max_motor_vel/V_max[0]);
            }
            cout << "Motorspeed-[Cable " << Cable_num << "]:   "<< motor_vel[i] << endl;
            if(i != g_ConstForceCable_ID[0]/* && i != g_ConstForceCable_ID[1] && i != g_ConstForceCable_ID[2]*/){//不是恒力控绳索 i != g_ConstForceCable_ID[0] && i != g_ConstForceCable_ID[1]
              if(g_rxbWorkStatus==1)
                  Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,static_cast<long>(motor_vel[i]));
              else
                  Motor_Driver_Velocity_Mode(Group_Num,Driver_Num,5000,0);
            }
        }
        for(int i=0; i < DriveGroup_num*ModuJnt_num_all; i++){//DriveGroup_num*ModuJnt_num_all
            motorVel_ad2_ConstForceCable[i] = motor_vel[g_ConstForceCable_ID[i]];
        }
    }

    //机械臂随机运动
    void RxbHardwareInterface::randomMotionCallback(const std_msgs::UInt32::ConstPtr &data){
        if(data->data == 1){
            g_randomMotionFlag = g_randomMotionFlag ? false : true;
        }
    }

    //用于力感知实验
    void RxbHardwareInterface::exForceSensor_callbackFunc(const geometry_msgs::Twist::ConstPtr &data){
        if(data != nullptr){
            // 单位为N mm
            exForceMeasured.angular.x = data->angular.x * 1000;
            exForceMeasured.angular.y = data->angular.y * 1000;
            exForceMeasured.angular.z = data->angular.z * 1000;
            // 单位为N
            exForceMeasured.linear.x = data->linear.x;
            exForceMeasured.linear.y = data->linear.y;
            exForceMeasured.linear.z = data->linear.z;
        }
    }
    void RxbHardwareInterface::forceSense_callbackFunc(const rxb_msgs::forceSenseResult::ConstPtr &data){
      if(data != nullptr){
        forceSense[2*data->ID] = data->force[0];
        forceSense[2*data->ID+1] = data->force[1];
      }
    }

    //用于导纳控制
    void RxbHardwareInterface::AdmittanceControlSwitch_CallbackFunc(const std_msgs::UInt32::ConstPtr& data){
      if(data->data == 1){
        admittanceControlIsOn = admittanceControlIsOn ? false : true;
        if(admittanceControlIsOn){
          for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
            acState[i].x = 0.0;
            acState[i].v = 0.0;
          }
        }else{
          RXB_Cable_Plan.clear();
          RXB_joint_plan.clear();

          cout << "======================================================" << endl;
          cout << "======================================================" << endl;
          cout << "======================================================" << endl;
          cout << " rxb_currentJointState_Controllable_DOF: " << " ";
          for(int i = 0; i < Controllable_DOF_num_all; i++){
            cout << rxb_currentJointState_Controllable_DOF.j[i] << " ";
          }
          cout << endl;
          cout << " admi_jointq: " << " ";
          for(int i = 0; i < Controllable_DOF_num_all; i++){
            cout << admi_jointq.j[i] << " ";
          }
          cout << endl;
          cout << "======================================================" << endl;
          cout << "======================================================" << endl;
          cout << "======================================================" << endl;

          rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;//从当前角度开始规划
          p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, admi_jointq, RXB_joint_plan);
          jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
        }
      }
    }
    void RxbHardwareInterface::AdmittanceControl(double forceSense[], State state[], float M, float B, float K, double dt){
      bool biggerThanThresFlag = false;
      double tempForce = 0.0;
      for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
          tempForce += pow(forceSense[i], 2);
      }
      if(sqrt(tempForce) >= 2.5){
          biggerThanThresFlag = true;
      }

      State k1, k2, k3, k4;
      double force[Controllable_DOF_num_all] = {0};
      for(unsigned long i = 0; i < Controllable_DOF_num_all; i++){
        if(i%2 == 0){
            force[i] = -forceSense[i];
        }else{
            force[i] = forceSense[i];
        }

        if(!biggerThanThresFlag){
            force[i] = 0;
        }

//        if(fabs(force[i]) <= 2.0)
//          force[i] = 0;
//        else if(force[i] > 2.0)
//          force[i] -= 2.0;
//        else
//          force[i] += 2.0;

        k1 = MBKEquation(state[i], M, B, K, force[i]);
        k2 = MBKEquation({state[i].x + 0.5 * dt * k1.x, state[i].v + 0.5 * dt * k1.v}, M, B, K, force[i]);
        k3 = MBKEquation({state[i].x + 0.5 * dt * k2.x, state[i].v + 0.5 * dt * k2.v}, M, B, K, force[i]);
        k4 = MBKEquation({state[i].x + dt * k3.x, state[i].v + dt * k3.v}, M, B, K, force[i]);

        state[i].x = state[i].x + (dt / 6.0) * (k1.x + 2*k2.x + 2*k3.x + k4.x);
        state[i].v = state[i].v + (dt / 6.0) * (k1.v + 2*k2.v + 2*k3.v + k4.v);
      }
    }
    RxbHardwareInterface::State RxbHardwareInterface::MBKEquation(const State& state, float M, float B, float K, double F) {
        State dstate;
        dstate.v = (F - static_cast<double>(B) * state.v - static_cast<double>(K) * state.x) / static_cast<double>(M);
        dstate.x = state.v;
        return dstate;
    }
    void RxbHardwareInterface::AdmittanceControl_Seg1Set_SubFunc(const std_msgs::Float32MultiArray::ConstPtr& data){
        M = data->data[0];
        B = data->data[1];
        K = data->data[2];
    }

    // 用于神经网络训练
    void RxbHardwareInterface::NNTrainingNewRc_CallbackFunc(const std_msgs::UInt8::ConstPtr& data){
      if(data->data == 1){
        nnTrainingNewRcFlag = nnTrainingNewRcFlag ? false : true;
        if(nnTrainingNewRcFlag){
          nnTrainingStartFlag = false;
          rcCount = 0;

          ofstream oFile;
          oFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/data/nnTrainingData.txt", ios::out);
          oFile.close();
        }
      }
    }
    void RxbHardwareInterface::NNTrainingStart_CallbackFunc(const rxb_msgs::neuralNetworkCommand::ConstPtr& data){
      if(nnTrainingNewRcFlag){
        nnTrainingStartFlag = true;
        rcCount = 0;

        // 只考虑单段单一方向旋转
        if(data->revoluteSign == 0){
          extension = 0;
        }else{
          extension = data->jointID*2 + data->revoluteSign;
        }
      }
    }

    // 用于任务空间控制
    void RxbHardwareInterface::TaskSpaceCtrl_Mode_SubFunc(const std_msgs::UInt8::ConstPtr& data){
        if(data->data == 1){
            taskSpaceCtrl_mode = 1;
            taskSpaceCtrl_firstFlag = true;
        }else if(data->data == 2){
            taskSpaceCtrl_mode = 2;
        }
    }
    void RxbHardwareInterface::TaskSpaceCtrl_Move_SubFunc(const std_msgs::Float64MultiArray::ConstPtr& data){
        if(g_rxbMode == RxbMode_TaskSpace_Ctrl && taskSpaceCtrl_mode == 1){
            static jointState targetJoint;
            static jointState targetJointLast;

            if(taskSpaceCtrl_firstFlag){
              targetJoint = rxb_currentJointState_Controllable_DOF;
              taskSpaceCtrl_firstFlag = false;
            }

            for(int i = 0; i < Controllable_DOF_num_all; i++){
                targetJointLast.j[i] = targetJoint.j[i];
                targetJoint.j[i] = data->data[static_cast<unsigned long>(i)];
            }
            targetJointLast.t = targetJointLast.duration = 0;

            if(g_motorMode != MotorVelocityMode){
                g_motorMode = MotorVelocityMode;
                Motor_Driver_Reset(0, 0);
                usleep(500000);
                Motor_Driver_Mode_Choice(0, 0, Velocity_Mode);
                usleep(500000);
                g_motorMode = MotorPositionMode;
            }

            RXB_Cable_Plan.clear();
            RXB_joint_plan.clear();

            p2p_jointTrajectoryPlan(targetJointLast, targetJoint, RXB_joint_plan);
            jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
        }else if(g_rxbMode == RxbMode_TaskSpace_Ctrl && taskSpaceCtrl_mode == 2){

        }
    }

    // 补充基础功能
    void RxbHardwareInterface::ActualForwardKinematic(const double jointAngle[]){
        actualT = actualT0;
        Eigen::MatrixXd tempMatrix(4, 4);
        for(unsigned long j = 0; j < Gene_Jnt_Num; j++){
            actualT = actualT * mr::MatrixExp6(mr::VecTose3(BList[j])*(Deg2Rad*jointAngle[j]));
        }

        std_msgs::Float32MultiArray actualT_msg;
        for(long i = 0; i < 3; i++){
            for(long j = 0; j < 4; j++){
                actualT_msg.data.push_back(static_cast<float>(actualT(i, j)));
            }
        }
        actualT_pub.publish(actualT_msg);
    }
    RxbHardwareInterface::SlidingWindowSmooth::SlidingWindowSmooth(const string& name_, unsigned long dim_): name(name_), dim(dim_){
      sum.resize(dim, 0);
      result.resize(dim, 0);
    }
    void RxbHardwareInterface::SlidingWindowSmooth::AddData(const double* const data_){
      vector<double> data(dim, 0);
      for(unsigned long i = 0; i < dim; i++){
          data[i] = data_[i];
      }

      // 加入新的数据
      dq.push_back(data);
      for(unsigned long i = 0; i < dim; i++){
        sum[i] += data[i];
      }

      // 处理滑动窗口
      unsigned long dqSize = dq.size();
      if(dqSize > windowSize){
        for(unsigned long i = 0; i < dim; i++){
          sum[i] -= dq[0][i];
        }
        dq.pop_front();
        dqSize--;
      }

      // 更新结果
      for(unsigned long i = 0; i < dim; i++){
        result[i] = sum[i] / dqSize;
      }
    }
    void RxbHardwareInterface::SlidingWindowSmooth::Disp(void) const{
      cout << name << " result: ";
      for(unsigned long i = 0; i < dim; i++){
        cout << result[i] << "  ";
      }
      cout << endl;
    }
    RxbHardwareInterface::KalmanFilter::KalmanFilter(const string& name_, const unsigned long dim_, const vector<double>& Q_)
      : name(name_), dim(dim_){
      A << 1, global_T_Interval_s, 0.5*global_T_Interval_s*global_T_Interval_s,
           0, 1, global_T_Interval_s,
           0, 0, 1;
      H << 1, 0, 0;
      Q.setZero();
      Q.diagonal() << Q_[0], Q_[1], Q_[2];
      R << 1;

      x.resize(dim);
      for(unsigned long i = 0; i < dim; i++){
        x[i].setZero();
      }
      P.resize(dim);
      for(unsigned long i = 0; i < dim; i++){
        P[i].setZero();
        P[i].diagonal() << 1, 1, 1;
      }

      result.resize(dim);
    }
    void RxbHardwareInterface::KalmanFilter::Predict(void){
      for(unsigned long i = 0; i < dim; i++){
        x[i] = A * x[i];
        P[i] = A * P[i] * A.transpose() + Q;
      }
    }
    void RxbHardwareInterface::KalmanFilter::Update(const vector<double>& z_){
      for(unsigned long i = 0; i < dim; i++){
        Matrix<double, 1, 1> z;
        z << z_[i];

        MatrixXd K = P[i]*H.transpose() * (H*P[i]*H.transpose() + R).inverse();
        x[i] = x[i] + K*(z-H*x[i]);
        P[i] = (MatrixXd::Identity(3, 3) - K*H) * P[i];
        result[i] = x[i];
      }
    }
    void RxbHardwareInterface::KalmanFilter::AddData(const vector<double>& z){
      if(z.size() != dim){
        cerr << "Class KalmanFilter Funciton AddData wrong." << endl;
        return;
      }

      Predict();
      Update(z);
    }
    void RxbHardwareInterface::KalmanFilter::Disp(void) const{
      cout << name << "----------------------------------------" << endl;
      for(unsigned long i = 0; i < dim; i++){
        cout << "ID " << i << ":";
        cout << result[i].transpose();
        cout << endl;
      }
    }
    void RxbHardwareInterface::CalBodyJacobian(const vector<VectorXd>& BList, const double* const alpha){
        MatrixXd T(4, 4);
        T.setIdentity();
        for(int i = Gene_Jnt_Num-1; i >= 0; i--){
            if(i < Gene_Jnt_Num-1){
                T = T*mr::MatrixExp6(-mr::VecTose3(BList[static_cast<unsigned long>(i+1)])*(Deg2Rad*alpha[i+1]));
            }
            bodyJacobian.col(i) = mr::Adjoint(T)*BList[static_cast<unsigned long>(i)];
        }
    }
    void RxbHardwareInterface::CalBodyTwist(const MatrixXd& bodyJacobian, const vector<Vector3d>& jointAngleState){
        VectorXd dAlpha(Gene_Jnt_Num, 1);
        for(long i = 0; i < Gene_Jnt_Num; i++){
            dAlpha[i] = jointAngleState[static_cast<unsigned long>(i)](1);
        }

//        cout << "The bodyTwist is: ";
        bodyTwist = bodyJacobian * (Deg2Rad * dAlpha);
//        for(long i = 0; i < 3; i++){
//            bodyTwist[i] *= Rad2Deg;
//            cout << bodyTwist[i] << "\t";
//        }
//        cout << endl;

        std_msgs::Float32MultiArray bodyTwist_msg;
        for(long i = 0; i < 6; i++){
            bodyTwist_msg.data.push_back(static_cast<float>(bodyTwist[i]));
        }
        bodyTwist_pub.publish(bodyTwist_msg);
    }
    void RxbHardwareInterface::TrajectoryDemo_SubFunc(const rxb_msgs::trajectoryDemo::ConstPtr& data){
        ifstream iFile;
        if(data->mode == 0){  // Segmental
            switch(data->ID) {  // 读取文件
            case 1:
                ROS_INFO("Loading the trajectory1..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory1.txt", ios::in);
                break;
            case 2:
                ROS_INFO("Loading the trajectory2..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory2.txt", ios::in);
                break;
            case 3:
                ROS_INFO("Loading the trajectory3..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory3.txt", ios::in);
                break;
            case 4:
                ROS_INFO("Loading the trajectory4..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory4.txt", ios::in);
                break;
            case 5:
                ROS_INFO("Loading the trajectory5..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory5.txt", ios::in);
                break;
            case 6:
                ROS_INFO("Loading the trajectory6..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory6.txt", ios::in);
                break;
            case 7:
                ROS_INFO("Loading the trajectory7..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory7.txt", ios::in);
                break;
            case 8:
                ROS_INFO("Loading the trajectory8..........");
                iFile.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_hardware_interface/traj/trajectory8.txt", ios::in);
                break;
            default:
                break;
            }
            if (!iFile.is_open()) {
                cout << "traj file cannot open" << endl;
                return;
            }

            double iData; // 文件读取的数据
            uint8_t count = 0;  // 计数器
            bool timeFlag = false, usableFlag = true;  // 标志位
            jointState point;
            std::list<jointState> pointList;
            unsigned int pointsNum = 0;
            std::vector<double> timeGap;
            while(iFile >> iData){
                if(!timeFlag){  // 记录数据
                    if(fabs(iData-999999999) <= 0.001){
                        if(count != 0){ // 不完整的规划角
                            usableFlag = false;
                            break;
                        }
                        timeFlag = true;
                        timeGap = std::vector<double>(pointsNum-1, 0);
                        continue;
                    }
                    point.j[count] = iData;
                    if(++count == Controllable_DOF_num_all){
                        count = 0;
                        pointList.push_back(point);
                        pointsNum++;
                    }
                }else{  // 记录规划间隔时间
                    if(count >= pointsNum-1){
                        usableFlag = false;
                        break;
                    }
                    timeGap[count++] = iData;
                }
            }
            iFile.close();

            if(!usableFlag || !timeFlag){
                cerr << "Wrong file input." << endl;
                return;
            }

            // 先运行至初始点
            point = *pointList.begin();
            rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;
            RXB_joint_plan.clear();
            RXB_Cable_Plan.clear();
            p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, point, RXB_joint_plan);
            jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
            while(RXB_joint_plan.size() > 0);

            // 执行轨迹
            TrajectoryDemo_SegmentalPlan(pointList, timeGap, RXB_joint_plan);
            jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
            trajectoryDemo_flag = true;

        }else{  // Continuum

        }
    }
    void RxbHardwareInterface::TrajectoryDemo_SegmentalPlan(list<jointState>& pointList, const vector<double>& timeGap, list<jointState>& jointPlan){
        unsigned int pointsNum = static_cast<unsigned int>(timeGap.size())+1;
        list<jointState>::iterator iter = pointList.begin();
        jointState curPoint = *iter;
        for(unsigned int i = 0; i < pointsNum-1; i++){
            for(unsigned long j = 0; j < static_cast<unsigned long>(timeGap[i]/global_T_Interval_s); j++){
                curPoint.t += global_T_Interval_s;
                curPoint.duration = global_T_Interval_s;
                jointPlan.push_back(curPoint);
            }
            iter++;
            p2p_jointTrajectoryPlan(curPoint, *iter, jointPlan);
            curPoint = *iter;
        }
    }

    /// 在线神经网络训练
    RxbHardwareInterface::SlidingWindow::SlidingWindow(unsigned int dim_, unsigned int windowSize_): dim(dim_), windowSize(windowSize_){}
    void RxbHardwareInterface::SlidingWindow::AddData(const vector<double>& data){
        if(data.size() != dim){
            cerr << "Class SlidingWindow: Wrong data size." << endl;
            return;
        }
        rc.push_back(data);
        if(rc.size() > windowSize){
            rc.pop_front();
        }
    }
    vector<double> RxbHardwareInterface::SlidingWindow::GetResult(void){
        if(rc.size() < windowSize){
            return {};
        }

        vector<double> ret(dim*windowSize, 0);
        for(unsigned long i = 0; i < windowSize; i++){
            for(unsigned long j = 0; j < dim; j++){
                ret[i*dim+j] = rc[i][j];
            }
        }
        return ret;
    }
    void RxbHardwareInterface::NeuralNetwork_CorCableForceStart_SubFunc(const std_msgs::Bool::ConstPtr& data){
        if(data->data){
            neuralNetwork_corCableForce_flag = true;
//            g_randomMotionFlag = true;
        }
    }
    void RxbHardwareInterface::NeuralNetwork_CorCableForceStop_SubFunc(const std_msgs::Bool::ConstPtr& data){
        if(data->data){
            neuralNetwork_corCableForce_flag = false;
//            g_randomMotionFlag = false;
        }
    }
    void RxbHardwareInterface::NeuralNetwork_CorCableForce_DataRecord_Start_SubFunc(const std_msgs::UInt8::ConstPtr& data){
        neuralNetwork_corCableForce_dataRecord_stateFlag = data->data;

        // 清空文件
        fstream file;
        switch(neuralNetwork_corCableForce_dataRecord_stateFlag){
        case 1:
            file.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/trainSet.txt", ios::out);
            break;
        case 2:
            file.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/devSet.txt", ios::out);
            break;
        case 3:
            file.open("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/testSet.txt", ios::out);
            break;
        }

        neuralNetwork_corCableForce_dataRecord_count = 0;
    }
    void RxbHardwareInterface::NeuralNetwork_CorCableForce_DataRecord_Stop_SubFunc(const std_msgs::UInt8::ConstPtr& data){
        neuralNetwork_corCableForce_dataRecord_stateFlag = data->data;
    }
    void RxbHardwareInterface::NeuralNetwork_CorCableForce_Model_Dev_State_SubFunc(const std_msgs::Bool::ConstPtr& data){
        neuralNetwork_corCableForce_model_dev_state = data->data;
    }

    // 机械臂随机运动
    void RxbHardwareInterface::randomMotionFunc(void){
        jointState targetJoint;

        // 创建一个随机数生成器
        std::random_device rd;  // 用来获取一个随机数种子
        std::mt19937 gen(rd()); // 以随机设备rd初始化Mersenne Twister生成器

        // 定义一个随机数分布范围
        std::uniform_real_distribution<double> dis(-8.0, 8.0); // 创建一个[-8, 8]的均匀分布

        for(int i = 0; i < Controllable_DOF_num_all; i++) {
            targetJoint.j[i] = dis(gen);
        }

        rxb_currentJointState_Controllable_DOF.t = rxb_currentJointState_Controllable_DOF.duration = 0;//从当前角度开始规划
        p2p_jointTrajectoryPlan(rxb_currentJointState_Controllable_DOF, targetJoint, RXB_joint_plan);
        jointPlan2CablePlan(RXB_joint_plan, RXB_Cable_Plan);
    }

}
