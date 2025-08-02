#include "global.h"
#include "motor_driver.h"
#include <math.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include "unistd.h"
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <ros/ros.h>
#include "controlcan.h"
#include "rxb_hardware_interface.h"
namespace RxbHardwareInterface {

    void delay_ms(int ms) {
        usleep(1000 * ms);
    }


    void Motor_Driver_Reset(UCHAR Group, UCHAR Number) {
        //需要发送的帧，结构体设置
        UINT canid = 0x000;
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;

        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;
        for (int i = 0; i < send[0].DataLen; i++) {
            send[0].Data[i] = 0x55;
        }
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) { //由CAN1发送

        }
    }

    void Motor_Driver_Mode_Choice(UCHAR Group, UCHAR Number, UCHAR mode) {
        UINT canid = 0x001;
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;

        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8; //<< is priority
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;
        send[0].Data[0] = mode;
        for (int i = 1; i < send[0].DataLen; i++) {
            send[0].Data[i] = 0x55;
        }
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
            if(mode == Current_Velocity_Mode) {
                cout << "MOTOR Is Velocity_Mode!" << endl;
                g_motorMode = MotorVelocityMode;
            }
            if(mode == Current_Velocity_Position_Mode) {
                cout << "MOTOR Is Position_MOde!" << endl;
                g_motorMode = MotorPositionMode;
            }
        }
    }

    void Motor_Driver_Config(UCHAR Group, UCHAR Number, UCHAR Temp_Time1, UCHAR Temp_Time2) {
        unsigned short canid = 0x00A;   // time1:0~255(为0，关闭电流速度位置反馈)  time2:0~255（为0，关闭限位信号反馈）
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;
        send[0].Data[0] = Temp_Time1;
        send[0].Data[1] = Temp_Time2;
        send[0].Data[2] = 0x55;
        send[0].Data[3] = 0x55;
        send[0].Data[4] = 0x55;
        send[0].Data[5] = 0x55;
        send[0].Data[6] = 0x55;
        send[0].Data[7] = 0x55;
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
        }
    }

    void Motor_Driver_Current_Velocity_Mode(UCHAR Group, UCHAR Number, short Temp_Current, short Temp_Velocity) {
        unsigned short canid = 0x007;   //current:0~32767mA  velocity:-32767~+32767RPM
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;
        if(Temp_Current < 0) {
            Temp_Current = abs(Temp_Current);
        }
        send[0].Data[0] = (UCHAR)((Temp_Current >> 8) & 0xff);
        send[0].Data[1] = (UCHAR)(Temp_Current & 0xff);
        send[0].Data[2] = (UCHAR)((Temp_Velocity >> 8) & 0xff);
        send[0].Data[3] = (UCHAR)(Temp_Velocity & 0xff);
        send[0].Data[4] = 0x55;
        send[0].Data[5] = 0x55;
        send[0].Data[6] = 0x55;
        send[0].Data[7] = 0x55;
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
            //  cout <<"Velocity_Mode:"<<Temp_Current<<"mA"<<Temp_Velocity<<"RPM"<<endl;
        }
    }

    void Motor_Driver_Current_Velocity_Position_Mode(UCHAR Group, UCHAR Number, short Temp_Current, short Temp_Velocity, long Temp_Position) {
        unsigned short canid = 0x009;   //current:0~32767mA  velocity:0~+32767RPM  Position:-2147483648~+2147483648 qc
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;
        if(Temp_Current < 0) {
            Temp_Current = abs(Temp_Current);
        }
        if(Temp_Velocity < 0) {
            Temp_Current = abs(Temp_Velocity);
        }
        send[0].Data[0] = (UCHAR)((Temp_Current >> 8) & 0xff);
        send[0].Data[1] = (UCHAR)(Temp_Current & 0xff);
        send[0].Data[2] = (UCHAR)((Temp_Velocity >> 8) & 0xff);
        send[0].Data[3] = (UCHAR)(Temp_Velocity & 0xff);
        send[0].Data[4] = (UCHAR)((Temp_Velocity >> 24) & 0xff);
        send[0].Data[5] = (UCHAR)((Temp_Velocity >> 16) & 0xff);
        send[0].Data[6] = (UCHAR)((Temp_Velocity >> 8) & 0xff);
        send[0].Data[7] = (UCHAR)(Temp_Velocity & 0xff);
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
            //  cout <<"Position_Mode:"<<Temp_Current<<"mA"<<Temp_Velocity<<"RPM"<<Temp_Position<<"qc"<<endl;
        }
    }

    void Motor_Driver_Online_Check(unsigned char Group, unsigned char Number) {
        unsigned short canid = 0x00F;
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;

        send[0].Data[0] = 0x55;
        send[0].Data[1] = 0x55;
        send[0].Data[2] = 0x55;
        send[0].Data[3] = 0x55;
        send[0].Data[4] = 0x55;
        send[0].Data[5] = 0x55;
        send[0].Data[6] = 0x55;
        send[0].Data[7] = 0x55;
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
            cout << "Motor_Driver_Online_Check" << endl;
        }
    }

    void Motor_Driver_Num_Back(u16 can_id, u16 *Num) {
        //motor driver's canid is [group number mode],12bits
        Num[1] = can_id;
        Num[1] = (Num[1] >> 4) & 0x0f; //number
        Num[0] = can_id >> 8; //group
    }

    u8 DirverID2Motnum(u8 DirverID ) {
        u8 i;

        for (i = 1; i <= Motor_num_all; i++)
            if(g_Driver_ID[i - 1] == DirverID) {
                return i;
                break;
            }
    }

    /*
    USED
    */
    void  Motor_Driver_Velocity_Position_Mode(unsigned char Group, unsigned char Number, short Temp_PWM, short Temp_Velocity, long Temp_Position) {
        unsigned short canid = 0x006;
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;

        if(Temp_PWM > 5000) {
            Temp_PWM = 5000;
        } else if(Temp_PWM < -5000) {
            Temp_PWM = -5000;
        }
        if(Temp_PWM < 0) {
            Temp_PWM = abs(Temp_PWM);
        }
        if(Temp_Velocity < 0) {
            Temp_Velocity = abs(Temp_Velocity);
        }

        send[0].Data[0] = (unsigned char)((Temp_PWM >> 8) & 0xff);
        send[0].Data[1] = (unsigned char)(Temp_PWM & 0xff);
        send[0].Data[2] = (unsigned char)((Temp_Velocity >> 8) & 0xff);
        send[0].Data[3] = (unsigned char)(Temp_Velocity & 0xff);
        send[0].Data[4] = (unsigned char)((Temp_Position >> 24) & 0xff);
        send[0].Data[5] = (unsigned char)((Temp_Position >> 16) & 0xff);
        send[0].Data[6] = (unsigned char)((Temp_Position >> 8) & 0xff);
        send[0].Data[7] = (unsigned char)(Temp_Position & 0xff);
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
            //  cout <<"Motor_Driver_Velocity_Position_Mode"<<endl;
        }
    }

    /*
    USED
    */
    void  Motor_Driver_Velocity_Mode(unsigned char Group, unsigned char Number, short Temp_PWM, long Temp_Velocity) {
        unsigned short canid = 0x004;
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x000;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        if ((Group <= 7) && (Number <= 15)) {
            canid |= Group << 8;
            canid |= Number << 4;
        } else {
            return;
        }
        send[0].ID = canid;

        if(Temp_PWM > 5000) {
            Temp_PWM = 5000;
        } else if(Temp_PWM < -5000) {
            Temp_PWM = -5000;
        }
        if(Temp_PWM < 0) {
            Temp_PWM = abs(Temp_PWM);
        }


        send[0].Data[0] = (unsigned char)((Temp_PWM >> 8) & 0xff);
        send[0].Data[1] = (unsigned char)(Temp_PWM & 0xff);
        send[0].Data[2] = (unsigned char)((Temp_Velocity >> 8) & 0xff);
        send[0].Data[3] = (unsigned char)(Temp_Velocity & 0xff);
        send[0].Data[4] = 0x55;
        send[0].Data[5] = 0x55;
        send[0].Data[6] = 0x55;
        send[0].Data[7] = 0x55;
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) {
            //  cout <<"Motor_Driver_Velocity_Position_Mode"<<endl;
        }
    }

    void MotorPosReceive(unsigned char id, unsigned char data[]) {
        u16 GroupNum[2];
        u8 num;
        int i;
        Motor_Driver_Num_Back(id, GroupNum);//电机号解码
        num = 15 * GroupNum[0] + GroupNum[1];
        i = DirverID2Motnum(num);//转换为对应的绳索号码 0-Motor_num_all
        // std::cout << "get " << i << " motor_feedback" << std::endl;
//        if(g_InitMotorFlag == 0) {
//            g_Init_MotPos[i - 1] = (double)((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7])) / g_DrivenBox[i - 1]; //motor初始位置
//            //  printf("motor%d:  init_pos:%lf\n\r", i, g_Init_MotPos[i - 1]);
//            std::cout << "motor" << i << ": init_pos:" << g_Init_MotPos[i - 1] << std::endl;
//            g_InitMotorFlag = 1;
//        } else {
        //     std::cout << "data: " << (int)data[4] << " " << (int)data[5] << " " << (int)data[6] << " " << (int)data[7] << " " << std::endl;
        if(i>0&&i<=Motor_num_all){
          g_motor_cur[i - 1] = (data[0] << 8) | (data[1]);
          g_motor_vel[i - 1] = (float)((data[2] << 8) | (data[3])) / g_DrivenBox_V[i-1];
          g_cable_pos_feedback[i - 1] = (double)((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7])) / g_DrivenBox[i - 1] - g_Init_CablePos[i - 1];//2022.7.24改为+
          //因为减去了初始值，所以相当于是绳长的相对变化量
          //printf("recieve->motor%d:  cur:%f   vel:%f  pos:%lf\n\r",i,g_motor_cur[i-1],g_motor_vel[i-1],g_motor_pos[i-1]);
          //g_mot_feedback_flag[i - 1] = true;
        }
        //  std::cout << "motor" << i << ": curr_pos:" << g_motor_pos[i - 1] << std::endl;
        //}
    }

    void SensorDataReveive(uint32_t id, unsigned char data[]){//实际得到的g_jointAngle排序与关节顺序一致，即YPPY-YPPY-YPPY-YP-PYYP-PYYP-PYYP-PYYP

        double Joint_feedback[4];
        int seg = 0;//采集卡序号
        u8 captureCard_ID = 0;//采集卡ID
        int i = 0;
        const static int angleEncorder_Inv[Gene_Jnt_Num] = {0, 1, 1, 1, 1, 0, 0, 0};//需要纠正臂段角度编码器的正方向，1为需要反向的关节，0为方向正确的关节

        //三种采集卡采集顺序
        const static int construction_YPPY = 0;
        const static int construction_YP = 1;
        const static int construction_PYYP = 2;
        //采集卡采集顺序对应的关节数
        const static int num_Joint_Of_Construction_YPPY = 4;
        const static int num_Joint_Of_Construction_YP = 2;
        const static int num_Joint_Of_Construction_PYYP = 4;
        //整个柔性臂上采集卡构造
        int construction[Joint_CaptureCard_Num] = {construction_YPPY, construction_YPPY};
        int num_Of_Previous_Joint = 0;

        for (i = 0; i < 4; i++){//将获取的字符数组进行解码，一般而言，一个采集卡能够采集四个角度编码器信号，每个编码器信号用2个字节的空间储存（即两个字符空间）
            Joint_feedback[i] = (double)((data[2*i] << 8)|(data[2*i+1]))/100;
        }

        captureCard_ID = (id>>11 &0x0FF);//对应采集卡ID

        //根据采集卡ID captureCard_ID 得到 g_CaptureCard_ID 下标，赋值到采集卡序号 seg (从1开始）
        if((seg = findElementIndex(g_CaptureCard_ID, Joint_CaptureCard_Num, captureCard_ID)) == -1){
            return;
        };

        for (i = 0; i < seg-1; i++){//计算前置关节的数量，以确定数组的index
            switch (construction[i]){
            case construction_YPPY:
                num_Of_Previous_Joint += num_Joint_Of_Construction_YPPY;
                break;
            case construction_YP:
                num_Of_Previous_Joint += num_Joint_Of_Construction_YP;
                break;
            case construction_PYYP:
                num_Of_Previous_Joint += num_Joint_Of_Construction_PYYP;
                break;
            }
        }

        if (construction[seg-1] == construction_YPPY){
            g_JointAngle_Absolute[num_Of_Previous_Joint + 1 - 1] = Joint_feedback[0];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 2 - 1] = Joint_feedback[1];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 3 - 1] = Joint_feedback[2];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 4 - 1] = Joint_feedback[3];
        }else if(construction[seg-1] == construction_YP){
            g_JointAngle_Absolute[num_Of_Previous_Joint + 1 - 1] = Joint_feedback[0];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 2 - 1] = Joint_feedback[1];
        }else if(construction[seg-1] == construction_PYYP){
            g_JointAngle_Absolute[num_Of_Previous_Joint + 1 - 1] = Joint_feedback[0];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 2 - 1] = Joint_feedback[1];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 3 - 1] = Joint_feedback[2];
            g_JointAngle_Absolute[num_Of_Previous_Joint + 4 - 1] = Joint_feedback[3];
        }

        //这段程序你可能会觉得很奇怪，为什么不给g_jointAngle赋值后再根据angleEncorder_Inv给它掉转正负号，实际上一开始就是这么写的。
        //但是由于程序是多线程的，在主线程中会不停地调用g_jointAngle给ui界面传递数值，所以不能给g_jointAngle赋值一个中间值
        for (i = 0; i < Gene_Jnt_Num; i++){
            if (g_JointAngle_Absolute[i] - g_JointAngle_Zero[i] >= 50){
                if (angleEncorder_Inv[i] == 0)
                    g_jointAngle[i] = g_JointAngle_Absolute[i] - g_JointAngle_Zero[i] - 360;
                else
                    g_jointAngle[i] = -(g_JointAngle_Absolute[i] - g_JointAngle_Zero[i] - 360);
            }
            else if (g_JointAngle_Absolute[i] - g_JointAngle_Zero[i] <= -50){
                if (angleEncorder_Inv[i] == 0)
                    g_jointAngle[i] = g_JointAngle_Absolute[i] - g_JointAngle_Zero[i] + 360;
                else
                    g_jointAngle[i] = -(g_JointAngle_Absolute[i] - g_JointAngle_Zero[i] + 360);
            }
            else{
                if (angleEncorder_Inv[i] == 0)
                    g_jointAngle[i] = g_JointAngle_Absolute[i] - g_JointAngle_Zero[i];
                else
                    g_jointAngle[i] = -(g_JointAngle_Absolute[i] - g_JointAngle_Zero[i]);
            }
        }
   }

    /*void SensorDataReveive(uint32_t id, unsigned char data[]){

        double Joint_feedback[100];
        double index[] = {0,1,2,7};//取负值的序号
        int16_t tex = 0;
        int16_t seg = 0;
        //cout << "slsl" << rec2len << endl;
        //printf("id: %08x , data: %016x .，%d\n",rec2[i].ID,rec2[i].Data,sizeof (rec2[i].Data));
        //printf("%x   %x ",rec2[0].Data[0] , (rec2[0].Data[0] << 8) | (rec2[0].Data[1]));
        //printf("d:%x\n",(rec2[0].ID>>20 &0x00F));

        tex = (id>>20 &0x00F);//对应第几组
        seg = (id>>11 &0x00F);//对应第几块板
        for (int i=0;i<4;i++) {
            Joint_feedback[8*(seg-1)+4*(tex-1)+i] = double(((data[2*i] << 8) | (data[2*i+1])))/100;//内置式编码器只需要将所有的8*(seg-1)+4*(tex-1)+i第一个8改成4
            if(Joint_feedback[8*(seg-1)+4*(tex-1)+i] >= 180)
            {
                Joint_feedback[8*(seg-1)+4*(tex-1)+i] = -1*(360-Joint_feedback[8*(seg-1)+4*(tex-1)+i]);
            }
            for (int t = 0;t < sizeof (index)/sizeof (double);t++) {
                if((8*(seg-1)+4*(tex-1)+i)%8 == index[t])//序号值对上了
                {
                    Joint_feedback[8*(seg-1)+4*(tex-1)+i] = -1*Joint_feedback[8*(seg-1)+4*(tex-1)+i];
                }
            }
            g_jointAngle[8*(seg-1)+4*(tex-1)+i] = -1*Joint_feedback[8*(seg-1)+4*(tex-1)+i];
        }
        for (int i=0;i<Controllable_DOF_num_all;i++) {
            if(i%2 == 0){
                g_PlanjointAngle_nouse[i] = (g_jointAngle[8*(i/2)+3]+g_jointAngle[8*(i/2)+4])/2;
            }
            else {
                g_PlanjointAngle_nouse[i] = (g_jointAngle[8*((i-1)/2)+2] + g_jointAngle[8*((i-1)/2)+5])/2;
            }

        }

        //cout << g_encoder_data[0]  <<endl;
        //cout << g_encoder_data[1]  <<endl;
        //cout << g_encoder_data[2]  <<endl;
        //cout << g_encoder_data[3]  <<endl;
   }*/

//获取传感器（关节编码器数据）
    void SensorDataGet(UCHAR seg) {
        /*
         * 输入：seg,关节角度采集卡ID号
         */

        //需要发送的帧，结构体设置
        VCI_CAN_OBJ send[1];
        send[0].ID = 0x001007F8;
        send[0].SendType = 1;
        send[0].RemoteFlag = 0; // 0数据帧 1远程证
        send[0].ExternFlag = 1; // 0标准帧（11位） 扩展帧（29位）
        send[0].DataLen = 8;
        send[0].ID = send[0].ID | (g_CaptureCard_ID[seg-1] << 11);
        //cout << "2" << hex << send[0].ID << endl;
        for (int i = 0; i < send[0].DataLen; i++) {
            send[0].Data[i] = 0x00;
        }
        //cout << "1" << endl;
        if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1) { //由CAN2发送
            //cout << "2" << endl;
            //SensorDataReveive();
        }
        //return ;
    }

    int findElementIndex(const u8 arr[], int size, int target) {
        for (int i = 0; i < size; ++i) {
            if (arr[i] == target) {
                return (i+1); // 找到目标元素，返回下标
            }
        }
        return -1; // 未找到目标元素，返回-1
    }

}
