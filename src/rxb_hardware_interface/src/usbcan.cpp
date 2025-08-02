#include "usbcan.h"
#include "motor_driver.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;
namespace RxbHardwareInterface {

    void *receive_func(void *param) { //反馈函数
        int rec1len = 0, rec2len = 0;
        VCI_CAN_OBJ rec1[3000];//接收缓存，设为3000为佳。
        VCI_CAN_OBJ rec2[3000];
        int i, j;
        int *run = (int *)param; //线程启动，退出控制。
        int can1 = 0;
        int can2 = 1; //CAN2的接受反馈
        while((*run) & 0x0f) {
            if((rec1len = VCI_Receive(VCI_USBCAN2, 0, can1, rec1, 3000, 100)) > 0) { //调用接收函数，如果有数据，进行数据处理显示。设备类型，设备索引，can线路,数据首地址，100正常
                for(j = 0; j < rec1len; j++) {
                    MotorPosReceive(rec1[j].ID, rec1[j].Data);
                }
            }
        }
        return nullptr;
    }

    void *receive_func2(void *param) { //反馈函数
        int rec1len = 0, rec2len = 0;
        VCI_CAN_OBJ rec2[3000];//接收缓存，设为3000为佳。

        int i, j;
        int *run = (int *)param; //线程启动，退出控制。
        int can2 = 1; //CAN2的接受反馈

        //测试线程读取数据速度，要用就取消注释
//        auto start = std::chrono::high_resolution_clock::now();
//        auto end = std::chrono::high_resolution_clock::now();

        while((*run) & 0x0f) {

            if((rec2len = VCI_Receive(VCI_USBCAN2, 0, can2, rec2, 3000, 0)) > 0) { //can2接受反馈

              //测试线程读取数据速度，要用就取消注释
//              start = std::chrono::high_resolution_clock::now();

              for(j = 0; j < rec2len; j++) {
                    SensorDataReveive(rec2[j].ID, rec2[j].Data);
              }

              //测试线程读取数据速度，要用就取消注释
//              end = std::chrono::high_resolution_clock::now();
//              auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//              std::cout << "程序执行时间: "<< duration.count() << "微秒" << std::endl;
//              std::cout << std::endl;
            }
        }
        return nullptr;
    }


    int m_run0;
    int m_run1;
    pthread_t threadid;
    pthread_t threadid2;

    void can_init(UCHAR Timing0, UCHAR Timing1) {
        VCI_BOARD_INFO pInfo;//用来获取设备信息。
        VCI_BOARD_INFO pInfo1 [50];
        int num = 0;

        cout << ">>Init teh can device!\r\n" << endl; //指示程序已运行
        num = VCI_FindUsbDevice2(pInfo1);  //寻找USB接口
        cout << ">>USBCAN DEVICE NUM:" << num << "\n" << endl;

        if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1) { //打开设备
            cout << ">>open 2 deivce success!\n" << endl; //打开设备成功
        } else {
            cout << ">>open deivce error!\n" << endl;
            exit(1);
        }
        if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) { //读取设备序列号、版本等信息。
            printf(">>Get VCI_ReadBoardInfo success!\n");
        } else {
            printf(">>Get VCI_ReadBoardInfo error!\n");
            exit(1);
        }

        //初始化参数
        VCI_INIT_CONFIG config;
        VCI_INIT_CONFIG config2;

        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;        //接收所有帧
        config.Timing0 = Timing0; /*波特率1000 Kbps  0x00  0x14*//*波特率125 Kbps  0x03  0x1C*/
        config.Timing1 = Timing1;
        config.Mode = 0; //正常模式

        config2.AccCode = 0;
        config2.AccMask = 0xFFFFFFFF;
        config2.Filter = 1;        //接收所有帧
        config2.Timing0 = Timing0; /*波特率1000 Kbps  0x00  0x14*//*波特率125 Kbps  0x03  0x1C*/
        config2.Timing1 = Timing1;
        config2.Mode = 0; //正常模式

        if (VCI_InitCAN(VCI_USBCAN1, 0, 0, &config) != 1) {
            cout << ">>Init CAN1 error\n" << endl;
            VCI_CloseDevice(VCI_USBCAN1, 0);
        }
        if (VCI_StartCAN(VCI_USBCAN1, 0, 0) != 1) {
            cout << ">>Start CAN1 error\n" << endl;
            VCI_CloseDevice(VCI_USBCAN1, 0);
        }
        printf("CAN1 INIT SUCCEESS!\n");


        if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config2) != 1) {
            cout << ">>Init CAN2 error\n" << endl;
            VCI_CloseDevice(VCI_USBCAN2, 0);
        }
        if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1) {
            cout << ">>Start CAN2 error\n" << endl;
            VCI_CloseDevice(VCI_USBCAN2, 0);
        }
        printf("CAN2 INIT SUCCEESS!\n");

        m_run0 = 1;
        m_run1 = 1;
        int ret,ret2;

        //如果需要用到电机数据就取消注释
//        ret = pthread_create(&threadid, NULL, receive_func, &m_run0);//接收电机驱动器传回的信息
        ret2 = pthread_create(&threadid2, NULL, receive_func2, &m_run1);//接收角度编码器传回的信息
        printf("open can receive function!\n");
    }

    void pthread() {
        //delay_ms(10000);
        m_run0 = 0;//线程关闭指令。
        pthread_join(threadid, NULL); //等待线程关闭。
        m_run1 = 0;//线程关闭指令。
        pthread_join(threadid2, NULL); //等待线程关闭。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
        usleep(100000);//延时100ms。
    }
    void can_close() {
        pthread();
        VCI_CloseDevice(VCI_USBCAN2, 0);
        cout << "close the motor device" << endl;
    }


}




