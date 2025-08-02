#ifndef __MOTOR_DRIVER_H_
#define __MOTOR_DRIVER_H_

#include "global.h"
#include "usbcan.h"
#include "rxb_msgs/sensorRead.h"
using namespace std;

#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

namespace RxbHardwareInterface {

    void  delay_ms(int ms);
    void  Motor_Driver_Reset(unsigned char Group, unsigned char Number);
    void  Motor_Driver_Mode_Choice(unsigned char Group, unsigned char Number, unsigned char Mode);
    void  Motor_Driver_Velocity_Position_Mode(unsigned char Group, unsigned char Number, short Temp_PWM, short Temp_Velocity, long Temp_Position);
    void  Motor_Driver_Velocity_Mode(unsigned char Group, unsigned char Number, short Temp_PWM, long Temp_Velocity);
    void  Motor_Driver_Current_Velocity_Mode(unsigned char Group, unsigned char Number, short Temp_Current, short Temp_Velocity);
// void  Motor_Driver_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);
    void  Motor_Driver_Config(unsigned char Group, unsigned char Number, unsigned char Temp_Time, unsigned char Ctl1_Ctl2);
    void  Motor_Driver_Online_Check(unsigned char Group, unsigned char Number);

    void getAllMotPos();
    void  Mot_Read_Pos(unsigned char Group, unsigned char Number, unsigned char Temp_Time, unsigned char Ctl1_Ctl2);
    void MotorPosReceive(unsigned char id, unsigned char data[]);
    void SensorDataDecode(UINT id, unsigned char data[]);
    void SensorDataGet(UCHAR seg);
    void SensorDataReveive(uint32_t id, unsigned char data[]);
    u8 DirverID2Motnum(u8 DirverID );

    int findElementIndex(const u8 arr[], int size, int target);//找到数组指定元素的下标

}
#endif


