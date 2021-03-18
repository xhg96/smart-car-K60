#ifndef __ADI2018_H__
#define __ADI2018_H__


extern float ADI_acc_x;    //加速度
extern float ADI_acc_y;
extern float ADI_acc_z;

extern float ADI_gyro_x;   //角速度   
extern float ADI_gyro_y;
extern float ADI_gyro_z;

extern float ADI_pitch;    //角度
extern float ADI_roll;
extern float ADI_yaw;

int ADI_GetData();

#endif