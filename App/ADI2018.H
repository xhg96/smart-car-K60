#ifndef __ADI2018_H__
#define __ADI2018_H__


extern float ADI_acc_x;    //���ٶ�
extern float ADI_acc_y;
extern float ADI_acc_z;

extern float ADI_gyro_x;   //���ٶ�   
extern float ADI_gyro_y;
extern float ADI_gyro_z;

extern float ADI_pitch;    //�Ƕ�
extern float ADI_roll;
extern float ADI_yaw;

int ADI_GetData();

#endif