#include "include.h"
#include "ADI2018.H"
/*
0x55 //数据包头
0x61 //标志位
*/

static char re_buff[30];      //接收到的原始数据

float ADI_acc_x = 0;    //加速度
float ADI_acc_y = 0;
float ADI_acc_z = 0;

float ADI_gyro_x = 0;   //角速度   
float ADI_gyro_y = 0;
float ADI_gyro_z = 0;

float ADI_pitch = 0;    //角度
float ADI_roll = 0;
float ADI_yaw = 0;

  //获取模块数据
int ADI_GetData(){
  uint8 temp_H = 0x00, temp_L = 0x00; //临时变量，存高，低8位
  
  //发送数据包，提示模块发送数据

  uart_querybuff(UART2, re_buff, 100); //读数据包
  
  if( !(re_buff[0] == 0x55 && re_buff[1] == 0x61) ) //检验帧头与FLAG是否准确
    return 0;
  
  temp_L = re_buff[2]; temp_H = re_buff[3];     //取加速度高，低位
  ADI_acc_x = ((temp_H<<8)|temp_L) / 32768 * 16 * 9.8;      //(g 为重力加速度，可取 9.8m/s 2 )
  temp_L = re_buff[4]; temp_H = re_buff[5];     
  ADI_acc_y = ((temp_H<<8)|temp_L) / 32768 * 16 * 9.8;    
  temp_L = re_buff[6]; temp_H = re_buff[7];     
  ADI_acc_z = ((temp_H<<8)|temp_L) / 32768 * 16 * 9.8;
  
  temp_L = re_buff[8]; temp_H = re_buff[9];     
  ADI_gyro_x = ((temp_H<<8)|temp_L) / 32768 * 2000;    
  temp_L = re_buff[10]; temp_H = re_buff[11];     
  ADI_gyro_y = ((temp_H<<8)|temp_L) / 32768 * 2000;  
  temp_L = re_buff[12]; temp_H = re_buff[13];     
  ADI_gyro_z = ((temp_H<<8)|temp_L) / 32768 * 2000;  

  temp_L = re_buff[14]; temp_H = re_buff[15];     
  ADI_pitch = ((temp_H<<8)|temp_L) / 32768 * 180;      
  temp_L = re_buff[16]; temp_H = re_buff[17];     
  ADI_roll =  ((temp_H<<8)|temp_L) / 32768 * 180;   
  temp_L = re_buff[18]; temp_H = re_buff[19];     
  ADI_yaw =   ((temp_H<<8)|temp_L) / 32768 * 180;
  
  return 1;
    
}