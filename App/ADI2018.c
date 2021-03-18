#include "include.h"
#include "ADI2018.H"
/*
0x55 //���ݰ�ͷ
0x61 //��־λ
*/

static char re_buff[30];      //���յ���ԭʼ����

float ADI_acc_x = 0;    //���ٶ�
float ADI_acc_y = 0;
float ADI_acc_z = 0;

float ADI_gyro_x = 0;   //���ٶ�   
float ADI_gyro_y = 0;
float ADI_gyro_z = 0;

float ADI_pitch = 0;    //�Ƕ�
float ADI_roll = 0;
float ADI_yaw = 0;

  //��ȡģ������
int ADI_GetData(){
  uint8 temp_H = 0x00, temp_L = 0x00; //��ʱ��������ߣ���8λ
  
  //�������ݰ�����ʾģ�鷢������

  uart_querybuff(UART2, re_buff, 100); //�����ݰ�
  
  if( !(re_buff[0] == 0x55 && re_buff[1] == 0x61) ) //����֡ͷ��FLAG�Ƿ�׼ȷ
    return 0;
  
  temp_L = re_buff[2]; temp_H = re_buff[3];     //ȡ���ٶȸߣ���λ
  ADI_acc_x = ((temp_H<<8)|temp_L) / 32768 * 16 * 9.8;      //(g Ϊ�������ٶȣ���ȡ 9.8m/s 2 )
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