#ifndef _CONTROL_H
#define _CONTROL_H
#include "deal_img.h"
#ifndef BOOM3_QT_DEBUG
#include "motor.h"
#endif
/*******�궨��*****************/
#define SD5_FTM  FTM1
#define SD5_CH   FTM_CH1
#define SD5_HZ   300

//#define SERVOPWM_MAX   1000     //������PWMֵ
//#define SERVOPWM_MIN   -1000   //�Ҳ����PWMֵ
extern uint16 SERVOPWM_MAX;
extern uint16 SERVOPWM_MIN;

/*******ö��******************/
typedef struct{
    uint8 top;
    uint8 othertop;
    uint8 av_de;
}ROAD_STATUS;
typedef struct{
  uint16 k;
  uint16 b;//3.7
  uint16 left_k;
  uint16 right_k;
}CONTROL_PARAM;
typedef struct{

    int16 error;
    int16 preError;
    int16 prepreError;

    uint16 Kp;
    uint16 Kdin;
    uint16 Kdout;
    int16 PID_Out;            //������PWMֵ(�����)

}PositionalPID_st;
/****����******/
void Servo_Init();
void turn();
void get_n1n2();
float average_de(uint8 src[][XM], uint8 width[YM],char flag);
void getline(uint8 src[][XM], uint8 width[YM], float kb[2], uint8 flag);
int16 Calc_ServoPID(PositionalPID_st *pp);
void getTop();
  //�����ⲿ����
extern CONTROL_PARAM CP;
extern uint16 MID_PWM;
extern PositionalPID_st Servo_PID;
extern float kb_send[2];
extern float Dout_part;
extern float Din_part;
extern uint16 Ann_k;
extern uint16 taw;
#endif          //_CONTROL_H
