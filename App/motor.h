/*���ͷ�ļ�*/
#ifndef _MOTOR_H
#define _MOTOR_H

#include "include.h"
#include "control.h"
#include "fuzzy_pid.h"

typedef enum{

	NO_INIT = 0,
	START_INIT,
        
        DELAY_START,
        DELAY_TIME_UP, 
        
        DMP_READ,
	
}DMPDELAY_e;    //dmp��ʱ��־�����

extern DMPDELAY_e DMP_Delay_Status;
extern int speed_type;
//extern int aim_speed;
//extern int now_speed;

extern uint16 max_speed;  
extern uint16 max2_speed;

extern uint16 encoderThreshold;
extern uint16 bumpEncoderThreshold;
extern uint16 RuningTime;   
extern int StartDelayCount;        //������ʱ������
extern int StopDelayCount;
extern uint8 stopLineDetected;
extern uint8 StopLineNoDetect;
extern uint8 stopRemember;

extern uint16 annulus_speed;     //Բ���ٶȲ���
extern uint16 ann_speed_k;

//����ٶȱ���
extern uint16 wan_speed;         //���ι�ʽ�ĳ�����
extern uint16 min_speed;         //�����С�ٶ�
extern uint16 cwrstop_speed;       //��������ٶ��������
extern uint16 dwrstop_speed;       //��������ٶ��������
extern uint16 ann_min_speed;
//�����ٶȱ���
extern uint16 base_speed;        //��������ٶ�
extern uint16 outm_speed;        //������ٵ�����ٶ�
extern uint16 add_speed;//4        //����
extern uint16 dec_add_flag;

//ɲ���ٶȱ���
extern uint16 extraStopVal_Short;       //��������ٶ��������
extern uint16 extraStopVal_Turnin;
extern uint16 extraStopVal_Long; 
extern uint16 speed_k;
extern uint16 speed_k2;

extern uint16 bra_speed1 ;
extern uint16 bra_speed2 ;
extern uint16 bra_speed3 ;
extern uint16 bra_speed4 ;

//�����ٶȱ���
extern uint16 ra_speed;         //�µ��ٶ�
extern uint16 obcutsp;          //�ϰ��������ٶ�
extern uint16 max2_speed;        //���ι�ʽ������ٶ�
extern uint16 bump_speed;
//�����жϱ���
extern uint16 zhidao_ade;         //��ֱ�����ж�
extern uint16 wan_ade;            //������ٵ��ж�
extern uint16 set_yh_top;

//ɲ���жϱ���
extern uint16 dzhi_top;           //ֱ��ɲ�����ж�
extern uint16 set_othertop;       //��һ���˵�����
extern uint16 set_cothertop;      //��һ���˵�����
extern uint16 wanru_ade;          //������ٵ��ж�

void Track_type_detect();
void Speed_Set();
void sendSpeedInfo();
void Car_go();
void squareFomulTest();
int16 encoderValLimit(int16 raw_val_now ,int16 raw_val_last);

void Normal_Driving();
void Timing_Driving();
void testDriving();

int StartDelay_s(int seconds);
int StopDelay_10ms(int mil_seconds);
#endif          //_MOTOR_H