#include "common.h"
#include "MK60_FTM.h"
#include "m_Sevro.h"



/* ȫ�ֱ��� */
PositionalPID_st Servo_PID;   //�������PID�ṹ�� 

uint16 ServoMidPWM_CNV = 2930;    //����������ʱ��FTM_CNV�Ĵ���ֵ(����Ƶ��125M��ps=6,mod=39062) 1:2939, 2:2919
                                                               //(����Ƶ��100M��ps=5,mod=62500) 1:4697, 2:4670
uint16 MID_PWM = 5255;
/******************************************************************************
 *      
 *     �������ܣ�PID�ṹ���ڲ�������ʼ��
 *     ������ 
 *     ����ֵ��
******************************************************************************/
void Servo_Init()
{
    Servo_PID.error = 0;
    Servo_PID.preError = 0;
  
    Servo_PID.Kp = 100;
    Servo_PID.Kd = 0;
    
    Servo_PID.PID_Out = 0;

    ftm_pwm_init(SD5_FTM, SD5_CH, SD5_HZ, MID_PWM );

}

/******************************************************************************
 *      
 *     �������ܣ�λ��ʽPD���������PWMֵ
 *     �������ṹ�� PIDS ָ�� 
 *     ����ֵ�����PWMֵ
******************************************************************************/
int16 Calc_ServoPID(PositionalPID_st *pp)
{
    float kp,kd;
    int16 error,d_error;
    
    error = pp->error;      			   //e(k)
    d_error = pp->error - pp->preError;    //e(k)-e(k-1)
    
    pp->preError = error;
	
    kp = 1;
    kd = (float)pp->Kd / 100;
	
    pp->PID_Out = (int16)(kp * error + kd * d_error);
    
    
    if(pp->PID_Out > SERVOPWM_MAX)
        pp->PID_Out = SERVOPWM_MAX;
    if(pp->PID_Out < SERVOPWM_MIN)
        pp->PID_Out = SERVOPWM_MIN;
    
    return (pp->PID_Out);
}


