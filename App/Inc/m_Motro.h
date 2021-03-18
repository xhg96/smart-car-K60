/*��
*    @file      ServoCtrl.h" 
*    @brief    �������ͷ�ļ�
*/

#ifndef __M_MOTRO_H__
#define __M_MOTRO_H__

#define CAR_NUM 2

#define NEW 1
#define OLD 2

#define MOTOR_EN    PTD8
#define MOTOR_FTM   FTM0

#if CAR_NUM==NEW
#define MOTOR_PWM3_CH  FTM_CH7
#define MOTOR_PWM4_CH  FTM_CH6
#else
#define MOTOR_PWM3_CH  FTM_CH7
#define MOTOR_PWM4_CH  FTM_CH6
#endif
#define MOTOR_HZ    10*1000         //10K Hz(100M busƵ�� ��ӦmodֵΪ10000)

#define QUAD_FTM  FTM2

#define ENABLE_MOTOR PTD8_OUT = 1
#define DISABLE_MOTOR PTD8_OUT = 0

typedef struct {

	float var[3];
        int16 speed;
	int16 nowSpeed;       
	int16 aimSpeed;	    
	int16 MotroPWM;    
        float AC_P;
        int16 bmqSpeed[3];
	
}SPEEDInfo_st;

typedef enum{

	DISABLE,        //��ֹ���Ź�
	ENABLE,         //ʹ��
        ENABLE_OPERATED,       //��ʹ�ܣ������ظ�����
        DISABLE_OPERATED,  
	
}WatchDog_e;

/* С������ģʽö������ */
typedef enum{

	TIMING_SPEED_RUN,
	NORMAL_RUN,
        TIMING_RUN,
        QUADRA_RUN,     //���ι�ʽ
	
}RUNMode_e;

/* �������PID�ṹ�� */
typedef struct {
    
    int16 REF;
    int16 FeedBack;
    int16 FeedBackSave[3];
    
    int16 PreError;
    int16 PrePreError;
    
    uint16 Kp;
    uint16 Ki;
    uint16 Kd;
    
    int16 PID_Out;      
    
}IncrementalPID_st;

/* �ⲿ�������� */
extern IncrementalPID_st Motro_PID;
extern SPEEDInfo_st SPD;
extern RUNMode_e RunMode;
extern int16 D_Error;
extern uint16 change_ki;        
extern uint16 change_kib; 
extern uint8 stallProtectFlag;
extern float V_battery;
extern uint16 turnILimitsub;
/* ���ʹ�� */
#define  MotroEnable(n)  PTXn_T(MOTOR_EN,OUT) = n

extern void SpeedInfo_Init();
extern void Motro_Init();

/* ������� */
extern void Motro_Shifting();
extern void StopMotro();

/* ��ȡ������ */
extern void GetEncoder();
void stopCar();
void ABSControl(int16 pwm);
#endif