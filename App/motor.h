/*电机头文件*/
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
	
}DMPDELAY_e;    //dmp延时标志物相关

extern DMPDELAY_e DMP_Delay_Status;
extern int speed_type;
//extern int aim_speed;
//extern int now_speed;

extern uint16 max_speed;  
extern uint16 max2_speed;

extern uint16 encoderThreshold;
extern uint16 bumpEncoderThreshold;
extern uint16 RuningTime;   
extern int StartDelayCount;        //发车延时计数器
extern int StopDelayCount;
extern uint8 stopLineDetected;
extern uint8 StopLineNoDetect;
extern uint8 stopRemember;

extern uint16 annulus_speed;     //圆环速度参数
extern uint16 ann_speed_k;

//弯道速度变量
extern uint16 wan_speed;         //二次公式的常数项
extern uint16 min_speed;         //弯道最小速度
extern uint16 cwrstop_speed;       //长入弯减速多减得余量
extern uint16 dwrstop_speed;       //短入弯减速多减得余量
extern uint16 ann_min_speed;
//加速速度变量
extern uint16 base_speed;        //出弯基础速度
extern uint16 outm_speed;        //出弯加速的最大速度
extern uint16 add_speed;//4        //加速
extern uint16 dec_add_flag;

//刹车速度变量
extern uint16 extraStopVal_Short;       //短入弯减速多减得余量
extern uint16 extraStopVal_Turnin;
extern uint16 extraStopVal_Long; 
extern uint16 speed_k;
extern uint16 speed_k2;

extern uint16 bra_speed1 ;
extern uint16 bra_speed2 ;
extern uint16 bra_speed3 ;
extern uint16 bra_speed4 ;

//其他速度变量
extern uint16 ra_speed;         //坡道速度
extern uint16 obcutsp;          //障碍减掉的速度
extern uint16 max2_speed;        //二次公式的最大速度
extern uint16 bump_speed;
//加速判断变量
extern uint16 zhidao_ade;         //入直道的判断
extern uint16 wan_ade;            //出弯加速的判断
extern uint16 set_yh_top;

//刹车判断变量
extern uint16 dzhi_top;           //直道刹车的判断
extern uint16 set_othertop;       //另一顶端的设置
extern uint16 set_cothertop;      //另一顶端的设置
extern uint16 wanru_ade;          //弯道加速的判断

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