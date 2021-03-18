#ifndef _M_SPEEDCONTROL_H_
#define _M_SPEEDCONTROL_H_


typedef enum {

	NORMAL_SHIFT,	//使用二次函数变速
	FULL_ACCELE,	//全加速
	
	BRAKE,			//刹车

	CURVE_ACCELE,	//出弯加速
	
}SPEED_TYPE_e;
//
//typedef struct {
//
//	int16 var[3];         //记录三次右边编码器的值
//
//	int16 nowSpeed;         //左右编码器速度平均值(等效当前速度)
//	int64 sumSpeed;         //当前累计总速度
//	int16 avgSpeed;         //当前平均速度
//	uint16 aimSpeed;	     //目标速度
//
//	int16 MotroPWM;        //电机输出PWM值 
//	
//}SPEEDInfo_st;
//
//
//
///* 小车运行模式枚举类型 */
//typedef enum{
//
//	CONSTANT_SPEED,
//	NORMAL_RUN,
//    TIMING_RUN,
//	
//}RUNMode_e;

typedef struct
{
	uint16 add_speed;               //加速
    
	uint16 extroSpeed;              //直道最大速度
	uint16 corner_minSpeed;         //弯道最小速度
	uint16 exitCorner_maxSpeed;     //出弯加速的最大速度
	uint16 rampSpeed;				//过坡道速度
	uint16 chase_minSpeed;			//追逐最小速度
	
	uint16 speed_k;             //二次公式系数
	uint16 curve_maxSpeed;     //二次公式的最大速度

	uint16 zrstop_speed;       //直道减速多减的余量
	uint16 cwrstop_speed;      //长入弯减速多减的余量
	uint16 dwrstop_speed;      //短入弯减速多减的余量

	uint16 zhidao_ade_th;         //入直道的判断
	uint16 ruwan_ade_th;          //入弯减速的判断
	uint16 chuwan_ade_th;         //出弯加速的判断

	uint16 top1_th;				//直道判断top1阈值
	
}SPEEDParam_st;


extern uint16 absde[];
extern uint8 top_save[];
extern uint8 top1;

/*
 *  供外部调用的函数接口声明
 */
extern void SpeedParam_Init();
extern uint16 Get_AimSpeed(void);

//
//extern SPEEDParam_st SpeedParam;	          //速度参数结构体
//extern SPEED_TYPE_e SpeedType;
//extern RUNMode_e RunMode;



#endif
