/*!
 *   @file      ServoCtrl.h" 
 *   @brief     舵机控制头文件
*/

#ifndef _M_SERVO_H_
#define _M_SERVO_H_


//#define SD5_FTM  FTM1
//#define SD5_CH   FTM_CH1
//#define SD5_HZ   300
//
////#define MID_PWM  750          //正中间的PWM值(50Hz，0.75%占空比，即1.5ms左右)
//#define SERVOPWM_MAX   1100     //左侧最大PWM值
//#define SERVOPWM_MIN   -1100   //右侧最大PWM值
//
//
//#define SERVO_MAXPWM_CNV  +360    //左侧打角最大时的FTM_CNV寄存器偏移量
//#define SERVO_MINPWM_CNV  -360    //右侧打角最大时的FTM_CNV寄存器偏移量
//
//
///* 舵机控制PD结构体 */
//typedef struct{
//
//    int16 error;
//    int16 preError;
//    
//    uint16 Kp;
//    uint16 Kd;
//    
//    int16 PID_Out;            //舵机输出PWM值(相对量)
//    
//}PositionalPID_st;
//
///* 外部变量声明 */
//extern uint16 ServoMidPWM_CNV;
//extern PositionalPID_st Servo_PID;			//舵机控制PID结构体
//extern uint16 MID_PWM;
//
///*
// *  供外部调用的函数接口声明
// */
//extern void Servo_Init();    //PID结构体初始化 
//
//extern int16 Calc_ServoPID(PositionalPID_st *pp);



#endif