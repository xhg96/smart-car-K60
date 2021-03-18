/************************************

 * 调用函数对图形数组进行处理得到路况信息，对各标志位置位
 * 浙江工业大学

 ***********************************/

#ifndef _GO_H
#define _GO_H


extern uint16 Ramp_time;
extern uint16 Stop_time;
extern uint16 ChaseDistance_t;

extern uint8 RampUp_flag,RampDown_flag;
extern uint8 circle_counter;

extern IncrementalPID_st Distance_PID;

extern DOUBLECAR_INFO_st DoubleCar_Info;

extern void Car_Go(void);    //流程

#endif  /*end of go.h*/