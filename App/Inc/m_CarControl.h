/************************************

 * ���ú�����ͼ��������д���õ�·����Ϣ���Ը���־λ��λ
 * �㽭��ҵ��ѧ

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

extern void Car_Go(void);    //����

#endif  /*end of go.h*/