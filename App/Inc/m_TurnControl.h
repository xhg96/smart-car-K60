#ifndef _CONTROL_H
#define _CONTROL_H


/* �����ⲿ���� */
extern float Slope_L,Slope_R;
extern float Offset_L[],Offset_R[];

extern uint16 Lowest_Row,HighestRow_Circle;

extern int16 servo_PWM;

extern uint16 RampUp_Angle,RampDown_Angle;

extern uint16 Turn_CompensatingFactor_L,Turn_CompensatingFactor_R;
extern uint16 Factor_b; 
extern uint16 Obstacle_bias;

/*
 *  ���ⲿ���õĺ����ӿ�����
 */
extern void Car_Turn(void);    //С��ת�����


#endif          //_CONTROL_H