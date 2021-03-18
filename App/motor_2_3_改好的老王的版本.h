/*电机头文件*/
#ifndef _MOTOR_H
#define _MOTOR_H

#include "include.h"
#include "control.h"
#include "fuzzy_pid.h"

extern int speed_type;
//void count_speed();
void Speed_Get();
void speed_pid(float aim,int now,uint8 flag);
void a_speed_pid(int aim,int now);
void pwm(int speed_l,int speed_r);
void Speed_go();
void stop_car();
void StopCar();

void Speed_Get();
void Track_type_detect();
void Speed_Set();

void Car_go();

void send_speedval();
#endif          //_MOTOR_H