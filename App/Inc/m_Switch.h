#ifndef __M_SWITCH_H__
#define __M_SWITCH_H__

#include "MK60_gpio.h"

#define  GET_SWITCH1()  PTE12_IN
#define  GET_SWITCH2()  PTE11_IN
#define  GET_SWITCH3()  PTE10_IN
#define  GET_SWITCH4()  PTE9_IN
#define  GET_SWITCH5()  PTE8_IN
#define  GET_SWITCH6()  PTE7_IN
#define  GET_SWITCH7()  PTE6_IN
#define  GET_SWITCH8()  PTD15_IN



//开关端口的枚举
typedef enum
{
    SWITCH_1,
    SWITCH_2,
    SWITCH_3,
    SWITCH_4,
    SWITCH_5,
    SWITCH_6,
    SWITCH_7,
    SWITCH_8,

    SWITCH_MAX,
} SWITCH_e;

typedef enum
{
    SWITCH_ON = 0,         //按键按下时对应电平
    SWITCH_OFF = 1,         //按键弹起时对应电平
} SWITCH_STATUS_e;

typedef enum
{
  VHIGH = 0,
  VMEDIUM,
  VLOW,
  
}VOICE_TYPE_e;

/*
 *  供外部调用的函数接口声明
 */
extern void              Switch_Init();
extern SWITCH_STATUS_e   Switch_Get(SWITCH_e key);

void touch_IRQHandler();
void touchSwitchInit();

#endif