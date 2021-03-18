#ifndef __LED_H__
#define __LED_H__

#include "m_WS2812B.h"

#define  BEEP_PORT  PTB17

/* ���������� */
#define  BEEP_ON   PTXn_T(BEEP_PORT,OUT) = 1
#define  BEEP_OFF  PTXn_T(BEEP_PORT,OUT) = 0

/* LEDָʾ�ƿ��� */
#define  LED1_ON  PTXn_T(PTE28,OUT) = 0
#define  LED1_OFF PTXn_T(PTE28,OUT) = 1
#define  LED2_ON  PTXn_T(PTE27,OUT) = 0
#define  LED2_OFF PTXn_T(PTE27,OUT) = 1
#define  LED3_ON  PTXn_T(PTE26,OUT) = 0
#define  LED3_OFF PTXn_T(PTE26,OUT) = 1
#define  LED4_ON  PTXn_T(PTE25,OUT) = 0
#define  LED4_OFF PTXn_T(PTE25,OUT) = 1

#define  LED1_TURN PTXn_T(PTE28,T) = 1
#define  LED2_TURN PTXn_T(PTE27,T) = 1
#define  LED3_TURN PTXn_T(PTE26,T) = 1
#define  LED4_TURN PTXn_T(PTE25,T) = 1


/*
 *  ���ⲿ���õĺ����ӿ�����
 */
extern void m_LED_And_BEEP_Init();


#endif