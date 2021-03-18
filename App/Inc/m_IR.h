#ifndef __IR__NEC_H__
#define __IR__NEC_H__


#define  IR_PORT  PTC16
#define  IR_IN    PTXn_T(IR_PORT,IN)

#define  UltroSound_Send  PTC10_OUT

//红外遥控识别码(ID),每款遥控器的该值基本都不一样,但也有一样的.
//我们选用的遥控器识别码为0
#define REMOTE_ID 0

extern void IR_Init();
extern uint8 IR_Decodeing(uint8 * IR_value);

extern void UltraSound_Init();

extern void PORTC_IRQHandler(void);


#endif
