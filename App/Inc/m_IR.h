#ifndef __IR__NEC_H__
#define __IR__NEC_H__


#define  IR_PORT  PTC16
#define  IR_IN    PTXn_T(IR_PORT,IN)

#define  UltroSound_Send  PTC10_OUT

//����ң��ʶ����(ID),ÿ��ң�����ĸ�ֵ��������һ��,��Ҳ��һ����.
//����ѡ�õ�ң����ʶ����Ϊ0
#define REMOTE_ID 0

extern void IR_Init();
extern uint8 IR_Decodeing(uint8 * IR_value);

extern void UltraSound_Init();

extern void PORTC_IRQHandler(void);


#endif
