#ifndef  WS_2811_H
#define  WS_2811_H

#define  CASCADE  2


extern uint8 RGB[CASCADE][3];

extern void WS_Updata();
extern void WS_Init();
extern void WS_SetColorAll(uint8 red, uint8 green, uint8 blue);


#endif
