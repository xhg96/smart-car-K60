/*！
*     液晶显示+参数调试程序
*/

#ifndef __M_LCD_H__
#define __M_LCD_H__


#include "VCAN_LCD.h"
typedef enum
{
    HaventStarted = 0,
    TimeUp,      
    RunOutLine,     
    StopLineDetected,  
    StallProtect,
} stopReason_e;

extern stopReason_e stopReason;
extern const uint8 pic_LCDinit[];
extern const uint8 pic_Battery[];
extern const uint8 pic_Upload[];
extern const uint8 pic_Connect[];

/*
 *  供外部调用的函数接口声明
 */
extern void myLCD_Init();
extern void mLCD_point(uint8 X, uint8 Y, uint16 rgb565);
extern void mLCD_str(uint8 X, uint8 Y, const uint8 *Str, uint16 Color, uint16 bkColor);
extern void mLCD_num(uint8 X, uint8 Y, int num, uint8 max_num_bit, uint16 Color, uint16 bkColor);
extern void mLCD_pic(Site_t site, Size_t size, const uint8 *pic, uint16 Color, uint16 bkColor);
extern void newBoardTest(void);
extern void LCD_ParamAndImgDisplay();
extern void endInfoShow();
  // 外部变量
extern uint32 whileTime;
extern uint32 dealTime;
#endif