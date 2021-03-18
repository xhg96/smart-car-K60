#ifndef __LCD_H
#define __LCD_H		

#include "MK60_gpio.h"
#include "VCAN_LCD.h"

//用户配置
#define  ILI9225_DIR  1       //定义默认方向 (0为横屏、1为竖屏)

//定义LCD的尺寸
#define ILI9225_W 220
#define ILI9225_H 176

#if (USE_LCD == LCD_ILI9225G)

/*********************** API接口 ******************************/
//提供API接口给LCD调用

#define LCD_H                   ILI9225_H                 //高
#define LCD_W                   ILI9225_W                 //宽

#define LCD_INIT()              LCD_ILI9225_init()             //初始化
#define LCD_PTLON(site,size)    LCD_ILI9225_ptlon(site,size)   //开窗
#define LCD_RAMWR()             LCD_ILI9225_WR_CMD(0x22)       //写模式
#define LCD_WR_DATA(data)       LCD_ILI9225_WR_DATA(data)      //写数据
#define LCD_WR_CMD(cmd)         LCD_ILI9225_WR_CMD(cmd)        //命令
#define LCD_SET_DIR(opt)        LCD_ILI9225_dir(opt)           //方向

#define LCD_DIR                 ILI9225_DIR                    //获取方向

#endif  //(USE_LCD == LCD_ILI9225G)

extern void     LCD_ILI9225_init();
extern void     LCD_ILI9225_dir(uint8 option);
extern void     LCD_ILI9225_ptlon(Site_t site, Size_t size);


//LCD 的管脚定义

#define     LCD_ILI9225_WR      PTA26
#define     LCD_ILI9225_RD      PTB10
#define     LCD_ILI9225_CS      PTA24
#define     LCD_ILI9225_RS      PTA28
#define     LCD_ILI9225_RST     PTA25


#define     LCD_ILI9225_WR_OUT      PTXn_T(LCD_ILI9225_WR,OUT)    //写数据
#define     LCD_ILI9225_RD_OUT      PTXn_T(LCD_ILI9225_RD,OUT)    //读数据
#define     LCD_ILI9225_CS_OUT      PTXn_T(LCD_ILI9225_CS,OUT)    //片选
#define     LCD_ILI9225_RS_OUT      PTXn_T(LCD_ILI9225_RS,OUT)    //数据/命令
#define     LCD_ILI9225_RST_OUT     PTXn_T(LCD_ILI9225_RST,OUT)   //复位

#define     ILI9225_DATAOUT         PTB_B0_OUT  //数据输出


//写命令函数
#define LCD_ILI9225_WR_CMD(cmd) do\
        {\
            LCD_ILI9225_RD_OUT=1;\
            LCD_ILI9225_RS_OUT=0;\
            LCD_ILI9225_CS_OUT=0;\
            ILI9225_DATAOUT=(uint8)(cmd>>8);\
            LCD_ILI9225_WR_OUT=0;\
            LCD_ILI9225_WR_OUT=1;\
            ILI9225_DATAOUT=(uint8)(cmd);\
            LCD_ILI9225_WR_OUT=0;\
            LCD_ILI9225_WR_OUT=1;\
            LCD_ILI9225_CS_OUT=1;\
        }while(0)

//写数据函数
#define LCD_ILI9225_WR_DATA(data) do\
        {\
            LCD_ILI9225_RD_OUT=1;\
            LCD_ILI9225_RS_OUT=1;\
            LCD_ILI9225_CS_OUT=0;\
            ILI9225_DATAOUT=(uint8)(data>>8);\
            LCD_ILI9225_WR_OUT=0;\
            LCD_ILI9225_WR_OUT=1;\
            ILI9225_DATAOUT=(uint8)(data);\
            LCD_ILI9225_WR_OUT=0;\
            LCD_ILI9225_WR_OUT=1;\
            LCD_ILI9225_CS_OUT=1;\
        }while(0) 


#endif  