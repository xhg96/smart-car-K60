#ifndef _LCD_KEY_H_
#define _LCD_KEY_H_
#include "deal_img.h"
#define FPNUM 7
#define SPNUM 7
#define MAINNUM 7

typedef struct 
{
  char name[15];
  Site_t site;
}var_struct_1;

typedef struct
{
    uint32  val;            //目前的值
    uint32  oldval;         //通常情况下，两者是相同，修改后，没按确认键，则不相同。按确认键发送后，则相同
                            //即最后发送的值
    uint32  minval;         //最小值
    uint32  maxval;         //最大值
    Site_t  site;           //LCD 显示的坐标位置
}var_struct_2;            //变量信息

typedef struct
{
    float  val;            //目前的值
    float  oldval;         //通常情况下，两者是相同，修改后，没按确认键，则不相同。按确认键发送后，则相同
                            //即最后发送的值
    float  minval;         //最小值
    float  maxval;         //最大值
    Site_t  site;           //LCD 显示的坐标位置
}var_struct_3;            //变量信息

typedef struct
{
  var_struct_2 val1;
  var_struct_2 val2;
  var_struct_2 val3;
  var_struct_2 val4;
  var_struct_2 val5;
  var_struct_2 val6;
}page_val;

void M_KEY_Init();
void show_first_page();
void LCD_reflash();
void M_key_event();
void showPicture(uint8 *img,uint8 w,uint8 h);
void showMap(const uint8 map[][XM],Site_t site,Size_t size,char flag);
void showbaseMap(const uint8 map[][XM],Site_t site,Size_t size);
void show_change();
void showotherMap(const uint8 map[][XM],Site_t site,Size_t size);
#endif