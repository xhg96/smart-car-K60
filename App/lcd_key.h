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
    uint32  val;            //Ŀǰ��ֵ
    uint32  oldval;         //ͨ������£���������ͬ���޸ĺ�û��ȷ�ϼ�������ͬ����ȷ�ϼ����ͺ�����ͬ
                            //������͵�ֵ
    uint32  minval;         //��Сֵ
    uint32  maxval;         //���ֵ
    Site_t  site;           //LCD ��ʾ������λ��
}var_struct_2;            //������Ϣ

typedef struct
{
    float  val;            //Ŀǰ��ֵ
    float  oldval;         //ͨ������£���������ͬ���޸ĺ�û��ȷ�ϼ�������ͬ����ȷ�ϼ����ͺ�����ͬ
                            //������͵�ֵ
    float  minval;         //��Сֵ
    float  maxval;         //���ֵ
    Site_t  site;           //LCD ��ʾ������λ��
}var_struct_3;            //������Ϣ

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