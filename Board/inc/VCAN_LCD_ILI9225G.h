#ifndef __LCD_H
#define __LCD_H		

#include "MK60_gpio.h"
#include "VCAN_LCD.h"

//�û�����
#define  ILI9225_DIR  1       //����Ĭ�Ϸ��� (0Ϊ������1Ϊ����)

//����LCD�ĳߴ�
#define ILI9225_W 220
#define ILI9225_H 176

#if (USE_LCD == LCD_ILI9225G)

/*********************** API�ӿ� ******************************/
//�ṩAPI�ӿڸ�LCD����

#define LCD_H                   ILI9225_H                 //��
#define LCD_W                   ILI9225_W                 //��

#define LCD_INIT()              LCD_ILI9225_init()             //��ʼ��
#define LCD_PTLON(site,size)    LCD_ILI9225_ptlon(site,size)   //����
#define LCD_RAMWR()             LCD_ILI9225_WR_CMD(0x22)       //дģʽ
#define LCD_WR_DATA(data)       LCD_ILI9225_WR_DATA(data)      //д����
#define LCD_WR_CMD(cmd)         LCD_ILI9225_WR_CMD(cmd)        //����
#define LCD_SET_DIR(opt)        LCD_ILI9225_dir(opt)           //����

#define LCD_DIR                 ILI9225_DIR                    //��ȡ����

#endif  //(USE_LCD == LCD_ILI9225G)

extern void     LCD_ILI9225_init();
extern void     LCD_ILI9225_dir(uint8 option);
extern void     LCD_ILI9225_ptlon(Site_t site, Size_t size);


//LCD �ĹܽŶ���

#define     LCD_ILI9225_WR      PTA26
#define     LCD_ILI9225_RD      PTB10
#define     LCD_ILI9225_CS      PTA24
#define     LCD_ILI9225_RS      PTA28
#define     LCD_ILI9225_RST     PTA25


#define     LCD_ILI9225_WR_OUT      PTXn_T(LCD_ILI9225_WR,OUT)    //д����
#define     LCD_ILI9225_RD_OUT      PTXn_T(LCD_ILI9225_RD,OUT)    //������
#define     LCD_ILI9225_CS_OUT      PTXn_T(LCD_ILI9225_CS,OUT)    //Ƭѡ
#define     LCD_ILI9225_RS_OUT      PTXn_T(LCD_ILI9225_RS,OUT)    //����/����
#define     LCD_ILI9225_RST_OUT     PTXn_T(LCD_ILI9225_RST,OUT)   //��λ

#define     ILI9225_DATAOUT         PTB_B0_OUT  //�������


//д�����
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

//д���ݺ���
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