#include "common.h"
#include "include.h"
#include "VCAN_UI_VAR.h"
#include  "MK60_FLASH.h"    //FLASH�����ڶϵ籣��
#include  "MyFlash.h"

uint8 Flash_resetFlag=0;

uint16 SECTOR_NUM;

/****From UI_VAR.c****/
extern ui_var_info_t  num_info[VAR_MAX];
extern float *var_addr[VAR_MAX];

/*****************��Ӳ�������ѡ����************/
void var_flash_select()
{
  if(!gpio_get(PTE11))                  //����ѡ���� ����
  {
    SECTOR_NUM=SECTOR_NUM0-5;
  }
  else if(!gpio_get(PTE10))              //����ѡ���� ����
  {
    SECTOR_NUM=SECTOR_NUM0-7;
  }
  else if(!gpio_get(PTE9))                  //����ѡ���� ����
  {
    SECTOR_NUM=SECTOR_NUM0-6;
  }
  else SECTOR_NUM=SECTOR_NUM0-5;       //Ĭ�ϵ���
}

void FlashSave(void)
{
    flash_erase_sector(SECTOR_NUM);//������Ŀ
    for(uint8 var_num =1; var_num<VAR_MAX;var_num++)
    {            //����      ƫ�Ƶ�ַ       ����                   ָ�����飬���˴������������ݣ�����Ŵ����
      flash_write(SECTOR_NUM,64*(var_num-1),(uint32)( *((float*) (var_addr[var_num]))*VAR_SHOWEN(var_num) ) );
    }  //64λһд ������ȫд������                                //����
}

void FlashRead(void)
{
  uint8 var_num;
  if(Flash_resetFlag==0)
  {                             
    for (var_num = 1; var_num < VAR_MAX; var_num++ )
    {
      *((float *)(var_addr[var_num])) = (float)((float)flash_read(SECTOR_NUM,64*(var_num-1),uint32)/VAR_SHOWEN(var_num)); 
    }  
  }
}

int Key_Press_Flag = 0;
void Key_Scan()
{ 
    if(Key_Press_Flag == 1)
    {
        static int count=0;
        count++;
        if(count == 50) { Key_Press_Flag = 0; count =0; }
    }
    if(Key_Press_Flag == 0)
    {
        /******����ѡ��********************/
        if(gpio_get(PTE3) == 0)
        {
            gpio_set (PTD15, 1); //������������ʾ
            FlashSave();
            gpio_set (PTD15, 0); 
            Key_Press_Flag=1;
        }
    }
}