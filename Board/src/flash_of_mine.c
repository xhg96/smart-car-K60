#include "flash_of_mine.h"
#include "common.h"
#include "include.h"
#include "VCAN_UI_VAR.h"
#include  "MK60_FLASH.h"    //FLASH，用于断电保存

extern uint8 flash_num;     //扇区选择
extern ui_var_info_t  num_info[VAR_MAX];
extern float *var_addr[VAR_MAX];

uint8 Flash_resetFlag=0;        //读flash标志位，暂不用，即每次开机都会读flash
uint16 MY_SECTOR_NUM;

void var_flash_select()
{
  if(gpio_get(SW3) == 1 && gpio_get(SW6) ==1) //开关3 + 6，超高速参数
    MY_SECTOR_NUM=SECTOR_NUM0-5;
  else if(gpio_get(SW3) == 0 && gpio_get(SW6) ==1)//开关6，高速参数
    MY_SECTOR_NUM=SECTOR_NUM0-2; 
  else if(gpio_get(SW3) == 1 && gpio_get(SW6)==0)//开关3，低速参数
    MY_SECTOR_NUM=SECTOR_NUM0-4;
  else                                          //默认，中速参数
    MY_SECTOR_NUM=SECTOR_NUM0-3;
}

void FlashSave(void)
{
    flash_erase_sector(MY_SECTOR_NUM);
    for(uint8 var_num =1; var_num<VAR_MAX;var_num++)
    {
      flash_write(MY_SECTOR_NUM,64*(var_num-1),(uint32)(*((float*)(var_addr[var_num]))*VAR_SHOWEN(var_num)));
    }
}

void FlashRead(void)
{
  uint8 var_num;
  if(Flash_resetFlag==0)
  {
    for (var_num = 1; var_num < VAR_MAX; var_num++ )
    {
      *((float *)(var_addr[var_num])) = (float)((float)flash_read(MY_SECTOR_NUM,64*(var_num-1),uint32)/VAR_SHOWEN(var_num)); 
    }  
  }
}

int Key_Press_Flag = 0;

//*****按键改完参数后通过开关2将其写入flash，蜂鸣器不再响后关闭开关2
void Key_Scan()                 
{ 
    if(Key_Press_Flag == 1)
    {
        static int count=0;
        count++;
        if(count == 100) { Key_Press_Flag = 0; count =0; }
    }
    if(Key_Press_Flag == 0)
    {
        /******步长选择********************/
        if(gpio_get(SW2) == 1)  //开关2开启时，写flash
        {
            gpio_set (PTE26, 1); //开启蜂鸣器提示
            FlashSave();
            gpio_set (PTE26, 0); //flash写完关蜂鸣器
            Key_Press_Flag=1;
        }
    }
}
