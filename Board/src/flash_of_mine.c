#include "flash_of_mine.h"
#include "common.h"
#include "include.h"
#include "VCAN_UI_VAR.h"
#include  "MK60_FLASH.h"    //FLASH�����ڶϵ籣��

extern uint8 flash_num;     //����ѡ��
extern ui_var_info_t  num_info[VAR_MAX];
extern float *var_addr[VAR_MAX];

uint8 Flash_resetFlag=0;        //��flash��־λ���ݲ��ã���ÿ�ο��������flash
uint16 MY_SECTOR_NUM;

void var_flash_select()
{
  if(gpio_get(SW3) == 1 && gpio_get(SW6) ==1) //����3 + 6�������ٲ���
    MY_SECTOR_NUM=SECTOR_NUM0-5;
  else if(gpio_get(SW3) == 0 && gpio_get(SW6) ==1)//����6�����ٲ���
    MY_SECTOR_NUM=SECTOR_NUM0-2; 
  else if(gpio_get(SW3) == 1 && gpio_get(SW6)==0)//����3�����ٲ���
    MY_SECTOR_NUM=SECTOR_NUM0-4;
  else                                          //Ĭ�ϣ����ٲ���
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

//*****�������������ͨ������2����д��flash���������������رտ���2
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
        /******����ѡ��********************/
        if(gpio_get(SW2) == 1)  //����2����ʱ��дflash
        {
            gpio_set (PTE26, 1); //������������ʾ
            FlashSave();
            gpio_set (PTE26, 0); //flashд��ط�����
            Key_Press_Flag=1;
        }
    }
}
