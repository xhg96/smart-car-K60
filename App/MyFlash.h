#ifndef _MYFLASH_H
#define _MYFLASH_H

#define SECTOR_NUM0  (FLASH_SECTOR_NUM-1)         //������������������ȷ����ȫ


void var_flash_select();        //����ѡ����
void FlashSave(void);
void FlashRead(void);
void Key_Scan();
extern uint8 Flash_resetFlag;

#endif