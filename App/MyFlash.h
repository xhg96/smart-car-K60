#ifndef _MYFLASH_H
#define _MYFLASH_H

#define SECTOR_NUM0  (FLASH_SECTOR_NUM-1)         //尽量用最后面的扇区，确保安全


void var_flash_select();        //拨码选扇区
void FlashSave(void);
void FlashRead(void);
void Key_Scan();
extern uint8 Flash_resetFlag;

#endif