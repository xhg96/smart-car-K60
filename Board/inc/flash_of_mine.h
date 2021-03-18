#ifndef FLASHOFMINE
#define FLASHOFMINE

#define SECTOR_NUM0  (FLASH_SECTOR_NUM-1)
#define SW8    PTE6
#define SW7    PTE7
#define SW6    PTE8
#define SW5    PTE9
#define SW4    PTE10
#define SW3    PTE11
#define SW2    PTE25
#define SW1    PTE24

//extern uint8 Flash_resetFlag;

void Key_Scan();
void FlashRead(void);
void FlashSave(void);
void var_flash_select();

#endif