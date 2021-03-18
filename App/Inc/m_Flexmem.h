/******************************************************************************
* File:    flexmem_demo.h
* Purpose: Create defines that can be used with the Partition_Flash function.
******************************************************************************/

#ifndef _M_FLEXMEM_H_
#define _M_FLEXMEM_H_ 1


/* Defines for the eeprom_size parameter */
/* Note: due to an errata subsystem A and B must be the same size, so
 * many of these defines should not be used on current silicon.
 */
#define EEPROM_4_28     0x09      // subsystem A = 4 bytes, subsystem B = 28 bytes
#define EEPROM_8_24     0x19      // subsystem A = 8 bytes, subsystem B = 24 bytes
#define EEPROM_16_16    0x39      // subsystem A = 16 bytes, subsystem B = 16 bytes

#define EEPROM_8_56     0x08      // subsystem A = 8 bytes, subsystem B = 56 bytes
#define EEPROM_16_48    0x18      // subsystem A = 16 bytes, subsystem B = 48 bytes
#define EEPROM_32_32    0x38      // subsystem A = 32 bytes, subsystem B = 32 bytes

#define EEPROM_16_112   0x07      // subsystem A = 16 bytes, subsystem B = 112 bytes
#define EEPROM_32_96    0x17      // subsystem A = 32 bytes, subsystem B = 96 bytes
#define EEPROM_64_64    0x37      // subsystem A = 64 bytes, subsystem B = 64 bytes

#define EEPROM_32_224   0x06      // subsystem A = 32 bytes, subsystem B = 224 bytes
#define EEPROM_64_192   0x16      // subsystem A = 64 bytes, subsystem B = 192 bytes
#define EEPROM_128_128  0x36      // subsystem A = 128 bytes, subsystem B = 128 bytes

#define EEPROM_64_448   0x05      // subsystem A = 64 bytes, subsystem B = 448 bytes
#define EEPROM_128_384  0x15      // subsystem A = 128 bytes, subsystem B = 384 bytes
#define EEPROM_256_256  0x35      // subsystem A = 256 bytes, subsystem B = 256 bytes

#define EEPROM_128_896  0x04      // subsystem A = 128 bytes, subsystem B = 896 bytes
#define EEPROM_256_768  0x14      // subsystem A = 256 bytes, subsystem B = 768 bytes
#define EEPROM_512_512  0x34      // subsystem A = 512 bytes, subsystem B = 512 bytes

#define EEPROM_256_1792 0x03      // subsystem A = 256 bytes, subsystem B = 1792 bytes
#define EEPROM_512_1536 0x13      // subsystem A = 512 bytes, subsystem B = 1536 bytes
#define EEPROM_1K_1K    0x33      // subsystem A = 1 Kbytes, subsystem B = 1 Kbytes

#define EEPROM_512_3584 0x02      // subsystem A = 512 bytes, subsystem B = 3584 bytes
#define EEPROM_1K_3K    0x12      // subsystem A = 1 Kbytes, subsystem B = 3072 bytes
#define EEPROM_2K_2K    0x32      // subsystem A = 2 Kbytes, subsystem B = 2 Kbytes

#define EEPROM_1K_7K    0x01      // subsystem A = 1 Kbytes, subsystem B = 7 Kbytes
#define EEPROM_2K_6K    0x11      // subsystem A = 2 Kbytes, subsystem B = 6 Kbytes
#define EEPROM_4K_4K    0x31      // subsystem A = 4 Kbytes, subsystem B = 4 Kbytes

#define EEPROM_2K_14K   0x00      // subsystem A = 2 Kbytes, subsystem B = 14 Kbytes
#define EEPROM_4K_12K   0x10      // subsystem A = 4 Kbytes, subsystem B = 12 Kbytes
#define EEPROM_8K_8K    0x30      // subsystem A = 8 Kbytes, subsystem B = 8 Kbytes

/* Defines for the eflash_size parameter */
#define EFLASH_SIZE_0   0x00
#define EFLASH_SIZE_64  0x04
#define EFLASH_SIZE_128 0x05
#define EFLASH_SIZE_256 0x06
#define EFLASH_SIZE_512 0x08
#define EFLASH_SIZE_448 0x0C
#define EFLASH_SIZE_384 0X0D

  //以下为使用FLASH部分
extern uint8 flash_num;
void FlashSave(void);
void FlashRead(void);


/* function prototypes */
extern int Partition_Flash(int, int);


#endif  //__FLEXMEM_DEMO_H

/* End of "flexmem_demo.h" */
