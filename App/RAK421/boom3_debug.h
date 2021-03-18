#ifndef _BOOM3_DEBUG_H_
#define _BOOM3_DEBUG_H_
#include "common.h"
#include "include.h"
#include "rak_global.h"
#include "rak411_api.h"
#include "rak_config.h"
#include "deal_img.h"
#include "m_Motro.h"
typedef struct{
	uint8 cmd[4];
	int size;
}Wifi_Head;
typedef struct{
    Wifi_Head head;//8 byte
    uint8 map[YM][XM / 8]; //400 byte
    SPEEDInfo_st SPD;//32 byte
    float Pitch;//4 byte
    float Roll;//4 byte
    float Yaw;//4 byte
    uint32 whileTime;//4 byte
    uint32 dealTime;//4 byte
    int gyro[3];//4 byte*3
	uint8 data[39];
    uint8 end;//1 byte//至此512字节
	uint8 ccdBuff[128];
}WIFI_BUFF;

typedef union
{
	uint8 maxsize[512*2];//扇区大小为512字节
	WIFI_BUFF wifi_buff;
}SECTOR_SAVE_BUFF;
void GPIO_config();
void SPI_config();
void Reset_config();
void SYS_Delay(uint32_t us);

void rak421_init();
void wifi_msg();
extern volatile 	rak_intStatus		rak_strIntStatus;
extern uint8 	 	Send_RecieveDataFlag;
//extern WIFI_BUFF wifi_buff;
extern SECTOR_SAVE_BUFF SAB;
void wifi_wait_for_response();









#endif