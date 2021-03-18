#include "include.h"
#include "Img2sd.h"
#include "ff.h"
#include "VCAN_camera.h"       //摄像头总头文件
#include "deal_img.h"
#include "motor.h"
#include "m_Motro.h"
#include "boom3_debug.h"
#include "diskio.h"

extern uint32 whileTime;
extern uint8 basemap[YM][XM], top;
extern uint16 absde[3];
extern float Pitch,Roll,Yaw;
extern uint8 CCD_BUFF[TSL1401_SIZE];  
IMAGE2SDINFO Image2sd;
extern short gyro[3];
void clear_disk(uint32 sector)
{
  uint32 start=sector;
  uint8 temp[4];
  memcpy((uint8*)temp,(uint8*)(SAB.wifi_buff.head.cmd),sizeof(uint8)*4); 
  memset((uint8*)(SAB.wifi_buff.head.cmd),0,sizeof(uint8)*4);
  while(sector<start+2800)
  {
    if( RES_OK == wait_for_SEND_STATUS_OK() )
      { 
          
          disk_write_without_CMD13(0, (uint8*)(&SAB), sector, 1);  //直接写扇区
      }
    sector+=100;
  }
  memcpy((uint8*)(SAB.wifi_buff.head.cmd),(uint8*)temp,sizeof(uint8)*4);  
}

void img_sd_init_disk(void)
{
    static DSTATUS stat;
    
    Image2sd.stopCar_to_send_data = 0; //停车信号，开始向上位机发数据
    SAB.wifi_buff.head.cmd[0]='m';
    SAB.wifi_buff.head.cmd[1]='a';
    SAB.wifi_buff.head.cmd[2]='p';
    SAB.wifi_buff.head.cmd[3]='0';
    SAB.wifi_buff.head.size=sizeof(SAB.wifi_buff);
    SAB.wifi_buff.end=0x1B;
    stat = disk_initialize(0); 	
    if(stat == 0)
    {
        mLCD_str(0,20,"SD_Card Init ok ", BLUE,WHITE);
        Image2sd.SD_config_OK = 1;
        Image2sd.sector = 4096; 
        clear_disk(Image2sd.sector);
    }
    else
    {
        BEEP_ON;
        DELAY_MS(100);
        mLCD_str(0,20,"SD_Card Init error ", BLUE,WHITE);
        Image2sd.SD_config_OK = 0;
        BEEP_OFF;
    }
}
int send_data_to_SD_disk(void)
{
    if(Image2sd.SD_config_OK == 0)    //初始化失败
      return -1;
    SAB.wifi_buff.SPD=SPD;
    SAB.wifi_buff.Pitch=Pitch;
    SAB.wifi_buff.Roll=Yaw;
    SAB.wifi_buff.Yaw=Roll;
    SAB.wifi_buff.gyro[0]=gyro[0];
    SAB.wifi_buff.gyro[1]=gyro[1];
    SAB.wifi_buff.gyro[2]=gyro[2];
    SAB.wifi_buff.dealTime=dealTime;
    SAB.wifi_buff.whileTime=whileTime;
    if( RES_OK == wait_for_SEND_STATUS_OK() )
    { 
        Image2sd.sector = Image2sd.sector + 1;
        disk_write_without_CMD13(0, (uint8*)(&SAB), Image2sd.sector, 1);  //直接写扇区     
    }
    return 0;
}

int SD_disk_2_RAK421(void){
    uint8 recbuf[40*80 / 8];    //图像大小
    uint16 total_image_num;
    
    total_image_num = (Image2sd.sector - 4096)/2;
    
    if(total_image_num == 0 )
    {
        mLCD_str(0,60,"No image", BLUE,GREEN);
        return -1;
    }
    
    mLCD_num(0,60,(int)(total_image_num),4,RED,GREEN);
    mLCD_str(30,60,"imgs sending ...", BLUE,GREEN);  
    mLCD_str(18,80,"sent", BLUE,GREEN);
    Image2sd.sector = 4096;
   // rak421_init();
  //  wifi_wait_for_response();
    while(total_image_num > 0)
    {

     Image2sd.sector = Image2sd.sector + 2;
        
     disk_read(0, recbuf, Image2sd.sector, 1);       //读扇区

        uint16 img_count = 0;
        for(int8 i = 0; i < 80; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
                  SAB.wifi_buff.map[i][j] = recbuf[img_count];
                  img_count++;
                }
        }//     这个for结束，存了一副图像
          //此处添加发送函数
        
        total_image_num = total_image_num - 1;
        
        mLCD_num(56,80,(int)(total_image_num),4,RED,GREEN);
    }
    return 0;
}
/*!
 *  @brief     将disk_write函数write到SD的数据，通过disk_read函数read出来， 然后再存到文件里
 *  @time      17-6-25
 *  @author    cameling(吕佳俊)
 */

int SD_disk_to_FAT_file(void)
{
    uint8 recbuf[500]; 
    uint16 total_image_num;
   // uint32 mybw;
   // int res;
    
    total_image_num = (Image2sd.sector - 4096)/2;
    
    if(total_image_num == 0 )
    {
        mLCD_str(0,60,"No image", BLUE,GREEN);
        return -1;
    }
  
    mLCD_num(0,60,(int)(total_image_num),4,RED,GREEN);
   // mLCD_str(30,60,"imgs,data printf ing ...", BLUE,GREEN);
    mLCD_str(18,80,"SEND", BLUE,GREEN);
    Image2sd.sector = 4096;
    
    while(total_image_num > 0)
    {
        Image2sd.sector = Image2sd.sector + 2;
        
        disk_read(0, recbuf, Image2sd.sector, 1);       //读扇区
   
//
        for(uint16 i = 0; i < 500; i++)
        {
          
          //printf("data:%d\n",(int)recbuf[i]);
           // uart_putchar(UART0, recbuf[i]);       
        }
        
        total_image_num = total_image_num - 1;
        
        
        mLCD_num(56,80,(int)(total_image_num),4,RED,GREEN);
        
//        Image2sd.uart1_rec_busy = 1;
//        LED3_ON;
//        
//        while(Image2sd.uart1_rec_busy);
//        LED3_OFF;
    }    

        recbuf[0] = 0xfc;              //每一帧的开头
	recbuf[1] = 0x02;					
	
	recbuf[2] = 0x80;			 //先发数据长度
	recbuf[3] = 3;		           //结束
	
    uart_putbuff(UART0, recbuf, 4);
    
    mLCD_str(0,80,"    SEND OK    ", BLUE,GREEN);
    
    return 0;
}