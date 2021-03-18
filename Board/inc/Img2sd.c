#include "include.h"
#include "Img2sd.h"
#include "ff.h"
#include "VCAN_camera.h"       //摄像头总头文件
#include "deal_img.h"
#include "motor.h"
#include "boom3_debug.h"
#include "diskio.h"

extern uint32 whileTime;
extern uint8 basemap[Y][X], top;
extern uint16 absde[3];

IMAGE2SDINFO Image2sd;


/**********************************************
功能：sd驱动初始化，初始化方式为fat文件系统
描述：每次初始化成功后会创建一个文件，并写入了图像长宽
return： void
***********************************************/
//void img_sd_init(void)
//{
//    static int     res;			
//    char    myfilename[20];
//    uint32  Imag_num = 0;
//
//    mLCD_str(0,20,"SD_Card Init ing...", BLUE,WHITE);
//    
//    f_mount(0, &Image2sd.filesystem);
//
//    do
//    {
//        Imag_num ++;
//        sprintf(myfilename, "0:/FIRE%d.SD", Imag_num); //转化为字符串，保存在myfilename里，每一次SD初始化会新建一个文件
//		
//        res = f_open( &Image2sd.file, myfilename, FA_CREATE_NEW | FA_WRITE);
//        
//        /*
//		if( res == FR_DISK_ERR)
//		{
//        	printf( "\n没插入SD卡??\n" );
//        	return;
//   	 	}
//    	else if ( res == FR_OK )
//    	{
//        	printf( "\n文件打开成功 \n" );
//    	}
//    	else
//    	{
//        	printf("\n返回值异常");
//        	return;
//    	}
//		*/
//        
//        if(Image2sd.filesystem.fs_type == 0)
//        {
//            Image2sd.file.fs = 0;
//            Image2sd.SD_config_OK = 0;
//            
//            mLCD_str(0,20,"SD_Card Init Error ", BLUE,WHITE);
//            
//            return;
//        }
//
//    }while(res == FR_EXIST);        //如果文件存在，则命名继续加1
//
//    if ( res == FR_OK )//初始化成功
//    {
//        //uint16  imgsize[] = {75,40};      //高、宽、摄像头颜色类型
//        //uint32  mybw;
//        //res = f_write(&vcansrc, imgsize, sizeof(imgsize), &mybw);  //先写入高和宽，方便上位机处理
//      
//        Image2sd.flag_disk2file = 1; 
//        //Image2sd.sector = 4096;    //从第4096个扇区开始存数据
//        Image2sd.SD_config_OK = 1;
//        Image2sd.file_number = Imag_num;
//        mLCD_str(0,20,"SD_Card Init OK!   ", BLUE,WHITE);
//    }
//    else
//    {
//        f_close(&Image2sd.file);
//        Image2sd.file.fs = 0;
//    }
//}



void img_sd_init_disk(void)
{
    static DSTATUS stat;
    
    Image2sd.stopCar_to_send_data = 0; //停车信号，开始向上位机发数据
    
    stat = disk_initialize(0); 	
    if(stat == 0)
    {
        mLCD_str(0,20,"SD_Card Init ok ", BLUE,WHITE);
        Image2sd.SD_config_OK = 1;
        Image2sd.sector = 4096; 
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



//void img_sd_save(uint8 * imgaddr,uint32 size)
//{
//#define F_SYNC_TIME   25     //保存 n 次后 才 同步
//  
//    static uint8 time = 0;
//    int   res;
//    uint32 mybw;
//    
//    if(Image2sd.file.fs != 0)
//    {
//        time++;
//
//        res = f_write(&Image2sd.file, imgaddr, size ,&mybw);
//
//        if(res != FR_OK)
//        {
//            f_close(&Image2sd.file);
//            Image2sd.file.fs = 0;
//        }
//
//        if(time > F_SYNC_TIME)
//        {
//            time = 0 ;
//            //LED1_OFF;
//            BEEP_ON;
//            f_sync(&Image2sd.file);
//            //LED1_ON;
//        }
//    }
//}
//
//void img_sd_exit(void)
//{
//    f_close(&Image2sd.file);
//    Image2sd.file.fs = 0;
//}

/******************************************************************************
 * 功能   将basemap的像素转化为一个uint8 保存8个像素点
 * 时间   17-5-19
 * 作者   吕佳俊
******************************************************************************/
void basemap2transport(uint8 * sendbuf, uint8 start_pos)
{
  	//uint8 sendbuf[75*5]; //将要传送的数据保存在buf里面

	/* 将处理后的图像压缩，用于上传到上位机*/
	uint8 data_t;
	
	for(int8 i = 0; i < 80; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
            //每8个像素点压缩到1个字节里传输
			for(uint8 k = 0; k < 8; k++)
			{
                          if(basemap[i][j*8+k] == 0)    //白色	  
                            data_t &= ~(0x80 >> k);  //白色置0
                          else 
                            data_t |= 0x80 >> k;       //黑色是1
			}                             
			
			sendbuf[i*5 + j + start_pos] = data_t;
		}
	}
}

void basemap2dezip(uint8 * sendbuf, uint8 start_pos)//,uint8 * showmap[80][40])
{
  	//uint8 sendbuf[75*5]; //将要传送的数据保存在buf里面

	/* 将处理后的图像解压缩，用于上传到上位机*/
	uint8 data_t;
	
	for(int8 i = 0; i < 75; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
                  data_t = sendbuf[i*5 + j + start_pos];
                  
			for(uint8 k = 0; k < 8; k++)
			{
//                          if(data_t & (0x80 >> k) == (0x80 >> k))       //黑色
//                            showmap[i][j*8+k] = 1;
//                          else
//                            showmap[i][j*8+k] = 0;
                          
//                          if(basemap[i][j*8+k] == 0)    //白色	  
//                            data_t &= ~(0x80 >> k);  //白色置0
//                          else 
//                            data_t |= 0x80 >> k;       //黑色是1
			}                             
			
			
		}
	}
}

/******************************************************************************
 * 功能：保存图片到SD卡，disk模式
 * 时间：17-5-19 
 * 作者：吕佳俊
******************************************************************************/
int send_data_to_SD_disk(void)
{
	uint8 sendbuf[500] = {33}; 

      if(Image2sd.SD_config_OK == 0)    //初始化失败
        return -1;
	
	basemap2transport(sendbuf, 0);  //图像
	//其他信息
//	sendbuf[374+0] = BYTE1(SpeedInfo.aimSpeed);//目标速度-高位
//	sendbuf[374+1] = BYTE0(SpeedInfo.aimSpeed);//目标速度-低位
//	
//	sendbuf[374+2] = BYTE1(SpeedInfo.nowSpeed); //编码器速度平均值(等效当前速度)--高位
//	sendbuf[374+3] = BYTE0(SpeedInfo.nowSpeed); //有正负
//	
//	
//	sendbuf[374+6] = speed_type;		//速度控制类型标志，0表示二次公式循迹，1表示直线加速，2表示入弯减速
//	
//	sendbuf[374+7] = BYTE1(Servo_PID.PID_Out);      //舵机的相对控制量
//	sendbuf[374+8] = BYTE0(Servo_PID.PID_Out);     //-500到500
//    
//	
//	sendbuf[374+31] = BYTE1(absde[0]);      //偏差
//	sendbuf[374+32] = BYTE0(absde[0]);
//
//	sendbuf[374+33] = BYTE0(top);
//	//sendbuf[379+34] = BYTE0(other_top);
//
//	sendbuf[374+42] = BYTE3(whileTime);     //while循环时间
//	sendbuf[374+43] = BYTE2(whileTime);
//	sendbuf[374+44] = BYTE1(whileTime);
//	sendbuf[374+45] = BYTE0(whileTime);

      
    if( RES_OK == wait_for_SEND_STATUS_OK() )
    { 
        Image2sd.sector = Image2sd.sector + 2;
        
        disk_write_without_CMD13(0, sendbuf, Image2sd.sector, 1);  //直接写扇区
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
    rak421_init();
    wifi_wait_for_response();
    while(total_image_num > 0)
    {

     Image2sd.sector = Image2sd.sector + 2;
        
     disk_read(0, recbuf, Image2sd.sector, 1);       //读扇区

        uint16 img_count = 0;
        for(int8 i = 0; i < 80; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
                  wifi_buff.map[i][j] = recbuf[img_count];
                  img_count++;
                }
        }//     这个for结束，存了一副图像
          //此处添加发送函数
        total_image_num = total_image_num - 1;
        
        mLCD_num(56,80,(int)(total_image_num),4,RED,GREEN);
    }
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