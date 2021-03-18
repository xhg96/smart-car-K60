#include "include.h"
#include "Img2sd.h"
#include "ff.h"
#include "VCAN_camera.h"       //����ͷ��ͷ�ļ�
#include "deal_img.h"
#include "motor.h"
#include "boom3_debug.h"
#include "diskio.h"

extern uint32 whileTime;
extern uint8 basemap[Y][X], top;
extern uint16 absde[3];

IMAGE2SDINFO Image2sd;


/**********************************************
���ܣ�sd������ʼ������ʼ����ʽΪfat�ļ�ϵͳ
������ÿ�γ�ʼ���ɹ���ᴴ��һ���ļ�����д����ͼ�񳤿�
return�� void
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
//        sprintf(myfilename, "0:/FIRE%d.SD", Imag_num); //ת��Ϊ�ַ�����������myfilename�ÿһ��SD��ʼ�����½�һ���ļ�
//		
//        res = f_open( &Image2sd.file, myfilename, FA_CREATE_NEW | FA_WRITE);
//        
//        /*
//		if( res == FR_DISK_ERR)
//		{
//        	printf( "\nû����SD��??\n" );
//        	return;
//   	 	}
//    	else if ( res == FR_OK )
//    	{
//        	printf( "\n�ļ��򿪳ɹ� \n" );
//    	}
//    	else
//    	{
//        	printf("\n����ֵ�쳣");
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
//    }while(res == FR_EXIST);        //����ļ����ڣ�������������1
//
//    if ( res == FR_OK )//��ʼ���ɹ�
//    {
//        //uint16  imgsize[] = {75,40};      //�ߡ�������ͷ��ɫ����
//        //uint32  mybw;
//        //res = f_write(&vcansrc, imgsize, sizeof(imgsize), &mybw);  //��д��ߺͿ�������λ������
//      
//        Image2sd.flag_disk2file = 1; 
//        //Image2sd.sector = 4096;    //�ӵ�4096��������ʼ������
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
    
    Image2sd.stopCar_to_send_data = 0; //ͣ���źţ���ʼ����λ��������
    
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
//#define F_SYNC_TIME   25     //���� n �κ� �� ͬ��
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
 * ����   ��basemap������ת��Ϊһ��uint8 ����8�����ص�
 * ʱ��   17-5-19
 * ����   ���ѿ�
******************************************************************************/
void basemap2transport(uint8 * sendbuf, uint8 start_pos)
{
  	//uint8 sendbuf[75*5]; //��Ҫ���͵����ݱ�����buf����

	/* ��������ͼ��ѹ���������ϴ�����λ��*/
	uint8 data_t;
	
	for(int8 i = 0; i < 80; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
            //ÿ8�����ص�ѹ����1���ֽ��ﴫ��
			for(uint8 k = 0; k < 8; k++)
			{
                          if(basemap[i][j*8+k] == 0)    //��ɫ	  
                            data_t &= ~(0x80 >> k);  //��ɫ��0
                          else 
                            data_t |= 0x80 >> k;       //��ɫ��1
			}                             
			
			sendbuf[i*5 + j + start_pos] = data_t;
		}
	}
}

void basemap2dezip(uint8 * sendbuf, uint8 start_pos)//,uint8 * showmap[80][40])
{
  	//uint8 sendbuf[75*5]; //��Ҫ���͵����ݱ�����buf����

	/* ��������ͼ���ѹ���������ϴ�����λ��*/
	uint8 data_t;
	
	for(int8 i = 0; i < 75; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
                  data_t = sendbuf[i*5 + j + start_pos];
                  
			for(uint8 k = 0; k < 8; k++)
			{
//                          if(data_t & (0x80 >> k) == (0x80 >> k))       //��ɫ
//                            showmap[i][j*8+k] = 1;
//                          else
//                            showmap[i][j*8+k] = 0;
                          
//                          if(basemap[i][j*8+k] == 0)    //��ɫ	  
//                            data_t &= ~(0x80 >> k);  //��ɫ��0
//                          else 
//                            data_t |= 0x80 >> k;       //��ɫ��1
			}                             
			
			
		}
	}
}

/******************************************************************************
 * ���ܣ�����ͼƬ��SD����diskģʽ
 * ʱ�䣺17-5-19 
 * ���ߣ����ѿ�
******************************************************************************/
int send_data_to_SD_disk(void)
{
	uint8 sendbuf[500] = {33}; 

      if(Image2sd.SD_config_OK == 0)    //��ʼ��ʧ��
        return -1;
	
	basemap2transport(sendbuf, 0);  //ͼ��
	//������Ϣ
//	sendbuf[374+0] = BYTE1(SpeedInfo.aimSpeed);//Ŀ���ٶ�-��λ
//	sendbuf[374+1] = BYTE0(SpeedInfo.aimSpeed);//Ŀ���ٶ�-��λ
//	
//	sendbuf[374+2] = BYTE1(SpeedInfo.nowSpeed); //�������ٶ�ƽ��ֵ(��Ч��ǰ�ٶ�)--��λ
//	sendbuf[374+3] = BYTE0(SpeedInfo.nowSpeed); //������
//	
//	
//	sendbuf[374+6] = speed_type;		//�ٶȿ������ͱ�־��0��ʾ���ι�ʽѭ����1��ʾֱ�߼��٣�2��ʾ�������
//	
//	sendbuf[374+7] = BYTE1(Servo_PID.PID_Out);      //�������Կ�����
//	sendbuf[374+8] = BYTE0(Servo_PID.PID_Out);     //-500��500
//    
//	
//	sendbuf[374+31] = BYTE1(absde[0]);      //ƫ��
//	sendbuf[374+32] = BYTE0(absde[0]);
//
//	sendbuf[374+33] = BYTE0(top);
//	//sendbuf[379+34] = BYTE0(other_top);
//
//	sendbuf[374+42] = BYTE3(whileTime);     //whileѭ��ʱ��
//	sendbuf[374+43] = BYTE2(whileTime);
//	sendbuf[374+44] = BYTE1(whileTime);
//	sendbuf[374+45] = BYTE0(whileTime);

      
    if( RES_OK == wait_for_SEND_STATUS_OK() )
    { 
        Image2sd.sector = Image2sd.sector + 2;
        
        disk_write_without_CMD13(0, sendbuf, Image2sd.sector, 1);  //ֱ��д����
    }
    
    return 0;
}

int SD_disk_2_RAK421(void){
    uint8 recbuf[40*80 / 8];    //ͼ���С
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
        
     disk_read(0, recbuf, Image2sd.sector, 1);       //������

        uint16 img_count = 0;
        for(int8 i = 0; i < 80; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
                  wifi_buff.map[i][j] = recbuf[img_count];
                  img_count++;
                }
        }//     ���for����������һ��ͼ��
          //�˴���ӷ��ͺ���
        total_image_num = total_image_num - 1;
        
        mLCD_num(56,80,(int)(total_image_num),4,RED,GREEN);
    }
}
/*!
 *  @brief     ��disk_write����write��SD�����ݣ�ͨ��disk_read����read������ Ȼ���ٴ浽�ļ���
 *  @time      17-6-25
 *  @author    cameling(���ѿ�)
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
        
        disk_read(0, recbuf, Image2sd.sector, 1);       //������
   
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

        recbuf[0] = 0xfc;              //ÿһ֡�Ŀ�ͷ
	recbuf[1] = 0x02;					
	
	recbuf[2] = 0x80;			 //�ȷ����ݳ���
	recbuf[3] = 3;		           //����
	
    uart_putbuff(UART0, recbuf, 4);
    
    mLCD_str(0,80,"    SEND OK    ", BLUE,GREEN);
    
    return 0;
}