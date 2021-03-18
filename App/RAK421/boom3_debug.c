#include "boom3_debug.h"
volatile 	rak_intStatus		rak_strIntStatus;
uint8 	 	Send_RecieveDataFlag=RAK_FALSE;
uint32 	 	DIST_IP;
extern int init_wifi_module(void);
//WIFI_BUFF wifi_buff={ { { 'm', 'a', 'p', '0' }, Y*X / 8 }, { 0 }, 0x1B };;
SECTOR_SAVE_BUFF SAB={0};
extern 		rak_CmdRsp	 uCmdRspFrame;
WIFI_BUFF send[3];
int num=300;
void wifi_msg()
{
    if(1)
    {			 
           static uint8 n=0;
           --Send_RecieveDataFlag;
           send[n++]=SAB.wifi_buff;
           if(n==3)
           {
//           rak_send_data(0,0,uCmdRspFrame.recvFrame.socket_flag, sizeof(mapSendBuff),(uint8*)&mapSendBuff);
             rak_send_data(0,0,uCmdRspFrame.recvFrame.socket_flag, sizeof(send),(uint8*)send);
             n=0;
           }

    }
    if((gpio_get(INT_PIN)))
      if(rak_checkPktIrq() == RAK_TRUE)
      {
              rak_read_packet(&uCmdRspFrame);
              rak_clearPktIrq();
              if(uCmdRspFrame.rspCode[0]==RSPCODE_RECV_DATA)
              {
               //  Send_RecieveDataFlag=200;			
              }
      }
}
void wifi_wait_for_response()
{
  do
  {
    if((gpio_get(INT_PIN)))
          if(rak_checkPktIrq() == RAK_TRUE)
          {
                  rak_read_packet(&uCmdRspFrame);
                  rak_clearPktIrq();
                  if(uCmdRspFrame.rspCode[0]==RSPCODE_RECV_DATA)
                  {
                     break;			
                  }
          }
  }while(1);
}

void SAB_init()
{
  
}








/************************************/
void rak421_init()
{
    gpio_init(RESET_PIN,GPO,0);
    gpio_init(INT_PIN,GPI,0);
    spi_init(WIFI_SPI, WIFI_CS, MASTER,12500*1000);
    gpio_set(RESET_PIN,0);
    DELAY_MS(50);
    gpio_set(RESET_PIN,1);
    DELAY_MS(200);
    
    init_wifi_module();
    rak_open_socket(LOCAL_PORT,DIST_PORT,RAK_MODULE_SOCKET_MODE,(uint32_t)DIST_IP);
}
void SYS_Delay(uint32_t us)
{
  DELAY_US(us);
}
