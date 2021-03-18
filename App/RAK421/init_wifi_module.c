
#include "include.h"
#include "rak_config.h"
#include "rak_hal.h"
#include "rak_init_module.h"

rak_api rak_strapi;
rak_CmdRsp	 uCmdRspFrame;


int init_wifi_module(void)
{
	int i=0;
	int retval=0;


	rak_init_struct(&rak_strapi);


	while(!(gpio_get (INT_PIN)));
	rak_sys_init(&uCmdRspFrame);
//	for(i = 0; i < 18; i++)
//	{
//		#if 1 //RAK_DEBUG_PRINT
//       // DPRINTF("%c",uCmdRspFrame.initResponse.strdata[i]);	
//		#endif
//	} 
	
//	retval=  rak_get_version();
//	if(retval!=RUN_OK)
//  	{
//		
//    	return retval;
//  	}
//  	else
//  	{
//                       
//	    RAK_RESPONSE_TIMEOUT(RAK_GETVERSION_TIMEOUT);
//	    rak_read_packet(&uCmdRspFrame);
//		rak_clearPktIrq();
//  	}
	 
//	retval=rak_uscan(&rak_strapi.uScanFrame);
//    if(retval!=RUN_OK)
//  	{
//	
//    	 return retval;
//  	}
//  	else
//  	{
//            
//	    RAK_RESPONSE_TIMEOUT(RAK_SCAN_IMEOUT);
//	    rak_read_packet(&uCmdRspFrame);
//		rak_clearPktIrq();
//  	}
//	retval= rak_getscan(&rak_strapi.uGetscan);
//	if(retval!=RUN_OK)
//  	{
//	
//    	return retval;
//  	}
//  	else
//  	{
//      
//	    RAK_RESPONSE_TIMEOUT(RAK_GETSCAN_IMEOUT);
//	    rak_read_packet(&uCmdRspFrame);
//		rak_clearPktIrq();
//  	} 
////	 
	 
	retval=rak_set_psk(&rak_strapi.uPskFrame);
	if(retval!=RUN_OK)
  	{
		
    	return retval;
  	}
  	else
  	{
                       
	    RAK_RESPONSE_TIMEOUT(RAK_SETPSK_TIMEOUT);
	    rak_read_packet(&uCmdRspFrame);

		rak_clearPktIrq();
  	}  	
          retval= rak_connect(&rak_strapi.uConnFrame);
          if(retval!=RUN_OK)
          {
                  
          return retval;
          }
          else
          {
          
              RAK_RESPONSE_TIMEOUT(RAK_CONNECT_TIMEOUT);
              rak_read_packet(&uCmdRspFrame);
             // flag= uCmdRspFrame.CmdRspBuf[2];
                  rak_clearPktIrq();
          }  
	retval=rak_get_net_status();
	if(retval!=RUN_OK)
  	{
		
    	return retval;
  	}
  	else
  	{
        
	    RAK_RESPONSE_TIMEOUT(RAK_GETNET_TIMEOUT);
	    rak_read_packet(&uCmdRspFrame);
		rak_clearPktIrq();
  	}
	retval= rak_set_ipstatic(&rak_strapi.uIpstaticFrame) ;
	if(retval!=RUN_OK)
  	{
	
    	return retval;
  	}
  	else
  	{
         
	    RAK_RESPONSE_TIMEOUT(RAK_IPSTATIC_TIMEOUT);
	    rak_read_packet(&uCmdRspFrame);
		rak_clearPktIrq();
  	}
        //////////////
        retval = rak_ipconfig_dhcp(RAK_DHCP_MODE) ;
	if (retval!=RUN_OK)
	{
	    return retval;
	}
	else 
	{
	    RAK_RESPONSE_TIMEOUT(RAK_CONNECT_TIMEOUT);
	    rak_read_packet(&uCmdRspFrame);
		if (uCmdRspFrame.ipparamFrameRcv.status==0)
		{	
			printf ("IP=");
			for (i=3;i>=0;i--)
			{
			printf ("%d",uCmdRspFrame.ipparamFrameRcv.ipaddr[i]);
			if (i<3)
			   {  printf (".");  }
			}
			  printf ("\r\n")	;
        printf ("DHCP OK!\r\n");
		}
		  rak_clearPktIrq();
	}


	return retval; 		
}





int Poll_ReadData_test(void)
{
	int retval;
	int i=0;
	char socket_flag_list[8]={0};
	char socket_num=0;

	retval=rak_set_funbitmap(RAK_POLLDATA_ENABLE);
	if(retval!=RUN_OK)
  	{
		return retval;
    	
  	}
  	else
  	{
                      
	    RAK_RESPONSE_TIMEOUT(RAK_SETFUNCBITMAP_TIMEOUT);
	    rak_read_packet(&uCmdRspFrame);
		rak_clearPktIrq();
		
  	}
	
	rak_open_socket(25000,0,RAK_SOCKET_TCP_SEVER,0);
	rak_open_socket(25001,0,RAK_SOCKET_TCP_SEVER,0);
	rak_open_socket(24000,12345,RAK_SOCKET_TCP_CLIENT,0xc0a80b65);
	
	
	while(1)
    { 
#ifdef RAK_DEBUG_PRINT
		printf("\r\n\r\n");
#endif
		for(i=0;i<socket_num;i++)
		{
			retval=rak_poll_read_data(socket_flag_list[i],&uCmdRspFrame); 			  
			if(retval==RUN_OK)
			{
				if((uCmdRspFrame.rspCode[0]==POLL_RECVDATA))
				{
					if(uCmdRspFrame.mgmtResponse.status==0xfe)
					{
					#ifdef RAK_DEBUG_PRINT
						 printf("EVE:socket_fd=%d,not data\r\n",socket_flag_list[i]);
					#endif
					}
					else
					{
					#ifdef RAK_DEBUG_PRINT
						 printf("status out\r\n");
					#endif
					}
				}
				else if((uCmdRspFrame.rspCode[0]==RSPCODE_NET_DISC))
			    {
				#ifdef RAK_DEBUG_PRINT
			   		printf("EVE:socket_fd=%d,net disc\r\n",socket_flag_list[i]);	
			    #endif
				}
			    else  if((uCmdRspFrame.rspCode[0]==RSPCODE_SOCKET_CLOSE))
			    {
				#ifdef RAK_DEBUG_PRINT						
			    	printf("EVE:socket_fd=%d,socket close\r\n",socket_flag_list[i]);
				#endif	
					memcpy(&socket_flag_list[i],&socket_flag_list[i+1],8-i);
					socket_num--;		
			    }
				else  if((uCmdRspFrame.rspCode[0]==RSPCODE_SOCKET_SVR))
			    {
					socket_flag_list[socket_num]=uCmdRspFrame.recvsocketEst.socket_flag;
				#ifdef RAK_DEBUG_PRINT
			    	printf("EVE:socket_fd=%d,tcp sever svr fd=%d\r\n",socket_flag_list[i],uCmdRspFrame.recvsocketEst.socket_flag);		
			    #endif	
					socket_num++;
				}
				else  if((uCmdRspFrame.rspCode[0]==RSPCODE_RECV_DATA))
			    {
				#ifdef RAK_DEBUG_PRINT
			    	printf("EVE:socket_fd=%d,recv socket fd=%d,datalen=%d\r\n",socket_flag_list[i],uCmdRspFrame.recvFrame.socket_flag,
											uCmdRspFrame.recvFrame.data_len);		
			    #endif
				}				
	
			}
		}

	}


}







