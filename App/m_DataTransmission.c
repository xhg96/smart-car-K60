#include "common.h"
#include "include.h"


/* 外部变量 */
extern uint16 Ultrasound_Distance[3];

extern uint32 Current_time;



/* 全局变量 */
DMA_Data DMA_buffer = {0};    //DMA传输数据结构体
uint8 Data2ANO_Buffer[20];	//发送数据缓存

int sendNum = 0;      //本次要传送的数据大小
char espSend_CMD[19] = {0};   //ESP8266模块数据发送前的AT指令

uint8 RX_Buffer[RX_BUFFER_SIZE];	//串口接收数据缓存
uint8 TX_Buffer[TX_BUFFER_SIZE] = {'S','C'};	//串口发送数据缓存


/****************************************************************************** 
 *  @Function     DMAData_Init
 *  @Description  DMA串口传输数据初始化 
 *  @Callby       main.c
 *****************************************************************************/
void DMAData_Init()
{
	DMA_buffer.cmdf[0] = 1;
    DMA_buffer.cmdf[1] = ~1;
    DMA_buffer.cmdr[0] = ~1;
    DMA_buffer.cmdr[1] = 1;
}

/******************************************************************************  
 *	@Function     Esp8266_Init
 *	@Description  使用AT指令对ESP8266串口WIFI模块初始化
 *  @Callby       main.c
 ******************************************************************************/
void Esp8266_Init()
{
	char str[2] = {0};
    uart_init (ESP_UART, 115200);
    
    /* 检测设备是否存在 */
    uart_putstr(ESP_UART,"AT\r\n");
    
    uart_querystr(ESP_UART,str,2);
    
    if(str[0] == 'A' && str[1] == 'T')
    {
        mLCD_str(0,60,"ESP8266 Init ing...", BLUE,WHITE);
        
        DELAY_MS(500);    //延时别动，是初始化的关键；
        
        uart_putstr(ESP_UART,"AT+CWMODE=2\r\n");
        
        DELAY_MS(600);
        uart_putstr(ESP_UART,"AT+RST\r\n");
        
        DELAY_MS(600);
        uart_putstr(ESP_UART,"AT+CIPMUX=1\r\n");
        //uart_putstr(ESP_UART,"AT+CIPMUX=0\r\n");
        
        DELAY_MS(500);
        uart_putstr(ESP_UART,"AT+CIPSERVER=1,8080\r\n");	
        
        mLCD_str(0,60,"ESP8266 Init OK!   ", BLUE,WHITE);
    }
    else
    {
        mLCD_str(0,60,"ESP8266 Init Error ", BLUE,WHITE);
    }
    
    sendNum = sizeof(DMA_buffer);      //本次要传送的数据大小
    
    sprintf(espSend_CMD,"AT+CIPSEND=0,%d\r\n",sendNum);
}


void Esp8266_SendData()
{
    static uint8 send_cnt = 0;
    static uint8 send_time = 0;

	/*
	 * 将处理后的图像压缩，用于上传到上位机
	 */
	uint8 data_t;
	
	for(int8 i = 0; i < ROW_ADD; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
            //每8个像素点压缩到1个字节里传输
			for(uint8 k = 0; k < 8; k++)
			{
                if(basemap[i][j*8+k] == 0)    //白色
					data_t &= ~(0x80 >> k);
                else
                    data_t |= 0x80 >> k;
			}                             
			
			DMA_buffer.img_data[send_cnt*ROW_ADD*5 + (i*5+j)] = data_t;
		}
	}
	
    send_cnt++;
    
    if(send_cnt >= 5)
    {  
        send_cnt = 0;
        
        send_time++;
        
        if(send_time >= 2)
        {    
            send_time = 0;   
            
            UART4_C2 &= ~UART_C2_TIE_MASK;    //关串口发送中断(带FIFO的UART0必须这样才能正确发出数据)
            
            uart_putstr(ESP_UART,espSend_CMD);  
            DELAY_US(300);
            
            UART4_C2 |= UART_C2_TIE_MASK;    //开串口发送中断，由DMA接手
            
#ifdef USE_DMA
            DMA_EN(UART_DMA_CH);
#endif   
        }
    }
}


/*********************************发送函数*************************************
 * 功能    发送目标速度和实际速度到上位机
 * aspeed  为当前目标速度
 * nspeed  为当前实际速度
******************************************************************************/
void SendData2ANO(uint16 aspeed,int16 nspeed,int16 pwm)
{
	uint32 _cnt=0;
	uint8 sum = 0;
    
	Data2ANO_Buffer[_cnt++] = 0xAA;
	Data2ANO_Buffer[_cnt++] = 0xAA;
	Data2ANO_Buffer[_cnt++] = 0xF1;
	Data2ANO_Buffer[_cnt++] = 0 ;
	
	Data2ANO_Buffer[_cnt++] = BYTE1(aspeed);
	Data2ANO_Buffer[_cnt++] = BYTE0(aspeed);
    
	Data2ANO_Buffer[_cnt++] = BYTE1(nspeed);
	Data2ANO_Buffer[_cnt++] = BYTE0(nspeed);
    
    Data2ANO_Buffer[_cnt++] = BYTE1(pwm);
	Data2ANO_Buffer[_cnt++] = BYTE0(pwm);
    
	Data2ANO_Buffer[3] = _cnt-4;

	for(uint8 i = 0; i < _cnt; i++)
		sum += Data2ANO_Buffer[i];
	
	Data2ANO_Buffer[_cnt++] = sum;
    
	uart_putbuff (UART4, Data2ANO_Buffer, _cnt);  //发送
}


/******************************************************************************
 *  @brief      DMA初始化，UART请求传输内存数据到串口
 *  @param      DMA_CHn         通道号（DMA_CH0 ~ DMA_CH15）
 *  @param      SADDR           源地址( uint8* )
 *  @param      uratn           触发UART
 *  @param      count           一个主循环传输字节数
 
 *  @note       DMA PTXn触发源默认上升沿触发传输，若需修改，则初始化后调用 port_init 配置DMA 触发方式
                初始化后，需要调用 DMA_EN 来实现
 ******************************************************************************/
void dma_buff2uartx_init(DMA_CHn CHn, void *SADDR, UARTn_e uratn, uint32 count)
{
    ASSERT(count < 0x8000); //断言，最大只支持0x7FFF(0~14)

	//UART 寄存器 配置
    UART_C2_REG(UARTN[uratn]) |= UART_C2_TIE_MASK;		//发送中断或DMA传输请求使能
	UART_C5_REG(UARTN[uratn]) |= UART_C5_TDMAS_MASK;	//配置为DMA请求使能

    //DMA 寄存器 配置
    
    /* 开启时钟 */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //打开DMA模块时钟

#if defined(MK60DZ10)
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //打开DMA多路复用器时钟
#elif defined(MK60F15)
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX0_MASK;                    //打开DMA多路复用器时钟
#endif

	/*先关闭通道X的外设DMA请求*/
	DMA_DIS(CHn);

    /* 配置 DMA 通道 的 传输控制块 TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (uint32)SADDR;                      // 设置  源地址
    DMA_DADDR(CHn) =    (uint32)&UART_D_REG(UARTN[uratn]);  // 设置目的地址
    DMA_SOFF(CHn)  =    0x01u;                              // 设置源地址偏移 = 0x01, 即每次传输完成后地址加1
    DMA_DOFF(CHn)  =    0x00u;                              // 每次传输后，目的地址不变
    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)               // 源地址模数禁止  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(DMA_BYTE1)        // 源数据位宽 ：DMA_BYTEn  。    SSIZE = 0 -> 8-bit ，SSIZE = 1 -> 16-bit ，SSIZE = 2 -> 32-bit ，SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)               // 目标地址模数禁止
                         | DMA_ATTR_DSIZE(DMA_BYTE1)        // 目标数据位宽 ：DMA_BYTEn  。  设置参考  SSIZE
                         );

    DMA_CITER_ELINKNO(CHn) = DMA_CITER_ELINKNO_CITER(count);   //设置主循环计数器 current major loop count的循环次数
    DMA_BITER_ELINKNO(CHn) = DMA_BITER_ELINKNO_BITER(count);   //起始循环计数器，当主循环计数器减小为零的时候，重新加载起始循环计数器的值

    DMA_CR &= ~DMA_CR_EMLM_MASK;    //不使能副循环映射

    //当CR[EMLM] = 0 时:
    DMA_NBYTES_MLNO(CHn) = DMA_NBYTES_MLNO_NBYTES(1);  // 通道每次传输字节数，这里设置为1个字节。注：值为0表示传输4GB */

    /* 配置 DMA 传输结束后的操作 */
    DMA_SLAST(CHn)      =   (uint32)(-count);    //调整源地址的附加值,主循环结束后恢复源地址
    DMA_DLAST_SGA(CHn)  =   0;                   //调整目的地址的附加值,主循环结束后恢复目的地址或者保持地址
    DMA_CSR(CHn)        =   (0
                             //| DMA_CSR_BWC(3)           //带宽控制,每读一次，eDMA 引擎停止 8 个周期（0不停止；1保留；2停止4周期；3停止8周期）
                             | DMA_CSR_DREQ_MASK          //主循环结束后停止硬件请求
                             //| DMA_CSR_INTMAJOR_MASK    //主循环结束后产生中断
                            );

    /* 配置 DMA 触发源 */
#if defined(MK60DZ10)
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
#elif defined(MK60F15)
    DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR, CHn) = (0
#endif
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            //| DMAMUX_CHCFG_TRIG_MASK                      /* Trigger Mode: Periodic   PIT周期触发传输模式   通道1对应PIT1，必须使能PIT1，且配置相应的PIT定时触发 */
            | DMAMUX_CHCFG_SOURCE(DMA_UART0_Tx + 2*uratn)   /* 通道触发传输源 */
                                             );

    DMA_DIS(CHn);          //不使能通道CHn硬件请求
    DMA_IRQ_CLEAN(CHn);

    /* 开启中断 */
    //DMA_EN(CHn);        //使能通道CHn 硬件请求
    //DMA_IRQ_EN(CHn);    //允许DMA通道传输完成中断
}
     
/******************************************************************************
* @Function     CheckConnection
* @Description  双车模式下要进行握手
* @Input        
* @Output       
* @From         第十一届双车 —浙江工业大学
* @Callby       main()
******************************************************************************/
void CheckConnection()
{
	// 双车模式下发送握手信号
	if(DoubleCar_Info.car_ID == FOLLOW_CAR)
	{
		char ack[10] = {0}; 

		mLCD_str(0,72,"Connecting...",RED,WHITE);
		mLCD_str(0,88,"I'm RearCar!",RED,WHITE);
		
		while(1)
		{
			//uart_putstr(BT_PORT, "SC");
			uart_putchar(BT_PORT, 'S');

            //uart_querybuff (BT_PORT,ack,1);
            
			if( UART0_RCFIFO )
            { 
                uart_getchar(BT_PORT,ack);
                
                if(ack[0] == 'C')
                    break;        
            }
		}
	}
	else if(DoubleCar_Info.car_ID == LEADER_CAR)
	{
		char req = 0;

		mLCD_str(0,72,"Connecting...",RED,WHITE);
		mLCD_str(0,88,"I'm FrontCar!",RED,WHITE);
		
		while(1)
		{
			//uart_querychar(BT_PORT, &req);

			if( UART0_RCFIFO )
			{
                uart_getchar(BT_PORT,&req);
                
                if (req == 'S')
				{
                    uart_putchar(BT_PORT,'C');  //发送多个确保能收到此应答
				
                    break;
                }
			}
		}
	}
}

/******************************************************************************
* @Function     Rec_CommunicationData
* @Description  解析数据帧
* @Input        data_t:本次串口接收的数据
* @Output       
* @From         第十一届双车 —浙江工业大学
* @Callby       UART0_IRQHandler
******************************************************************************/
void Rec_CommunicationData(uint8 data_t)
{ 
	static uint8 state = 0;
	static uint8 checkSum = 0;

	RX_Buffer[state] = data_t;

	// 帧头
	if(state < 2)
	{
		switch(state)
		{
			case 0:
			{
				if(RX_Buffer[state] == 'S')
					state++;
				break;
			}
			case 1:
			{
				if(RX_Buffer[state] == 'C')
					state++;
				else
					state = 0;
				break;
			}
		}
	}
	// 有效数据
	else 
	{
        if (state == 2)
        {
            checkSum = 0;  //校验和
        }
		state++;

		// 前车接收数据
		if(DoubleCar_Info.car_ID == LEADER_CAR)
		{
            if(state >= 3 + 3)  //一帧接收结束(3Bytes帧 + 3Bytes数据)
			{
            	if(RX_Buffer[state-1] == checkSum)
            	{
					DoubleCar_Info.follower_IntegralDistance = (RX_Buffer[2]<<8) + RX_Buffer[3];  //后车积分距离

					//接收超车完成标志位
					if(RX_Buffer[4] & FINISH_OVERTAKE_MASK)
						DoubleCar_Info.flag1 |= FINISH_OVERTAKE_MASK;
					else
						DoubleCar_Info.flag1 &= ~FINISH_OVERTAKE_MASK;
            	}
				
				state = 0;
			}
       		else
        	{
            	checkSum ^= RX_Buffer[state-1];
        	}
		}
		// 后车接收数据
		else if(DoubleCar_Info.car_ID == FOLLOW_CAR)
		{
			if(state >= 3 + 4)  //一帧接收结束(3Bytes帧 + 4Bytes数据)
			{
            	if(RX_Buffer[state-1] == checkSum)
            	{
					
                    DoubleCar_Info.distance_cm = (RX_Buffer[2]<<8) + RX_Buffer[3];  //前车超声波测距距离

					//接收丢失超声波信号标志位
					if(RX_Buffer[4] & LOST_ULTRASOUND_MASK)
						DoubleCar_Info.flag1 |= LOST_ULTRASOUND_MASK;
					else
						DoubleCar_Info.flag1 &= ~LOST_ULTRASOUND_MASK;

					//接收准备超车标志位
					if(RX_Buffer[4] & READY_OVERTAKE_MASK)
						DoubleCar_Info.flag1 |= READY_OVERTAKE_MASK;
					else
						DoubleCar_Info.flag1 &= ~READY_OVERTAKE_MASK;

					DoubleCar_Info.flag_tracktype = RX_Buffer[5];
            	}
				state = 0;
			}
        	else
        	{
            	checkSum ^= RX_Buffer[state-1];
        	}
		}
	}
}

/******************************************************************************
* @Function     Send_CommunicationData
* @Description  发送交互数据
* @Input        
* @Output       
* @From         第十一届双车 —浙江工业大学
* @Callby       UART0_IRQHandler
******************************************************************************/
void Send_CommunicationData()
{
	uint8 checkSum_t = 0;
	uint8 BT_ValidData_LEN = 0;

	// 前车发送数据
	if(DoubleCar_Info.car_ID == LEADER_CAR)
	{
		BT_ValidData_LEN = 4;
                
		// 先发高8位，再发低8位
		TX_Buffer[2] = BYTE1(DoubleCar_Info.distance_cm);
		TX_Buffer[3] = BYTE0(DoubleCar_Info.distance_cm);
		TX_Buffer[4] = DoubleCar_Info.flag1;
		TX_Buffer[5] = DoubleCar_Info.flag_tracktype;
		
		for(uint8 i = 2; i < BT_ValidData_LEN+2; i++)
		{
			checkSum_t ^= TX_Buffer[i];  //计算校验和
		}
		
		TX_Buffer[BT_ValidData_LEN+2] = checkSum_t;

		uart_putbuff(BT_PORT,TX_Buffer,BT_ValidData_LEN+3);
	}
	// 后车发送数据
	else if(DoubleCar_Info.car_ID == FOLLOW_CAR)
	{
		BT_ValidData_LEN = 3;
			
		// 先发高8位，再发低8位
		TX_Buffer[2] = BYTE1(DoubleCar_Info.follower_IntegralDistance);
		TX_Buffer[3] = BYTE0(DoubleCar_Info.follower_IntegralDistance);
		TX_Buffer[4] = DoubleCar_Info.flag1;
		
		for(uint8 i = 2; i < BT_ValidData_LEN+2; i++)
		{
			checkSum_t ^= TX_Buffer[i];  //计算校验和
		}
		
		TX_Buffer[BT_ValidData_LEN+2] = checkSum_t;

		uart_putbuff(BT_PORT,TX_Buffer,BT_ValidData_LEN+3);
	}
}

void UART0_IRQHandler()
{
    if ( UART0_S1 & UART_S1_RDRF_MASK )    //接收数据寄存器满
    {
        //用户需要处理接收数据
		uint8 uart_data = UART0_D;

		// 解析数据帧
		Rec_CommunicationData(uart_data);
    }
}
                                              
void UART4_IRQHandler()
{
    if ( UART4_S1 & UART_S1_RDRF_MASK )    //接收数据寄存器满
    {
        //用户需要处理接收数据
		uint8 uart_data = UART4_D;
        
        if(uart_data == 0x23) //串口将数据发出，等待回应
            Image2sd.uart1_rec_busy = 0;
        
        if(uart_data == 0x24) //收到了开发板的停车信号，停车，开始向上位机发数据
            Image2sd.stopCar_to_send_data = 1;  
        
    }
}


