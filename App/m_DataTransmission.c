#include "common.h"
#include "include.h"


/* �ⲿ���� */
extern uint16 Ultrasound_Distance[3];

extern uint32 Current_time;



/* ȫ�ֱ��� */
DMA_Data DMA_buffer = {0};    //DMA�������ݽṹ��
uint8 Data2ANO_Buffer[20];	//�������ݻ���

int sendNum = 0;      //����Ҫ���͵����ݴ�С
char espSend_CMD[19] = {0};   //ESP8266ģ�����ݷ���ǰ��ATָ��

uint8 RX_Buffer[RX_BUFFER_SIZE];	//���ڽ������ݻ���
uint8 TX_Buffer[TX_BUFFER_SIZE] = {'S','C'};	//���ڷ������ݻ���


/****************************************************************************** 
 *  @Function     DMAData_Init
 *  @Description  DMA���ڴ������ݳ�ʼ�� 
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
 *	@Description  ʹ��ATָ���ESP8266����WIFIģ���ʼ��
 *  @Callby       main.c
 ******************************************************************************/
void Esp8266_Init()
{
	char str[2] = {0};
    uart_init (ESP_UART, 115200);
    
    /* ����豸�Ƿ���� */
    uart_putstr(ESP_UART,"AT\r\n");
    
    uart_querystr(ESP_UART,str,2);
    
    if(str[0] == 'A' && str[1] == 'T')
    {
        mLCD_str(0,60,"ESP8266 Init ing...", BLUE,WHITE);
        
        DELAY_MS(500);    //��ʱ�𶯣��ǳ�ʼ���Ĺؼ���
        
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
    
    sendNum = sizeof(DMA_buffer);      //����Ҫ���͵����ݴ�С
    
    sprintf(espSend_CMD,"AT+CIPSEND=0,%d\r\n",sendNum);
}


void Esp8266_SendData()
{
    static uint8 send_cnt = 0;
    static uint8 send_time = 0;

	/*
	 * ��������ͼ��ѹ���������ϴ�����λ��
	 */
	uint8 data_t;
	
	for(int8 i = 0; i < ROW_ADD; i++)
	{
		for(uint8 j = 0; j < 5; j++)
		{
            //ÿ8�����ص�ѹ����1���ֽ��ﴫ��
			for(uint8 k = 0; k < 8; k++)
			{
                if(basemap[i][j*8+k] == 0)    //��ɫ
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
            
            UART4_C2 &= ~UART_C2_TIE_MASK;    //�ش��ڷ����ж�(��FIFO��UART0��������������ȷ��������)
            
            uart_putstr(ESP_UART,espSend_CMD);  
            DELAY_US(300);
            
            UART4_C2 |= UART_C2_TIE_MASK;    //�����ڷ����жϣ���DMA����
            
#ifdef USE_DMA
            DMA_EN(UART_DMA_CH);
#endif   
        }
    }
}


/*********************************���ͺ���*************************************
 * ����    ����Ŀ���ٶȺ�ʵ���ٶȵ���λ��
 * aspeed  Ϊ��ǰĿ���ٶ�
 * nspeed  Ϊ��ǰʵ���ٶ�
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
    
	uart_putbuff (UART4, Data2ANO_Buffer, _cnt);  //����
}


/******************************************************************************
 *  @brief      DMA��ʼ����UART�������ڴ����ݵ�����
 *  @param      DMA_CHn         ͨ���ţ�DMA_CH0 ~ DMA_CH15��
 *  @param      SADDR           Դ��ַ( uint8* )
 *  @param      uratn           ����UART
 *  @param      count           һ����ѭ�������ֽ���
 
 *  @note       DMA PTXn����ԴĬ�������ش������䣬�����޸ģ����ʼ������� port_init ����DMA ������ʽ
                ��ʼ������Ҫ���� DMA_EN ��ʵ��
 ******************************************************************************/
void dma_buff2uartx_init(DMA_CHn CHn, void *SADDR, UARTn_e uratn, uint32 count)
{
    ASSERT(count < 0x8000); //���ԣ����ֻ֧��0x7FFF(0~14)

	//UART �Ĵ��� ����
    UART_C2_REG(UARTN[uratn]) |= UART_C2_TIE_MASK;		//�����жϻ�DMA��������ʹ��
	UART_C5_REG(UARTN[uratn]) |= UART_C5_TDMAS_MASK;	//����ΪDMA����ʹ��

    //DMA �Ĵ��� ����
    
    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��

#if defined(MK60DZ10)
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //��DMA��·������ʱ��
#elif defined(MK60F15)
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX0_MASK;                    //��DMA��·������ʱ��
#endif

	/*�ȹر�ͨ��X������DMA����*/
	DMA_DIS(CHn);

    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (uint32)SADDR;                      // ����  Դ��ַ
    DMA_DADDR(CHn) =    (uint32)&UART_D_REG(UARTN[uratn]);  // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    0x01u;                              // ����Դ��ַƫ�� = 0x01, ��ÿ�δ�����ɺ��ַ��1
    DMA_DOFF(CHn)  =    0x00u;                              // ÿ�δ����Ŀ�ĵ�ַ����
    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)               // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(DMA_BYTE1)        // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)               // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(DMA_BYTE1)        // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                         );

    DMA_CITER_ELINKNO(CHn) = DMA_CITER_ELINKNO_CITER(count);   //������ѭ�������� current major loop count��ѭ������
    DMA_BITER_ELINKNO(CHn) = DMA_BITER_ELINKNO_BITER(count);   //��ʼѭ��������������ѭ����������СΪ���ʱ�����¼�����ʼѭ����������ֵ

    DMA_CR &= ~DMA_CR_EMLM_MASK;    //��ʹ�ܸ�ѭ��ӳ��

    //��CR[EMLM] = 0 ʱ:
    DMA_NBYTES_MLNO(CHn) = DMA_NBYTES_MLNO_NBYTES(1);  // ͨ��ÿ�δ����ֽ�������������Ϊ1���ֽڡ�ע��ֵΪ0��ʾ����4GB */

    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   (uint32)(-count);    //����Դ��ַ�ĸ���ֵ,��ѭ��������ָ�Դ��ַ
    DMA_DLAST_SGA(CHn)  =   0;                   //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
    DMA_CSR(CHn)        =   (0
                             //| DMA_CSR_BWC(3)           //�������,ÿ��һ�Σ�eDMA ����ֹͣ 8 �����ڣ�0��ֹͣ��1������2ֹͣ4���ڣ�3ֹͣ8���ڣ�
                             | DMA_CSR_DREQ_MASK          //��ѭ��������ֹͣӲ������
                             //| DMA_CSR_INTMAJOR_MASK    //��ѭ������������ж�
                            );

    /* ���� DMA ����Դ */
#if defined(MK60DZ10)
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
#elif defined(MK60F15)
    DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR, CHn) = (0
#endif
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            //| DMAMUX_CHCFG_TRIG_MASK                      /* Trigger Mode: Periodic   PIT���ڴ�������ģʽ   ͨ��1��ӦPIT1������ʹ��PIT1����������Ӧ��PIT��ʱ���� */
            | DMAMUX_CHCFG_SOURCE(DMA_UART0_Tx + 2*uratn)   /* ͨ����������Դ */
                                             );

    DMA_DIS(CHn);          //��ʹ��ͨ��CHnӲ������
    DMA_IRQ_CLEAN(CHn);

    /* �����ж� */
    //DMA_EN(CHn);        //ʹ��ͨ��CHn Ӳ������
    //DMA_IRQ_EN(CHn);    //����DMAͨ����������ж�
}
     
/******************************************************************************
* @Function     CheckConnection
* @Description  ˫��ģʽ��Ҫ��������
* @Input        
* @Output       
* @From         ��ʮһ��˫�� ���㽭��ҵ��ѧ
* @Callby       main()
******************************************************************************/
void CheckConnection()
{
	// ˫��ģʽ�·��������ź�
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
                    uart_putchar(BT_PORT,'C');  //���Ͷ��ȷ�����յ���Ӧ��
				
                    break;
                }
			}
		}
	}
}

/******************************************************************************
* @Function     Rec_CommunicationData
* @Description  ��������֡
* @Input        data_t:���δ��ڽ��յ�����
* @Output       
* @From         ��ʮһ��˫�� ���㽭��ҵ��ѧ
* @Callby       UART0_IRQHandler
******************************************************************************/
void Rec_CommunicationData(uint8 data_t)
{ 
	static uint8 state = 0;
	static uint8 checkSum = 0;

	RX_Buffer[state] = data_t;

	// ֡ͷ
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
	// ��Ч����
	else 
	{
        if (state == 2)
        {
            checkSum = 0;  //У���
        }
		state++;

		// ǰ����������
		if(DoubleCar_Info.car_ID == LEADER_CAR)
		{
            if(state >= 3 + 3)  //һ֡���ս���(3Bytes֡ + 3Bytes����)
			{
            	if(RX_Buffer[state-1] == checkSum)
            	{
					DoubleCar_Info.follower_IntegralDistance = (RX_Buffer[2]<<8) + RX_Buffer[3];  //�󳵻��־���

					//���ճ�����ɱ�־λ
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
		// �󳵽�������
		else if(DoubleCar_Info.car_ID == FOLLOW_CAR)
		{
			if(state >= 3 + 4)  //һ֡���ս���(3Bytes֡ + 4Bytes����)
			{
            	if(RX_Buffer[state-1] == checkSum)
            	{
					
                    DoubleCar_Info.distance_cm = (RX_Buffer[2]<<8) + RX_Buffer[3];  //ǰ��������������

					//���ն�ʧ�������źű�־λ
					if(RX_Buffer[4] & LOST_ULTRASOUND_MASK)
						DoubleCar_Info.flag1 |= LOST_ULTRASOUND_MASK;
					else
						DoubleCar_Info.flag1 &= ~LOST_ULTRASOUND_MASK;

					//����׼��������־λ
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
* @Description  ���ͽ�������
* @Input        
* @Output       
* @From         ��ʮһ��˫�� ���㽭��ҵ��ѧ
* @Callby       UART0_IRQHandler
******************************************************************************/
void Send_CommunicationData()
{
	uint8 checkSum_t = 0;
	uint8 BT_ValidData_LEN = 0;

	// ǰ����������
	if(DoubleCar_Info.car_ID == LEADER_CAR)
	{
		BT_ValidData_LEN = 4;
                
		// �ȷ���8λ���ٷ���8λ
		TX_Buffer[2] = BYTE1(DoubleCar_Info.distance_cm);
		TX_Buffer[3] = BYTE0(DoubleCar_Info.distance_cm);
		TX_Buffer[4] = DoubleCar_Info.flag1;
		TX_Buffer[5] = DoubleCar_Info.flag_tracktype;
		
		for(uint8 i = 2; i < BT_ValidData_LEN+2; i++)
		{
			checkSum_t ^= TX_Buffer[i];  //����У���
		}
		
		TX_Buffer[BT_ValidData_LEN+2] = checkSum_t;

		uart_putbuff(BT_PORT,TX_Buffer,BT_ValidData_LEN+3);
	}
	// �󳵷�������
	else if(DoubleCar_Info.car_ID == FOLLOW_CAR)
	{
		BT_ValidData_LEN = 3;
			
		// �ȷ���8λ���ٷ���8λ
		TX_Buffer[2] = BYTE1(DoubleCar_Info.follower_IntegralDistance);
		TX_Buffer[3] = BYTE0(DoubleCar_Info.follower_IntegralDistance);
		TX_Buffer[4] = DoubleCar_Info.flag1;
		
		for(uint8 i = 2; i < BT_ValidData_LEN+2; i++)
		{
			checkSum_t ^= TX_Buffer[i];  //����У���
		}
		
		TX_Buffer[BT_ValidData_LEN+2] = checkSum_t;

		uart_putbuff(BT_PORT,TX_Buffer,BT_ValidData_LEN+3);
	}
}

void UART0_IRQHandler()
{
    if ( UART0_S1 & UART_S1_RDRF_MASK )    //�������ݼĴ�����
    {
        //�û���Ҫ�����������
		uint8 uart_data = UART0_D;

		// ��������֡
		Rec_CommunicationData(uart_data);
    }
}
                                              
void UART4_IRQHandler()
{
    if ( UART4_S1 & UART_S1_RDRF_MASK )    //�������ݼĴ�����
    {
        //�û���Ҫ�����������
		uint8 uart_data = UART4_D;
        
        if(uart_data == 0x23) //���ڽ����ݷ������ȴ���Ӧ
            Image2sd.uart1_rec_busy = 0;
        
        if(uart_data == 0x24) //�յ��˿������ͣ���źţ�ͣ������ʼ����λ��������
            Image2sd.stopCar_to_send_data = 1;  
        
    }
}


