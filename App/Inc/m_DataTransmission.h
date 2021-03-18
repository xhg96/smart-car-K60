#ifndef _DATA_TRANSMISSION_H_1
#define _DATA_TRANSMISSION_H_1



//#define USE_DMA (1)

#define  ESP_UART     UART4
#define  UART_DMA_CH  DMA_CH1

// USART Receiver buffer
#define RX_BUFFER_SIZE   10
#define TX_BUFFER_SIZE   10


/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


typedef struct 
{
	uint8 cmdf[2];
	uint8 img_data[ROW_ADD*5*5];
	uint8 cmdr[2];
}DMA_Data;


extern unsigned char RX_Buffer[RX_BUFFER_SIZE];
extern unsigned char TX_Buffer[TX_BUFFER_SIZE];

extern DMA_Data DMA_buffer;    //DMA�������ݽṹ��


/*
 *  ���ⲿ���õĺ����ӿ�����
 */

extern void SendData2ANO(uint16 aspeed,int16 nspeed,int16 pwm);

extern void Send_CommunicationData();
extern void CheckConnection();
extern void UART0_IRQHandler();
extern void UART4_IRQHandler();


#endif
