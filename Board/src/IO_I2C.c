/* IOI2C.c file
���ܣ�
�ṩI2C�ӿڲ���API ��
ʹ��IOģ�ⷽʽ
------------------------------------
*/

#include "common.h"
#include "MK60_gpio.h"
#include "IO_I2C.h"


static int IIC_Start(void);				
static void IIC_Stop(void);	  	
static uint8 IIC_Wait_Ack(void); 				
static void IIC_Ack(void);					
static void IIC_NAck(void);	

static void IIC_Send_Byte(uint8 txd);			
static uint8 IIC_Read_Byte(unsigned char ack);


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
void IIC_Init(void)
{			
    gpio_init  (I2C_SCL, GPO, 1); //��ʼ��SCL0
    gpio_init  (I2C_SDA, GPO, 1); //��ʼ��SDA0
    
	//��©�������������
    port_init_NoALT(I2C_SCL, ODO | PULLUP);
    port_init_NoALT(I2C_SDA, ODO | PULLUP);
}

/*******************************************************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
int IIC_Start(void)
{
	I2C_SDA_DDR_OUT();    //SDA�����     
    
	//START:when CLK is high,DATA change form high to low 
	I2C_SDA_H();	  	  
	I2C_SCL_H();
	I2C_DELAY();
	
 	I2C_SDA_L();
	I2C_DELAY();
	
	I2C_SCL_L();	//ǯסI2C���ߣ�׼�����ͻ��������
    
    return 1;
}

/*******************************************************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    ����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
	I2C_SDA_DDR_OUT();    //SDA����� 
    
	//STOP:when CLK is high DATA change form low to high
	I2C_SCL_L();
	I2C_SDA_L();    
 	I2C_DELAY();
	
	I2C_SCL_H(); 
	I2C_SDA_H();    //����I2C���߽����ź�
	I2C_DELAY();							   	
}

/*******************************************************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
*�� �� ֵ:      0������Ӧ��ʧ��
*               1������Ӧ��ɹ�
*******************************************************************************/
uint8 IIC_Wait_Ack(void)
{
	uint8 ucErrTime = 0;
	
	I2C_SDA_DDR_IN();    //SDA����Ϊ����  
	
	I2C_SDA_H();
	I2C_DELAY();	   
    
	I2C_SCL_H();
    I2C_DELAY();	 
    
	while( I2C_SDA_IN() )
	{
		ucErrTime++;
		
		if(ucErrTime > 50)
		{
			IIC_Stop();
			return 0;
		}
		
	    I2C_DELAY();
	}
	
	I2C_SCL_L();    //ʱ�����0    
	
	return 1;  
} 

/*******************************************************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
	I2C_SCL_L();
	
	I2C_SDA_DDR_OUT();    //SDA����Ϊ���  
	
	I2C_SDA_L();
	I2C_DELAY();
	
	I2C_SCL_H();
	I2C_DELAY();
	
	I2C_SCL_L();
}

/*******************************************************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
	I2C_SCL_L();
	
	I2C_SDA_DDR_OUT();    //SDA����Ϊ���  
	
	I2C_SDA_H();
	I2C_DELAY();
	
	I2C_SCL_H();
	I2C_DELAY();
	
	I2C_SCL_L();
}					 				     

/*******************************************************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t = 8;
	
    I2C_SDA_DDR_OUT(); 	    
	I2C_SCL_L();    //����ʱ�ӿ�ʼ���ݴ���
    
	while(t--)
    {
        if(txd & 0x80)
        {
            I2C_SDA_H();
        }
        else
        {
            I2C_SDA_L();
        }
		
        txd <<= 1; 
        I2C_DELAY();   
        
		I2C_SCL_H();
	    I2C_DELAY(); 
        
		I2C_SCL_L();	
	    I2C_DELAY();
    }
} 	 

/*******************************************************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    ��1���ֽڣ�
*               ack=1ʱ������ACK; 
*               ack=0������nACK 
*******************************************************************************/ 
uint8 IIC_Read_Byte(unsigned char ack)
{
    uint8 i,receive = 0;
	
    I2C_SDA_DDR_IN();    //SDA����Ϊ����
	
    for(i=0;i<8;i++ )
    {
        I2C_SCL_L(); 
        I2C_DELAY();
		
	    I2C_SCL_H();
        receive <<= 1;
		
        if( I2C_SDA_IN() )
			receive++;   
		
	    I2C_DELAY(); 
    }		
	
    if (ack)
        IIC_Ack();    //����ACK 
    else
        IIC_NAck();    //����nACK
	
    return receive;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
*��    ��:		dev  Ŀ���豸��ַ
				reg	 �Ĵ�����ַ
*��    ��:   	��������ֵ
*******************************************************************************/ 
uint8 I2C_ReadOneByte(uint8 dev,uint8 reg)
{
	uint8 res = 0;
	
	IIC_Start();	
	IIC_Send_Byte(dev);  //����д����	   
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg);	//���͵�ַ 
	IIC_Wait_Ack();	  
	
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ 
	IIC_Wait_Ack();
	
	res = IIC_Read_Byte(0);	   
	
    IIC_Stop();  //����һ��ֹͣ����
    
	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IIC_WriteOneByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
*��    ��:		dev  Ŀ���豸��ַ
				reg	   �Ĵ�����ַ
				data  ��Ҫд����ֽ�
*��    ��:	    1:ʧ��  0:�ɹ�
*******************************************************************************/
uint8 IIC_WriteOneByte(uint8 dev, uint8 reg, uint8 data)
{
    IIC_Start(); 
	IIC_Send_Byte(dev);   //����������ַ+д����	
    
	if(!IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
	
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    
	IIC_Send_Byte(data);//��������
	if(!IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
	
    IIC_Stop();	 
	return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_ReadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
*��    ��:		dev  Ŀ���豸��ַ
				reg	  �Ĵ�����ַ
				length Ҫ�����ֽ���
				*data  ���������ݽ�Ҫ��ŵ�ָ��
*��    ��:  	���������ֽ�����
*******************************************************************************/
uint8 IIC_ReadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data)
{
    uint8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);    //����д����	   
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg);    //���͵�ַ
    IIC_Wait_Ack();	 
	
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ  
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++)
    {
        if(count!=length-1)
            data[count]=IIC_Read_Byte(1);  
        else  
            data[count]=IIC_Read_Byte(0);	 
	}
    
    IIC_Stop();    //����һ��ֹͣ����
    return count;
}

/*******************************************************************************
*����ԭ��:		u8 IIC_WriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
*��	   ��:		dev  Ŀ���豸��ַ
				reg	  �Ĵ�����ַ
				length Ҫд���ֽ���
				*data  ��Ҫд�����ݵ��׵�ַ
*��    ��:	   	1�ɹ�
*******************************************************************************/ 
uint8 IIC_WriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data)
{
 	uint8 count = 0;
    
	IIC_Start();
	
	IIC_Send_Byte(dev);	   
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg);   
    IIC_Wait_Ack();	  
	
	for(count=0; count < length; count++)
    {
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	}
    
	IIC_Stop();
    
    return 1; //status == 0;
}

/*******************************************************************************
*����ԭ��:		u8 IIC_ReadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
*��    ��: 		dev  Ŀ���豸��ַ
				reg	   �Ĵ�����ַ
				*data  ���������ݽ�Ҫ��ŵĵ�ַ
*******************************************************************************/
uint8 IIC_ReadByte(uint8 dev, uint8 reg, uint8 *data)
{
	*data = I2C_ReadOneByte(dev, reg);
    return 1;
}



/*******************************************************************************
*����ԭ��:		u8 IIC_WriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
*��    ��:   	dev  Ŀ���豸��ַ
				reg	   �Ĵ�����ַ
				bitStart  Ŀ���ֽڵ���ʼλ
				length   λ����
				data    ��Ÿı�Ŀ���ֽ�λ��ֵ
*��    ��:  	�ɹ� Ϊ1; ʧ��Ϊ0
*******************************************************************************/
uint8 IIC_WriteBits(uint8 dev,uint8 reg,uint8 bitStart,uint8 length,uint8 data)
{
    uint8 b;
    
    if (IIC_ReadByte(dev, reg, &b) != 0) 
    {
        uint8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IIC_WriteOneByte(dev, reg, b);
    }
    else 
    {
        return 0;
    }
}

/*******************************************************************************
*����ԭ��:		u8 IIC_WriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
*��    ��:	    dev     Ŀ���豸��ַ
				reg	    �Ĵ�����ַ
				bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
				data Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
*��    ��:   	�ɹ� Ϊ1; ʧ��Ϊ0
*******************************************************************************/ 
uint8 IIC_WriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data)
{
    uint8 b;
    
    IIC_ReadByte(dev, reg, &b);
    
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return IIC_WriteOneByte(dev, reg, b);
}


/*******************************************************************************
*����ԭ��:		bool DMP_i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		����DMP��д��һ�����ȵ��ֽ�
*				����1:д��ʧ��
*				����0:д��ɹ�
*******************************************************************************/
int DMP_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    
    if (!IIC_Start())
        return 1;
    
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) 
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    
    for (i = 0; i < len; i++) 
    {
        IIC_Send_Byte(data[i]);
        
        if (!IIC_Wait_Ack()) 
        {
            IIC_Stop();
            return 0;
        }
    }
    
    IIC_Stop();
    
    return 0;
}

/*******************************************************************************
*����ԭ��:		bool DMP_i2cRead(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		����DMP���ȡһ�����ȵ��ֽ�
*				����1:��ȡʧ��
*				����0:��ȡ�ɹ�	
*******************************************************************************/
int DMP_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) 
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) 
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    
    IIC_Stop();
    
    return 0;
}

//------------------End of File----------------------------
