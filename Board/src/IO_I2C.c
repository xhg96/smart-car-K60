/* IOI2C.c file
功能：
提供I2C接口操作API 。
使用IO模拟方式
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


/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{			
    gpio_init  (I2C_SCL, GPO, 1); //初始化SCL0
    gpio_init  (I2C_SDA, GPO, 1); //初始化SDA0
    
	//开漏输出，上拉电阻
    port_init_NoALT(I2C_SCL, ODO | PULLUP);
    port_init_NoALT(I2C_SDA, ODO | PULLUP);
}

/*******************************************************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
int IIC_Start(void)
{
	I2C_SDA_DDR_OUT();    //SDA线输出     
    
	//START:when CLK is high,DATA change form high to low 
	I2C_SDA_H();	  	  
	I2C_SCL_H();
	I2C_DELAY();
	
 	I2C_SDA_L();
	I2C_DELAY();
	
	I2C_SCL_L();	//钳住I2C总线，准备发送或接收数据
    
    return 1;
}

/*******************************************************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
	I2C_SDA_DDR_OUT();    //SDA线输出 
    
	//STOP:when CLK is high DATA change form low to high
	I2C_SCL_L();
	I2C_SDA_L();    
 	I2C_DELAY();
	
	I2C_SCL_H(); 
	I2C_SDA_H();    //发送I2C总线结束信号
	I2C_DELAY();							   	
}

/*******************************************************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
*返 回 值:      0，接收应答失败
*               1，接收应答成功
*******************************************************************************/
uint8 IIC_Wait_Ack(void)
{
	uint8 ucErrTime = 0;
	
	I2C_SDA_DDR_IN();    //SDA设置为输入  
	
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
	
	I2C_SCL_L();    //时钟输出0    
	
	return 1;  
} 

/*******************************************************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
	I2C_SCL_L();
	
	I2C_SDA_DDR_OUT();    //SDA设置为输出  
	
	I2C_SDA_L();
	I2C_DELAY();
	
	I2C_SCL_H();
	I2C_DELAY();
	
	I2C_SCL_L();
}

/*******************************************************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck(void)
{
	I2C_SCL_L();
	
	I2C_SDA_DDR_OUT();    //SDA设置为输出  
	
	I2C_SDA_H();
	I2C_DELAY();
	
	I2C_SCL_H();
	I2C_DELAY();
	
	I2C_SCL_L();
}					 				     

/*******************************************************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t = 8;
	
    I2C_SDA_DDR_OUT(); 	    
	I2C_SCL_L();    //拉低时钟开始数据传输
    
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
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    读1个字节，
*               ack=1时，发送ACK; 
*               ack=0，发送nACK 
*******************************************************************************/ 
uint8 IIC_Read_Byte(unsigned char ack)
{
    uint8 i,receive = 0;
	
    I2C_SDA_DDR_IN();    //SDA设置为输入
	
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
        IIC_Ack();    //发送ACK 
    else
        IIC_NAck();    //发送nACK
	
    return receive;
}


/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
*输    入:		dev  目标设备地址
				reg	 寄存器地址
*返    回:   	读出来的值
*******************************************************************************/ 
uint8 I2C_ReadOneByte(uint8 dev,uint8 reg)
{
	uint8 res = 0;
	
	IIC_Start();	
	IIC_Send_Byte(dev);  //发送写命令	   
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg);	//发送地址 
	IIC_Wait_Ack();	  
	
	IIC_Start();
	IIC_Send_Byte(dev+1);  //进入接收模式 
	IIC_Wait_Ack();
	
	res = IIC_Read_Byte(0);	   
	
    IIC_Stop();  //产生一个停止条件
    
	return res;
}


/**************************实现函数********************************************
*函数原型:		unsigned char IIC_WriteOneByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
*输    入:		dev  目标设备地址
				reg	   寄存器地址
				data  将要写入的字节
*返    回:	    1:失败  0:成功
*******************************************************************************/
uint8 IIC_WriteOneByte(uint8 dev, uint8 reg, uint8 data)
{
    IIC_Start(); 
	IIC_Send_Byte(dev);   //发送器件地址+写命令	
    
	if(!IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
	
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
    
	IIC_Send_Byte(data);//发送数据
	if(!IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
	
    IIC_Stop();	 
	return 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_ReadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
*输    入:		dev  目标设备地址
				reg	  寄存器地址
				length 要读的字节数
				*data  读出的数据将要存放的指针
*返    回:  	读出来的字节数量
*******************************************************************************/
uint8 IIC_ReadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data)
{
    uint8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);    //发送写命令	   
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg);    //发送地址
    IIC_Wait_Ack();	 
	
	IIC_Start();
	IIC_Send_Byte(dev+1);  //进入接收模式  
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++)
    {
        if(count!=length-1)
            data[count]=IIC_Read_Byte(1);  
        else  
            data[count]=IIC_Read_Byte(0);	 
	}
    
    IIC_Stop();    //产生一个停止条件
    return count;
}

/*******************************************************************************
*函数原型:		u8 IIC_WriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
*输	   入:		dev  目标设备地址
				reg	  寄存器地址
				length 要写的字节数
				*data  将要写的数据的首地址
*返    回:	   	1成功
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
*函数原型:		u8 IIC_ReadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
*输    入: 		dev  目标设备地址
				reg	   寄存器地址
				*data  读出的数据将要存放的地址
*******************************************************************************/
uint8 IIC_ReadByte(uint8 dev, uint8 reg, uint8 *data)
{
	*data = I2C_ReadOneByte(dev, reg);
    return 1;
}



/*******************************************************************************
*函数原型:		u8 IIC_WriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
*输    入:   	dev  目标设备地址
				reg	   寄存器地址
				bitStart  目标字节的起始位
				length   位长度
				data    存放改变目标字节位的值
*返    回:  	成功 为1; 失败为0
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
*函数原型:		u8 IIC_WriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
*输    入:	    dev     目标设备地址
				reg	    寄存器地址
				bitNum  要修改目标字节的bitNum位
				data 为0 时，目标位将被清0 否则将被置位
*返    回:   	成功 为1; 失败为0
*******************************************************************************/ 
uint8 IIC_WriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data)
{
    uint8 b;
    
    IIC_ReadByte(dev, reg, &b);
    
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return IIC_WriteOneByte(dev, reg, b);
}


/*******************************************************************************
*函数原型:		bool DMP_i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		用于DMP库写入一定长度的字节
*				返回1:写入失败
*				返回0:写入成功
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
*函数原型:		bool DMP_i2cRead(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		用于DMP库读取一定长度的字节
*				返回1:读取失败
*				返回0:读取成功	
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
