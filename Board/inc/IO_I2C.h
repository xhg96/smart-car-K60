#ifndef _IO_I2C_H_
#define _IO_I2C_H_


#define I2C_SCL         PTC10
#define I2C_SDA         PTC11

#define I2C_SCL_H()         PTXn_T(I2C_SCL,OUT) = 1
#define I2C_SCL_L()         PTXn_T(I2C_SCL,OUT) = 0

#define I2C_SDA_H()         PTXn_T(I2C_SDA,OUT) = 1
#define I2C_SDA_L()         PTXn_T(I2C_SDA,OUT) = 0
#define I2C_SDA_IN()        PTXn_T(I2C_SDA,IN)

#define I2C_SDA_DDR_OUT()   PTXn_T(I2C_SDA,DDR) = 1
#define I2C_SDA_DDR_IN()    PTXn_T(I2C_SDA,DDR) = 0

#define I2C_DELAY()     DELAY_US(1)


//IIC Functions
extern void IIC_Init(void);                				 


extern uint8 I2C_ReadOneByte(uint8 I2C_Addr,uint8 addr);
extern uint8 IIC_ReadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data);
extern uint8 IIC_ReadByte(uint8 dev, uint8 reg, uint8 *data);

extern uint8 IIC_WriteOneByte(uint8 dev, uint8 reg, uint8 data);
extern uint8 IIC_WriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data);

extern uint8 IIC_WriteBits(uint8 dev,uint8 reg,uint8 bitStart,uint8 length,uint8 data);
extern uint8 IIC_WriteBit(uint8 dev,uint8 reg,uint8 bitNum,uint8 data);

/* ”√”⁄DMPø‚ */
extern int DMP_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
extern int DMP_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


#endif

