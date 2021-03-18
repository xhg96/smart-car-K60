#include "common.h"
#include "IO_I2C.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU6050.h"
#include "m_LCD.h"


#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (50)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f


float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw = 0; 

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    
    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
    }
}

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];

int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;


/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IIC_WriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
    
}

/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IIC_WriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IIC_WriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
    
    IIC_ReadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_testConnection(void)
*功　　能:	    检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
    if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
        return 1;
   	else return 0;
}

/**************************实现函数*******************************************
*函数原型:  设置低通滤波器配置
*功      能:
******************************************************************************/
void MPU6050_setDLPFMode(uint8_t mode){
    IIC_WriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/**************************实现函数*******************************************
* 开启加速度器X轴的安全自测功能。
* @安全自测启用参数
* @请参见MPU6050_RA_ACCEL_CONFIG字段
******************************************************************************/
void MPU6050_setAccelXSelfTest(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}

/**************************实现函数*******************************************
* 开启加速度器Y轴的安全自测功能。
* @安全自测启用参数
* @请参见MPU6050_RA_ACCEL_CONFIG字段
******************************************************************************/
void MPU6050_setAccelYSelfTest(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}

/**************************实现函数*******************************************
* 开启加速度器Z轴的安全自测功能。
* @安全自测启用参数
* @请参见MPU6050_RA_ACCEL_CONFIG字段
******************************************************************************/
void MPU6050_setAccelZSelfTest(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}



//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8 MPU_Set_LPF(uint16 lpf)
{
	uint8 data=0;
    
	if(lpf>=188) data=1;
	else if(lpf>=98) data=2;
	else if(lpf>=42) data=3;
	else if(lpf>=20) data=4;
	else if(lpf>=10) data=5;
	else data=6; 
    
	return IIC_WriteOneByte(devAddr,MPU6050_RA_CONFIG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8 MPU_Set_Rate(uint16 rate)
{
	uint8 data;
	
    if(rate>1000) rate=1000;
	if(rate<4) rate=4;
    
	data = 1000/rate-1;
	
    IIC_WriteOneByte(devAddr,MPU6050_RA_SMPLRT_DIV,data);	//设置采样率
 	
    return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Init(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(void)
{
    mLCD_str(0,40,"MPU6050 Init ing...", BLUE,WHITE);
    
    IIC_Init();
    
    if( MPU6050_testConnection() )
        mLCD_str(0,40,"MPU6050 Init OK!   ", BLUE,WHITE);
    else
        mLCD_str(0,40,"MPU6050 Init Error ", BLUE,WHITE);
    
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X80);	//复位MPU6050
 	DELAY_MS(100);
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X00);	//唤醒MPU6050
    
	IIC_WriteOneByte(devAddr,MPU6050_RA_GYRO_CONFIG,3<<3);	//陀螺仪最大量程2000
	IIC_WriteOneByte(devAddr,MPU6050_RA_ACCEL_CONFIG,2<<3); //加速度计最大量程
    MPU_Set_Rate(100);						                //设置采样率
    
    IIC_WriteOneByte(devAddr,MPU6050_RA_INT_ENABLE,0X00);	//关闭所有中断
	IIC_WriteOneByte(devAddr,MPU6050_RA_USER_CTRL,0X00);	//I2C主模式关闭
	IIC_WriteOneByte(devAddr,MPU6050_RA_FIFO_EN,0X00);		//关闭FIFO
	IIC_WriteOneByte(devAddr,MPU6050_RA_INT_PIN_CFG,0X80);	//INT引脚低电平有效
    
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X01);	//设置CLKSEL,PLL X轴为参考
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_2,0X00);	//加速度与陀螺仪都工作
    MPU_Set_Rate(100);						                //设置采样率为50Hz
    
    DELAY_MS(100);
    
    float gyro_sum = 0;
    uint16 gyro_tmp;
        
    for(uint16 i=0; i<100; i++)
    {
        gyro_tmp = ((uint16)I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8) + I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取X轴陀螺仪
          
        gyro_sum += gyro_tmp;  
        
        DELAY_MS(10);
    }
    
    Gy_offset = (int16)(gyro_sum / 100);
    
    //MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);      //设置时钟
    //MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);  //陀螺仪最大量程
    //MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//加速度度最大量程
    //MPU6050_setDLPFMode(MPU6050_DLPF_BW_42);              //设置低通滤波器	
}


/**************************************************************************
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DMP_Init(void)
{ 
    uint8 temp[1]={0};
    
    IIC_Init();
    
    DMP_i2cRead(0x68,0x75,1,temp);
	//printf("mpu_set_sensor......\r\n");

	mLCD_str(5,0,"mpu_set_sensor...", BLUE,WHITE);
	
    //if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
    {
	if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            //printf("mpu_set_sensor complete ......\r\n");
            mLCD_str(5,15,"mpu_set_sensor complete..", BLUE,WHITE);
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            //printf("mpu_configure_fifo complete ......\r\n");
            mLCD_str(5,30,"mpu_config_fifo complete.", BLUE,WHITE);
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
            //printf("mpu_set_sample_rate complete ......\r\n");
            mLCD_str(5,45,"mpu_set_sam_rate_complete", BLUE,WHITE);
        if(!dmp_load_motion_driver_firmware())
            //printf("dmp_load_motion_driver_firmware complete ......\r\n");
            mLCD_str(5,60,"dmp_load_m_d_complete...", BLUE,WHITE);
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
            //printf("dmp_set_orientation complete ......\r\n");
            mLCD_str(5,75,"dmp_set_ori complete...", BLUE,WHITE);
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                               DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
            //printf("dmp_enable_feature complete ......\r\n");
            mLCD_str(5,90,"dmp_enable_feature complete", BLUE,WHITE);
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
            //printf("dmp_set_fifo_rate complete ......\r\n");
            mLCD_str(5,105,"dmp_set_fifo_rate complete", BLUE,WHITE);

		run_self_test();
		
        if(!mpu_set_dmp_state(1))
        {
            printf("mpu_set_dmp_state complete ......\r\n");
			mLCD_str(5,120,"mpu_set_dmp_state complete", BLUE,WHITE);
                        LCD_clear(WHITE);
        }
		else
			mLCD_str(5,120,"mpu_set_dmp_state FAILED!", BLUE,WHITE);	

		
    }
}
/**************************************************************************
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Read_DMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
    
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	
	
	if (sensors & INV_WXYZ_QUAT )
	{    
		q0=quat[0] / q30;
    	q1=quat[1] / q30;
		q2=quat[2] / q30;
	    q3=quat[3] / q30;
        
        Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//俯仰角
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; //横滚角
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3; //航向角
        
        //printf("Pitch:%d  Roll:%d  Yaw:%d\n",(int)Pitch,(int)Roll,(int)Yaw);
	}
}

//------------------End of File----------------------------