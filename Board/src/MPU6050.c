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


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IIC_WriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
enabled =1   ˯��
enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_getDeviceID(void)
*��������:	    ��ȡ  MPU6050 WHO_AM_I ��ʶ	 ������ 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
    
    IIC_ReadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_testConnection(void)
*��������:	    ���MPU6050 �Ƿ��Ѿ�����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
    if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
        return 1;
   	else return 0;
}

/**************************ʵ�ֺ���*******************************************
*����ԭ��:  ���õ�ͨ�˲�������
*��      ��:
******************************************************************************/
void MPU6050_setDLPFMode(uint8_t mode){
    IIC_WriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/**************************ʵ�ֺ���*******************************************
* �������ٶ���X��İ�ȫ�Բ⹦�ܡ�
* @��ȫ�Բ����ò���
* @��μ�MPU6050_RA_ACCEL_CONFIG�ֶ�
******************************************************************************/
void MPU6050_setAccelXSelfTest(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}

/**************************ʵ�ֺ���*******************************************
* �������ٶ���Y��İ�ȫ�Բ⹦�ܡ�
* @��ȫ�Բ����ò���
* @��μ�MPU6050_RA_ACCEL_CONFIG�ֶ�
******************************************************************************/
void MPU6050_setAccelYSelfTest(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}

/**************************ʵ�ֺ���*******************************************
* �������ٶ���Z��İ�ȫ�Բ⹦�ܡ�
* @��ȫ�Բ����ò���
* @��μ�MPU6050_RA_ACCEL_CONFIG�ֶ�
******************************************************************************/
void MPU6050_setAccelZSelfTest(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IIC_WriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}



//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8 MPU_Set_LPF(uint16 lpf)
{
	uint8 data=0;
    
	if(lpf>=188) data=1;
	else if(lpf>=98) data=2;
	else if(lpf>=42) data=3;
	else if(lpf>=20) data=4;
	else if(lpf>=10) data=5;
	else data=6; 
    
	return IIC_WriteOneByte(devAddr,MPU6050_RA_CONFIG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8 MPU_Set_Rate(uint16 rate)
{
	uint8 data;
	
    if(rate>1000) rate=1000;
	if(rate<4) rate=4;
    
	data = 1000/rate-1;
	
    IIC_WriteOneByte(devAddr,MPU6050_RA_SMPLRT_DIV,data);	//���ò�����
 	
    return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_Init(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_Init(void)
{
    mLCD_str(0,40,"MPU6050 Init ing...", BLUE,WHITE);
    
    IIC_Init();
    
    if( MPU6050_testConnection() )
        mLCD_str(0,40,"MPU6050 Init OK!   ", BLUE,WHITE);
    else
        mLCD_str(0,40,"MPU6050 Init Error ", BLUE,WHITE);
    
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X80);	//��λMPU6050
 	DELAY_MS(100);
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X00);	//����MPU6050
    
	IIC_WriteOneByte(devAddr,MPU6050_RA_GYRO_CONFIG,3<<3);	//�������������2000
	IIC_WriteOneByte(devAddr,MPU6050_RA_ACCEL_CONFIG,2<<3); //���ٶȼ��������
    MPU_Set_Rate(100);						                //���ò�����
    
    IIC_WriteOneByte(devAddr,MPU6050_RA_INT_ENABLE,0X00);	//�ر������ж�
	IIC_WriteOneByte(devAddr,MPU6050_RA_USER_CTRL,0X00);	//I2C��ģʽ�ر�
	IIC_WriteOneByte(devAddr,MPU6050_RA_FIFO_EN,0X00);		//�ر�FIFO
	IIC_WriteOneByte(devAddr,MPU6050_RA_INT_PIN_CFG,0X80);	//INT���ŵ͵�ƽ��Ч
    
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
	IIC_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_2,0X00);	//���ٶ��������Ƕ�����
    MPU_Set_Rate(100);						                //���ò�����Ϊ50Hz
    
    DELAY_MS(100);
    
    float gyro_sum = 0;
    uint16 gyro_tmp;
        
    for(uint16 i=0; i<100; i++)
    {
        gyro_tmp = ((uint16)I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8) + I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡX��������
          
        gyro_sum += gyro_tmp;  
        
        DELAY_MS(10);
    }
    
    Gy_offset = (int16)(gyro_sum / 100);
    
    //MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);      //����ʱ��
    //MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);  //�������������
    //MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//���ٶȶ��������
    //MPU6050_setDLPFMode(MPU6050_DLPF_BW_42);              //���õ�ͨ�˲���	
}


/**************************************************************************
�������ܣ�MPU6050����DMP�ĳ�ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
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
�������ܣ���ȡMPU6050����DMP����̬��Ϣ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
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
        
        Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//������
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; //�����
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3; //�����
        
        //printf("Pitch:%d  Roll:%d  Yaw:%d\n",(int)Pitch,(int)Roll,(int)Yaw);
	}
}

//------------------End of File----------------------------