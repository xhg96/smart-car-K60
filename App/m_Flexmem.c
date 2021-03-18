#include "common.h"
#include "m_Flexmem.h"
#include "MK60_flash.h"
#include "vcan_ui_var.h"
#include "control.h"
#include "motor.h"
#include "m_Motro.h"

extern uint16 param0;           //图像处理中 的
extern uint16 obOffset;
extern uint16 obB;
extern uint16 ki;
extern uint16 RuningTime;            //运行定时(定时模式有效)
extern RUNMode_e RunMode;            //运行模式标志
extern uint16 ConstantSpeed;
extern uint16 rampDelayDown;
extern uint16 rampDelayUp;
extern uint8 *imgbuff;          //图像
extern uint16 lineAmend,speedLineK,speedLineB;
extern uint16 picTotalNumSet;         //存图像总数，于LCD中调整
extern uint16 skipNumSet;                //跳过循环数
extern uint16 savePicSwitch;              //存图像开关
extern uint8 flash_num ;
#define SECTOR_NUM  (FLASH_SECTOR_NUM - flash_num)         //尽量用最后面的扇区，确保安全

//变量地址数组
uint16 *var_addr[VAR_MAX] = {
                          (uint16 *)&CP.k,        //舵机部分参数
                          (uint16 *)&CP.b,
                          (uint16 *)&CP.left_k,
                          (uint16 *)&CP.right_k,
                          (uint16 *)&Servo_PID.Kdin,
                          (uint16 *)&Servo_PID.Kdout,
                          (uint16 *)&taw,
                          
                         (uint16 *)&Motro_PID.Kp,        //电机PID参数
                         (uint16 *)&ki,
                         (uint16 *)&Motro_PID.Kd,
                         (uint16 *)&encoderThreshold,
                         (uint16 *)&change_ki,
                         
                        (uint16 *)&change_kib,
                        (uint16 *)&ov7725_cnst_set,     //阈值
                          
                         (uint16 *)&RuningTime,          //定时运行参数
                         (uint16 *)&ConstantSpeed,
                                               //速度控制参数
                         (uint16 *)&picTotalNumSet,      //存图像参数
                         (uint16 *)&skipNumSet,
                          (uint16 *)&savePicSwitch,
                          
                          (uint16 *)&param0,
                          (uint16 *)&obOffset,
                          (uint16 *)&obB,
                          
                          (uint16 *)&Ann_k,               //圆环相关
                          (uint16 *)&annulus_speed,    
                          (uint16 *)&ann_speed_k,
                          
                          (uint16 *)&max_speed,
                          (uint16 *)&max2_speed,
                          (uint16 *)&wan_speed,
                          (uint16 *)&min_speed,
                          (uint16 *)&base_speed,
                          
                          (uint16 *)&outm_speed,
                          (uint16 *)&add_speed,
                          (uint16 *)&ra_speed,
                          (uint16 *)&obcutsp,
                          (uint16 *)&speed_k,
                          
                          (uint16 *)&extraStopVal_Long,
                          (uint16 *)&extraStopVal_Short,
                          (uint16 *)&extraStopVal_Turnin,
                          (uint16 *)&bump_speed,
                          (uint16 *)&turnILimitsub,
                          
                          (uint16 *)&bra_speed1,
                          (uint16 *)&bra_speed2,
                          (uint16 *)&bra_speed3,
                          (uint16 *)&bra_speed4,
                                          //赛道判断参数
                          (uint16 *)&zhidao_ade,
                          (uint16 *)&wan_ade,
                          (uint16 *)&dzhi_top,
                          (uint16 *)&set_othertop,
                          (uint16 *)&set_cothertop,
                          (uint16 *)&wanru_ade,
                          (uint16 *)&lineAmend,
                          (uint16 *)&speedLineK,
                          (uint16 *)&speedLineB,
                          (uint16 *)&set_yh_top,  
                          (uint16 *)&MID_PWM,     //舵机中值
                          
                               
                        (uint16 *)&rampDelayDown,
                        (uint16 *)&rampDelayUp,
                        (uint16 *)&ann_min_speed,
                        (uint16*)&bumpEncoderThreshold,
};

uint8 flash_num = 1;     //扇区选择

uint16 *var_addr[VAR_MAX];

uint16 MY_SECTOR_NUM;


void FlashSave(void)
{
  MY_SECTOR_NUM=FLASH_SECTOR_NUM-flash_num;
    flash_erase_sector(MY_SECTOR_NUM);
    for(uint8 var_num = 0; var_num<VAR_MAX;var_num++)
    {
      flash_write( MY_SECTOR_NUM,32*(var_num),(uint16) ( *((uint16*)var_addr[var_num]) ) );
    }
}

void FlashRead(void)
{
  MY_SECTOR_NUM=FLASH_SECTOR_NUM-flash_num;
  uint8 var_num;

    for (var_num = 0; var_num < VAR_MAX; var_num++ )
    {
      *((uint16*)(var_addr[var_num])) = (uint16)flash_read(MY_SECTOR_NUM,32*(var_num),uint16); 
    }  
}


/******************************************************************************
* Partition flash routine. This function can be used to setup
* the flash for enhanced EEPROM operation. In order to guarantee
* the eEE endurance the partition command should only be used one
* time (re-partitioning in a different configuration will erase
* wear-leveling data, so endurance can no longer be guaranteed).
* This function will test to make sure the flash has not been
* partitioned already.
*
* Parameters:
* eeprom_size size of the two EEPROM data sets (A and B) defines in flexmem_demo.h
* elfash_size amount of dflash memory available after partitioning defines in flexmem_demo.h
*
* Returns:
* 1  partitioning completed successfully
* 0  partitioning not completed (device is already partitioned)
******************************************************************************/
int Partition_Flash(int eeprom_size, int eflash_size)
{
    /* Test to make sure the device is not already partitioned. If it
    * is already partitioned, then return with no action performed.
    */
    if ((SIM_FCFG1 & SIM_FCFG1_DEPART(0xF)) != 0x00000F00)
    {
        //printf("\nDevice is already partitioned.\n");
        return 0;
    }   
    
    /* Write the FCCOB registers */
    FTFE_FCCOB0 = FTFE_FCCOB0_CCOBn(0x80); // Selects the PGMPART command
    FTFE_FCCOB1 = 0x00;
    FTFE_FCCOB2 = 0x00;
    FTFE_FCCOB3 = 0x00;
    
    /* FCCOB4 is written with the code for the subsystem sizes (eeprom_size define) */
    FTFE_FCCOB4 = eeprom_size;
    
    /* FFCOB5 is written with the code for the Dflash size (dflash_size define) */
    FTFE_FCCOB5 = eflash_size;
    
    
    /* All required FCCOBx registers are written, so launch the command */
    FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;
    
    /* Wait for the command to complete */
    while(!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));
    
    return 1;
}
 