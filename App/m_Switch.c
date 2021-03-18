#include "common.h"
#include "MK60_gpio.h"
#include "m_Switch.h"
#include "VCAN_KEY.H" 

void PORTD_IRQHandler(void);
extern KEY_MSG_t keymsg;   

/* 定义SWITCH编号对应的管脚 */
PTXn_e SWITCH_PTxn[SWITCH_MAX] = {PTD15, PTE6, PTE7, PTE8, PTE9, PTE10,PTE11, PTE12};

/******************************************************************************
 *  @brief      初始化switch端口
 *****************************************************************************/
void Switch_Init()
{
    uint8 i = SWITCH_MAX;

    //初始化全部 按键
    while(i--)
    {
        gpio_init(SWITCH_PTxn[i], GPI, 0);
        port_init_NoALT(SWITCH_PTxn[i], PULLUP);    //保持复用不变，仅仅改变配置选项
    }
    
    
    set_vector_handler(PORTD_VECTORn, PORTD_IRQHandler);
    enable_irq(PORTD_IRQn);               // 开PORTD中断(按键)

}

  //  PORTD中断服务函数(按键下降沿中断)
void PORTD_IRQHandler(void)
{
    uint32 flag;
    
    while(!PORTD_ISFR);
    flag = PORTD_ISFR;  //取中断引脚号
    PORTD_ISFR = ~0;    //清中断标志位
    
   if(flag & (1 << 11)){
       DELAY_MS(10);    //按键延时消抖    
		// 方向上键按下
       if ( !PTD11_IN ){
          keymsg.key = KEY_U;
          keymsg.status = KEY_DOWN;
       }
    }
    else if(flag & (1 << 14)){
        DELAY_MS(10);    
        
    // 方向下键按下
	if ( !PTD14_IN ){
          keymsg.key = KEY_D;
          keymsg.status = KEY_DOWN;
        }
    }
    else if(flag & (1 << 13)){
        DELAY_MS(10);   
        
		// 方向左键按下
      if ( !PTD13_IN ){
        keymsg.key = KEY_L;
        keymsg.status = KEY_DOWN;
      }
    }
    else if(flag & (1 << 10))
    {
      DELAY_MS(10);    //按键延时消抖
        
		// 方向右键按下
      if ( !PTD10_IN )
      {
	keymsg.key = KEY_R;
	keymsg.status = KEY_DOWN;
      }
    }
    else if(flag & (1 << 12))
    {
       DELAY_MS(10);    //按键延时消抖
        
		// 确认键按下
       if( !PTD12_IN ){
          keymsg.key = KEY_B;
          keymsg.status = KEY_DOWN;
        }
    }
}

/******************************************************************************
 *  @brief      获取switch状态
 *  @param      SWITCH_e         SWITCH编号
 *  @return     SWITCH_STATUS_e    SWITCH状态(SWITCH_ON,SWITCH_OFF)
 ******************************************************************************/
SWITCH_STATUS_e Switch_Get(SWITCH_e i)
{
    if(gpio_get(SWITCH_PTxn[i]) == SWITCH_OFF)
    {
        return SWITCH_OFF;
    }
    return SWITCH_ON;
}

uint16 picTotalNumSet = 1200;         //存图像总数，于LCD中调整
uint16 skipNumSet = 5;                   //跳过循环数
uint16 savePicSwitch = 0;              //存图像开关

/*      //原在main中的弃用函数

void imgSave2SD_hand(int skipNum, int picNumLimit){     //调用存图像
  static int skipCount   = 0;
  static int picNumCount = 0;
  
  skipCount++;
  if(skipCount > skipNum && picNumCount < picNumLimit){
    img_sd_save(imgbuff,CAMERA_SIZE);     //存图像
    skipNum = 0;
    picNumCount++;
  }
  else if(picNumCount == picNumLimit){
    skipCount = 0;      //防溢出
    img_sd_exit();
  }
    
}

//
//  @brief      存图像至microSD卡,while中调用
//  @param      skipNum,picNumLimit:  存图像间隔，存图像总数上限
//  @return     None
//


void m_imgSave2SD(int skipNum, int picNumLimit){
  static int skipCount   = 0;
  static int picNumCount = 0;
  static uint8 saveStatus = 0;  //存储状态 0
  skipCount = skipCount>10000?0:skipCount;      //防溢出
 
  if(saveStatus == 0 && keymsg.key == KEY_START && savePicSwitch == 1)
    saveStatus = 1;     //开始存图像
  
  if(saveStatus == 1){
    skipCount++;
    if(skipCount > skipNum && picNumCount < picNumLimit ){
      img_sd_save(imgbuff,CAMERA_SIZE);     //存图像
      skipNum = 0;
      picNumCount++;
    }
    else if(picNumCount == picNumLimit){
      savePicSwitch = 0;  //关闭存图像
      saveStatus = 0;   //还原状态
      img_sd_exit();
    }
  }
}

  //disk格式转FAT存sd卡
void save2SD_disk(){
        if( GET_SWITCH4() == SWITCH_ON &&(GET_SWITCH5() == SWITCH_ON || keymsg.key == KEY_START ))
        {     
            send_data_to_SD_disk();  //数据保存到扇区 
        }
            //停车后存SD卡
        if( RunMode == TIMING_RUN && stopFlag == 1 && GET_SWITCH4() == SWITCH_ON && keymsg.key == KEY_U)
        {
            if(SPD.nowSpeed < 20)
            {
		DISABLE_MOTOR;
                disable_irq(PIT0_IRQn);   // 关闭PIT0中断
                disable_irq(PIT1_IRQn);   // 关闭PIT1中断
                disable_irq(PORTB_IRQn);   //关闭PTB的场中断
               
                //开始将sd卡数据保存到file里
                SD_disk_to_FAT_file();
                DisableInterrupts;      
                while(keymsg.key == KEY_U);
            }
        }
}


void touch_IRQHandler(){
   uint32 flag;
    
    while(!PORTC_ISFR);
    flag = PORTC_ISFR;  //取中断引脚号
    PORTC_ISFR = ~0;    //清中断标志位
   // int a = PTC18_IN;
    
    if(flag & (1 << 18)){   
	DELAY_MS(10);	//此处可加延时消抖
       if (! PTC18_IN ){
          touchFlag = 1;
          setTimeBeep_ms(200,VHIGH);
          
       }
    }
    if(flag & (1 << 19)){   
	DELAY_MS(10);	
       if (! PTC19_IN ){
          touchFlag = 1;
          setTimeBeep_ms(200,VHIGH);
          
       }
    }
    
}

void touchSwitchInit(){
    //PTC18, 19设为 
  gpio_init(PTC18, GPI, 0);
  port_init_NoALT(PTC18,  PULLUP | IRQ_FALLING );    //保持复用不变，仅仅改变配置选项
  gpio_init(PTC19, GPI, 0);
  port_init_NoALT(PTC19,  PULLUP | IRQ_FALLING );    //保持复用不变，仅仅改变配置选项
  
  set_vector_handler(PORTC_VECTORn, touch_IRQHandler);
  enable_irq(PORTC_IRQn);   //使能
}
*/