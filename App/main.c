#include "include.h"
#include "control.h"
#include "motor.h"
#include "go.h"
#include "deal_img.h"
#include "data_transfer.h"
#include "MPU6050.h"
#include "IO_I2C.h"
#include "MK60_gpio.h"
  //外部变量
extern uint8 imgbuff[CAMERA_SIZE];     //用来处理的图像
extern uint8 leftmap[YM][XM];    // 2 3
extern uint8 rightmap[YM][XM];
extern uint8 sendNum;
  //函数声明
void init_all();
void startGetImg();
void waitFinishGet();
void beepHandler();
void MPU6050_DMP_Handler();
void setTimeBeep_ms(uint8 timeSet,uint8 vt);
void beepTest();
  //全局变量
uint8 collectbuff[CAMERA_SIZE]; //采集的图像 
int TimingCount = 0;                //定时模式计时

uint8 beepStatus = 0;           //用于蜂鸣器
uint16 beepTime = 0;
uint8 beepVoiceCount = 0;
uint32 runTimeCnt = 0;
uint8 CCD_BUFF[TSL1401_SIZE];   //CCD图像

  //发车相关变量
RUNMode_e RunMode = NORMAL_RUN;	//运行模式标志
VOICE_TYPE_e voiceType = VHIGH; //蜂鸣器类型
WatchDog_e WDState = ENABLE;    //看门狗姿态
uint8 stopFlag = 1;             //停车标志位

//uint8 lastNum=0;
uint32 imgTime = 0;  
int main(void)
{ 
   init_all();             // 初始化
   Read_EEPROM();   //读EEPROM 
  
   //standard();             //获取标准线
   ov7725_eagle_get_img(); // 开始图像采集
       
   MainMenu_Set();
 
    wdog_init_ms(500);
    wdog_enable();
    uint8 noDelayCnt[2] = {0,0};      //第一个while不延时用 
   // tsl1401_get_img();
    standard(); 
    
//    Motro_Init();
//    ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM3_CH, 0);
//    ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,500); 
    while(1)
    {      
      //别在这里加东西
        memcpy(imgbuff,collectbuff,CAMERA_SIZE);        //转存图像至imgbuff中
        startGetImg();        //开始采集图像 
        //whileTime = pit_time_get_us(PIT2);
        pit_time_start(PIT2);
        
        if(WDState == ENABLE) {wdog_enable(); WDState = ENABLE_OPERATED;}       //根据看门狗状态使能或关闭
        else if(WDState == DISABLE){wdog_disable(); WDState = DISABLE_OPERATED;}
        
        if(WDState == ENABLE_OPERATED) wdog_feed();  //使能情况下喂狗
        
        noDelayCnt[1] = noDelayCnt[0];   
        noDelayCnt[0] = 1;
        //newBoardTest();
        MPU6050_DMP_Handler();
       // tsl1401_get_img();
      
        go();                   //图像处理，舵机控制
        Track_type_detect();    //赛道类型判断；计算 目标速度
        send_data_to_SD_disk();
        
        if( GET_SWITCH1() == SWITCH_ON){        //LCD参数显示
          if(WDState != DISABLE_OPERATED) WDState = DISABLE; 

          LCD_ParamAndImgDisplay();
        }
        else if(GET_SWITCH1() != SWITCH_ON && WDState != ENABLE_OPERATED){
          WDState = ENABLE;
        }
          
      
        if(keymsg.key == KEY_STOP) { ///进入LCD参数调试主菜单
          wdog_disable(); 
          MainMenu_Set();
          WDState = ENABLE;
        }
      
    //    if( GET_SWITCH8() == SWITCH_ON){//&&sendNum!=lastNum){
         //Send_Speed_Info();  //发送数据至匿名科创上位机
         // SD5_Info();
         //lastNum=sendNum;     
          
    //    }       
        endInfoShow();
        dealTime=pit_time_get_us(PIT2);
        imgTime=pit_time_get_us(PIT3);
        
       if(imgTime/1000 < 12 && noDelayCnt[1] == 1) DELAY_MS(12 - imgTime/1000);
        waitFinishGet();      //等待图像采集结束，以保证舵机打角周期间隔相等       
    }
    return 0;
}

  // PORTB中断服务函数(摄像头场中断)
void PORTB_IRQHandler(void)  
{
    uint8  n;    //引脚号
    uint32 flag;
    
    while(!PORTB_ISFR);
    flag = PORTB_ISFR;
    PORTB_ISFR = ~0;                        //清中断标志位
    
    n = 23;                                 //场中断
    if(flag & (1 << n))                     //PTB23触发场中断
    {
        pit_time_start(PIT3);  
        ov7725_eagle_vsync();
    }
}

void PIT0_IRQHandler()
{
  PIT_Flag_Clear(PIT0);
  beepVoiceCount++;

  if(RunMode == TIMING_RUN || RunMode == QUADRA_RUN || RunMode == TIMING_SPEED_RUN) Timing_Driving();       //选择运行模式
  else Normal_Driving();  
  
    beepHandler();

}

  //  PIT1中断服务函数 
void PIT1_IRQHandler()  
{
   PIT_Flag_Clear(PIT1);
   
   if(stopFlag == 0) runTimeCnt++;
   if(runTimeCnt > 60000) runTimeCnt = 60000;
  
  if(keymsg.key == KEY_START)     //开始键按下情况下
    TimingCount++;
  if(TimingCount > 60000)
    TimingCount = 60000;
  
  if(keymsg.key == KEY_START) //开始键按下情况下
    StartDelayCount++;
  if(StartDelayCount > 60000)
    StartDelayCount = 60000;
  
  if(stopRemember == 1) //检测到停车线,允许检测停车线情况下
    StopDelayCount++;
  if(StopDelayCount > 60000)
    StopDelayCount = 60000;
  
 //   tsl1401_time_isr(); //CCD采集
    Car_go();     //电机速度控制，读取编码器  
      
}

  //开始取图像
void startGetImg()
{
  ov7725_eagle_img_flag=IMG_START;
  PORTB_ISFR = ~0;                        //写1清中断标志位(必须的，不然回导致一开中断就马上触发中断)
  enable_irq(PORTB_IRQn);                         //允许PTA的中断
}
  //等待取图像结束
void waitFinishGet()
{
    while(ov7725_eagle_img_flag != IMG_FINISH)           //等待图像采集完毕
    {
        if(ov7725_eagle_img_flag == IMG_FAIL)            //假如图像采集错误，则重新开始采集
        {
 
            ov7725_eagle_img_flag = IMG_START;           //开始采集图像
            PORTB_ISFR = ~0;                //写1清中断标志位(必须的，不然回导致一开中断就马上触发中断)
            enable_irq(PORTB_IRQn);                 //允许PTB的中断
        }
    }
}


void  init_all()
{       
  /* 分配128byte的flexRAM做EEPROM，512KB的flexNVM作为EEPROM的数据分配区(注意只有K60FX512单片机有此功能，其他型号单片机请使用FLASH) */
    Partition_Flash(EEPROM_128_896, EFLASH_SIZE_512);
    
    DISABLE_MOTOR;      //断开电机，防复位跑飞
    Servo_Init();          //初始化 舵机 PWM
    Motro_Init();          //初始化 电机，编码器
    //adc_init(ADC0_DP1);
    //DMP_Init();       置于发车后初始化
    /************************ 配置 K60 的优先级  ***********************/
    NVIC_SetPriorityGrouping(4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级

    NVIC_SetPriority(PORTB_IRQn,0);         //摄像头场中断优先级配置为最高 
    NVIC_SetPriority(DMA0_IRQn, 1);         //DMA中断
    NVIC_SetPriority(PIT1_IRQn, 2);         //读编码器，及速度控制优先级
    NVIC_SetPriority(PIT0_IRQn, 3);         //蜂鸣器处理，运行模式
    NVIC_SetPriority(PORTA_VECTORn,4);      //发车按键优先级
    NVIC_SetPriority(PORTD_IRQn,5);         //五轴按键中断
    //NVIC_SetPriority(UART2_RX_TX_IRQn,2);   //串口接收中断优先级低了会导致数据帧无法解析
    
//    tsl1401_init(10);             //CCD初始化，曝光时间10ms
//    tsl1401_set_addrs(TSL1401_L, CCD_BUFF);
    img_sd_init_disk();                     //disk初始化SD卡
    Switch_Init();                          //拨码开关，按键初始化
    m_LED_And_BEEP_Init();                  // LED蜂鸣器初始化

    ftm_quad_init(FTM2);                        //编码器，FTM2 正交解码初始化
    port_init_NoALT(FTM2_QDPHA_PIN,PULLUP);     //配置管脚上拉
    port_init_NoALT(FTM2_QDPHB_PIN,PULLUP);
    
    myLCD_Init();         //LCD初始化
    key_init(KEY_MAX);  //五轴按键初始化
 
    pit_init_ms(PIT1,10);               //PIT1中断处理按键扫描及发车
    pit_init_ms(PIT0,5);               //停车控制，存图像
    
      //设置中断服务函数  
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);  
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
    set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);
    set_vector_handler(DMA0_VECTORn ,ov7725_eagle_dma);       
    set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);  
    ov7725_eagle_init(collectbuff);           //摄像头初始化，把图像采集到 collectbuff 
    
    flash_init();         //初始化flash
    stopFlag = 1;
    stopReason = HaventStarted;
}


void setTimeBeep_ms(uint8 timeSet, uint8 vt){
  
  beepTime = timeSet;
  beepStatus = 1;
  if(vt >=2 || vt < 0) vt = 2;
  voiceType = vt;
}

void beepHandler(){
  static uint16 tCont = 0;
  
  tCont++;
  if(beepStatus == 0) tCont = 0;
  
  if(tCont *5> beepTime || keymsg.key == KEY_STOP)//确认时间
  {
    beepStatus = 0;
  }
  
  if(beepStatus == 1 && voiceType == VHIGH){     //100%占空比
    BEEP_ON;
    beepVoiceCount = 0;
  }
  else if(beepStatus == 1 && voiceType == VLOW){ //50%占空比
    if(beepVoiceCount <= 1)
      BEEP_ON;
    else {
      BEEP_OFF;
      beepVoiceCount = 0;
    }
  }
  else if(beepStatus == 1 && voiceType == VMEDIUM){        //80%
    if(beepVoiceCount <= 4)
      BEEP_ON;
    else {
      BEEP_OFF;
      beepVoiceCount = 0;
    }
  }
  else  {//时间已到
     BEEP_OFF;
     beepVoiceCount = 0;
  }

}
    //DMP读6050及初始化函数，于while(1)中调用
void MPU6050_DMP_Handler(){
  if(DMP_Delay_Status == START_INIT) {//开始允许初始化
     wdog_disable();
     DMP_Delay_Status = DELAY_START;    //开始延时
     DMP_Init();
     DMP_Delay_Status = DELAY_TIME_UP;  //延时时间到
     wdog_enable();
  }
  
  if(DMP_Delay_Status == DMP_READ) {
    Read_DMP();  
  }
}

