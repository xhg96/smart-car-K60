#include "include.h"
#include "control.h"
#include "motor.h"
#include "go.h"
#include "deal_img.h"
#include "data_transfer.h"
#include "MPU6050.h"
#include "IO_I2C.h"
#include "MK60_gpio.h"
  //�ⲿ����
extern uint8 imgbuff[CAMERA_SIZE];     //���������ͼ��
extern uint8 leftmap[YM][XM];    // 2 3
extern uint8 rightmap[YM][XM];
extern uint8 sendNum;
  //��������
void init_all();
void startGetImg();
void waitFinishGet();
void beepHandler();
void MPU6050_DMP_Handler();
void setTimeBeep_ms(uint8 timeSet,uint8 vt);
void beepTest();
  //ȫ�ֱ���
uint8 collectbuff[CAMERA_SIZE]; //�ɼ���ͼ�� 
int TimingCount = 0;                //��ʱģʽ��ʱ

uint8 beepStatus = 0;           //���ڷ�����
uint16 beepTime = 0;
uint8 beepVoiceCount = 0;
uint32 runTimeCnt = 0;
uint8 CCD_BUFF[TSL1401_SIZE];   //CCDͼ��

  //������ر���
RUNMode_e RunMode = NORMAL_RUN;	//����ģʽ��־
VOICE_TYPE_e voiceType = VHIGH; //����������
WatchDog_e WDState = ENABLE;    //���Ź���̬
uint8 stopFlag = 1;             //ͣ����־λ

//uint8 lastNum=0;
uint32 imgTime = 0;  
int main(void)
{ 
   init_all();             // ��ʼ��
   Read_EEPROM();   //��EEPROM 
  
   //standard();             //��ȡ��׼��
   ov7725_eagle_get_img(); // ��ʼͼ��ɼ�
       
   MainMenu_Set();
 
    wdog_init_ms(500);
    wdog_enable();
    uint8 noDelayCnt[2] = {0,0};      //��һ��while����ʱ�� 
   // tsl1401_get_img();
    standard(); 
    
//    Motro_Init();
//    ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM3_CH, 0);
//    ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,500); 
    while(1)
    {      
      //��������Ӷ���
        memcpy(imgbuff,collectbuff,CAMERA_SIZE);        //ת��ͼ����imgbuff��
        startGetImg();        //��ʼ�ɼ�ͼ�� 
        //whileTime = pit_time_get_us(PIT2);
        pit_time_start(PIT2);
        
        if(WDState == ENABLE) {wdog_enable(); WDState = ENABLE_OPERATED;}       //���ݿ��Ź�״̬ʹ�ܻ�ر�
        else if(WDState == DISABLE){wdog_disable(); WDState = DISABLE_OPERATED;}
        
        if(WDState == ENABLE_OPERATED) wdog_feed();  //ʹ�������ι��
        
        noDelayCnt[1] = noDelayCnt[0];   
        noDelayCnt[0] = 1;
        //newBoardTest();
        MPU6050_DMP_Handler();
       // tsl1401_get_img();
      
        go();                   //ͼ�����������
        Track_type_detect();    //���������жϣ����� Ŀ���ٶ�
        send_data_to_SD_disk();
        
        if( GET_SWITCH1() == SWITCH_ON){        //LCD������ʾ
          if(WDState != DISABLE_OPERATED) WDState = DISABLE; 

          LCD_ParamAndImgDisplay();
        }
        else if(GET_SWITCH1() != SWITCH_ON && WDState != ENABLE_OPERATED){
          WDState = ENABLE;
        }
          
      
        if(keymsg.key == KEY_STOP) { ///����LCD�����������˵�
          wdog_disable(); 
          MainMenu_Set();
          WDState = ENABLE;
        }
      
    //    if( GET_SWITCH8() == SWITCH_ON){//&&sendNum!=lastNum){
         //Send_Speed_Info();  //���������������ƴ���λ��
         // SD5_Info();
         //lastNum=sendNum;     
          
    //    }       
        endInfoShow();
        dealTime=pit_time_get_us(PIT2);
        imgTime=pit_time_get_us(PIT3);
        
       if(imgTime/1000 < 12 && noDelayCnt[1] == 1) DELAY_MS(12 - imgTime/1000);
        waitFinishGet();      //�ȴ�ͼ��ɼ��������Ա�֤���������ڼ�����       
    }
    return 0;
}

  // PORTB�жϷ�����(����ͷ���ж�)
void PORTB_IRQHandler(void)  
{
    uint8  n;    //���ź�
    uint32 flag;
    
    while(!PORTB_ISFR);
    flag = PORTB_ISFR;
    PORTB_ISFR = ~0;                        //���жϱ�־λ
    
    n = 23;                                 //���ж�
    if(flag & (1 << n))                     //PTB23�������ж�
    {
        pit_time_start(PIT3);  
        ov7725_eagle_vsync();
    }
}

void PIT0_IRQHandler()
{
  PIT_Flag_Clear(PIT0);
  beepVoiceCount++;

  if(RunMode == TIMING_RUN || RunMode == QUADRA_RUN || RunMode == TIMING_SPEED_RUN) Timing_Driving();       //ѡ������ģʽ
  else Normal_Driving();  
  
    beepHandler();

}

  //  PIT1�жϷ����� 
void PIT1_IRQHandler()  
{
   PIT_Flag_Clear(PIT1);
   
   if(stopFlag == 0) runTimeCnt++;
   if(runTimeCnt > 60000) runTimeCnt = 60000;
  
  if(keymsg.key == KEY_START)     //��ʼ�����������
    TimingCount++;
  if(TimingCount > 60000)
    TimingCount = 60000;
  
  if(keymsg.key == KEY_START) //��ʼ�����������
    StartDelayCount++;
  if(StartDelayCount > 60000)
    StartDelayCount = 60000;
  
  if(stopRemember == 1) //��⵽ͣ����,������ͣ���������
    StopDelayCount++;
  if(StopDelayCount > 60000)
    StopDelayCount = 60000;
  
 //   tsl1401_time_isr(); //CCD�ɼ�
    Car_go();     //����ٶȿ��ƣ���ȡ������  
      
}

  //��ʼȡͼ��
void startGetImg()
{
  ov7725_eagle_img_flag=IMG_START;
  PORTB_ISFR = ~0;                        //д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
  enable_irq(PORTB_IRQn);                         //����PTA���ж�
}
  //�ȴ�ȡͼ�����
void waitFinishGet()
{
    while(ov7725_eagle_img_flag != IMG_FINISH)           //�ȴ�ͼ��ɼ����
    {
        if(ov7725_eagle_img_flag == IMG_FAIL)            //����ͼ��ɼ����������¿�ʼ�ɼ�
        {
 
            ov7725_eagle_img_flag = IMG_START;           //��ʼ�ɼ�ͼ��
            PORTB_ISFR = ~0;                //д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
            enable_irq(PORTB_IRQn);                 //����PTB���ж�
        }
    }
}


void  init_all()
{       
  /* ����128byte��flexRAM��EEPROM��512KB��flexNVM��ΪEEPROM�����ݷ�����(ע��ֻ��K60FX512��Ƭ���д˹��ܣ������ͺŵ�Ƭ����ʹ��FLASH) */
    Partition_Flash(EEPROM_128_896, EFLASH_SIZE_512);
    
    DISABLE_MOTOR;      //�Ͽ����������λ�ܷ�
    Servo_Init();          //��ʼ�� ��� PWM
    Motro_Init();          //��ʼ�� �����������
    //adc_init(ADC0_DP1);
    //DMP_Init();       ���ڷ������ʼ��
    /************************ ���� K60 �����ȼ�  ***********************/
    NVIC_SetPriorityGrouping(4);            //�������ȼ�����,4bit ��ռ���ȼ�,û�������ȼ�

    NVIC_SetPriority(PORTB_IRQn,0);         //����ͷ���ж����ȼ�����Ϊ��� 
    NVIC_SetPriority(DMA0_IRQn, 1);         //DMA�ж�
    NVIC_SetPriority(PIT1_IRQn, 2);         //�������������ٶȿ������ȼ�
    NVIC_SetPriority(PIT0_IRQn, 3);         //��������������ģʽ
    NVIC_SetPriority(PORTA_VECTORn,4);      //�����������ȼ�
    NVIC_SetPriority(PORTD_IRQn,5);         //���ᰴ���ж�
    //NVIC_SetPriority(UART2_RX_TX_IRQn,2);   //���ڽ����ж����ȼ����˻ᵼ������֡�޷�����
    
//    tsl1401_init(10);             //CCD��ʼ�����ع�ʱ��10ms
//    tsl1401_set_addrs(TSL1401_L, CCD_BUFF);
    img_sd_init_disk();                     //disk��ʼ��SD��
    Switch_Init();                          //���뿪�أ�������ʼ��
    m_LED_And_BEEP_Init();                  // LED��������ʼ��

    ftm_quad_init(FTM2);                        //��������FTM2 ���������ʼ��
    port_init_NoALT(FTM2_QDPHA_PIN,PULLUP);     //���ùܽ�����
    port_init_NoALT(FTM2_QDPHB_PIN,PULLUP);
    
    myLCD_Init();         //LCD��ʼ��
    key_init(KEY_MAX);  //���ᰴ����ʼ��
 
    pit_init_ms(PIT1,10);               //PIT1�жϴ�����ɨ�輰����
    pit_init_ms(PIT0,5);               //ͣ�����ƣ���ͼ��
    
      //�����жϷ�����  
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);  
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); 
    set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);
    set_vector_handler(DMA0_VECTORn ,ov7725_eagle_dma);       
    set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);  
    ov7725_eagle_init(collectbuff);           //����ͷ��ʼ������ͼ��ɼ��� collectbuff 
    
    flash_init();         //��ʼ��flash
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
  
  if(tCont *5> beepTime || keymsg.key == KEY_STOP)//ȷ��ʱ��
  {
    beepStatus = 0;
  }
  
  if(beepStatus == 1 && voiceType == VHIGH){     //100%ռ�ձ�
    BEEP_ON;
    beepVoiceCount = 0;
  }
  else if(beepStatus == 1 && voiceType == VLOW){ //50%ռ�ձ�
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
  else  {//ʱ���ѵ�
     BEEP_OFF;
     beepVoiceCount = 0;
  }

}
    //DMP��6050����ʼ����������while(1)�е���
void MPU6050_DMP_Handler(){
  if(DMP_Delay_Status == START_INIT) {//��ʼ�����ʼ��
     wdog_disable();
     DMP_Delay_Status = DELAY_START;    //��ʼ��ʱ
     DMP_Init();
     DMP_Delay_Status = DELAY_TIME_UP;  //��ʱʱ�䵽
     wdog_enable();
  }
  
  if(DMP_Delay_Status == DMP_READ) {
    Read_DMP();  
  }
}

