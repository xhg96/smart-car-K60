#include "motor.h"
#include "math.h"
#include "MyFlash.h"
//#include "lcd_key.h"
#include "data_transfer.h"
//#include "lcd_key.h"
#include "common.h"
#include "m_LCD.h"
//speed_k=40
//wan_spped=260

DMPDELAY_e DMP_Delay_Status = NO_INIT;
int sramp_time=0;//�����޷�ʶ��С�µ����⴦��ļ�ʱ
int sramp_cutflag=0;//0��ʾ��������ֱ���ٶ���ر��µ���1��ʾ������ֱ������ر��µ�
float sramp_maxspeed=310;    //20s���������µ�����ʱ��ֱ���ٶ�
float oblow_flag=0;          //0�رգ�1�ȹؽ��٣�2�ȿ�����
float oblow_delay=2000;      //20s�������⽵�ٴ���


//uint8 stopFlagSave[2] = {1,1};  //ͣ����־λ��¼
uint16 RuningTime = 15;
uint16 ConstantSpeed = 200;        //���ٶ�ʱ�ܵ��ٶ�
uint8 stopLineDetected = 0;     //ͣ���߼�� 1��ʾ��⵽
int StartDelayCount = 0;        //������ʱ������
int StopDelayCount = 0;         //ͣ����ʱ������
uint8 StopLineNoDetect = 1;     //����������߱�־λ��0��ʾ��⣬1��ʾ�����
uint8 stopRemember = 0;        //���ڼ�ס��⵽ͣ����,0��ʾδ��⵽
uint8 CarState = 0;    
uint8 touchCnt = 0;
uint16 encoderThreshold = 90;  //�������޷�
uint16 bumpEncoderThreshold = 60;
uint16 annulus_speed = 270;     //Բ���ٶȲ���
uint16 ann_speed_k = 130;

//���ڼ�����ٶ�
uint16 speed_k=130;           //���ι�ʽϵ��
uint16 speed_k2=100;           //���ι�ʽϵ��
uint8 speedUpDelay = 0;    //��ֱ�����ٵ���ʱ����ֹ���г�ֱ��Ƶ��ɲ��
int16 realEncoderVal2 = 0;
//ֱ���ٶȱ���
uint16 max_speed=420;  

//����ٶȱ���
uint16 wan_speed=285;         //���ι�ʽ�ĳ�����
uint16 min_speed=225;         //�����С�ٶ�
uint16 ann_min_speed = 250;

//�����ٶȱ���
uint16 base_speed=273;        //��������ٶ�
uint16 outm_speed=370;        //������ٵ�����ٶ�
uint16 add_speed=3;//4        //����
//ɲ���ٶȱ���
uint16 extraStopVal_Short=10;       //��������ٶ��������
uint16 extraStopVal_Turnin=10;       //��������ٶ��������
uint16 extraStopVal_Long=10;        //ֱ�����ٶ��������

uint16 bra_speed1 = 385;
uint16 bra_speed2 = 350;
uint16 bra_speed3 = 310;
uint16 bra_speed4 = 275;

//�����ٶȱ���
uint16 ra_speed=230;         //�µ��ٶ�
uint16 obcutsp=240;          //�ϰ��������ٶ�
uint16 max2_speed=260;        //���ι�ʽ������ٶ�
uint16 bump_speed = 140;         //��ֵ�ٶ�
//�����жϱ���
uint16 zhidao_ade=50;         //��ֱ�����ж�
uint16 wan_ade=63;            //������ٵ��ж�

//ɲ���жϱ���
uint16 dzhi_top=73;           //ֱ��ɲ�����ж�
uint16 set_othertop=76;       //��һ���˵�����
uint16 set_cothertop=75;      //��һ���˵�����
uint16 wanru_ade=30;          //������ٵ��ж�
uint16 set_yh_top = 76;
uint16 stoptime=12;           //ɲ����ɵ�ʱ��

//ɲ������ж�
int stopaim_speed = 0;            //���ڼ�����ɵ��ж�
int stopaim_cont=0;           //��ɼ��ٵ�����������

//�������ͱ���
int stop_type=0;        //�����ж�ɲ������
int nono_flag=0;        //���ڷ�ֹ��ֱ��ɲ����󻹻���뵽��ֱ��ɲ��
int speed_type=0;       //�ٶȿ������ͱ�־��0��ʾ���ι�ʽѭ����1��ʾֱ�߼��٣�2��ʾ�������
int speedInfo[5]={0};     //�ٶ���Ϣ
//����
//uint8 sendNum=0;
uint8 xf_flag=0;
uint16 brake_test=0;
uint8 bumpDeal=0;
extern uint16 absde[3];
extern int dede[3];    //ƫ�ƫ��ı仯
extern int other_top;           //�����������������
extern uint8 top,top_save[2];
extern float  de_k;
extern float deviation;
extern uint8 stopFlag;          //ͣ����־λ
extern RUNMode_e RunMode;       //
extern IMG_FLAGS IF;
extern IMG_FLAGS LAST_IF;
extern void setTimeBeep_ms(uint8 timeSet, uint8 vt);
extern int TimingCount;            //��ʱ���м�����
extern uint8 DMPInitSet;
void Car_go()
{
      //  ++sendNum;
        GetEncoder();          //��ȡ������ֵ,����ǰ�ٶ�       
        Speed_Set();           //���ݱ�־λ�趨Ŀ���ٶ�

         if (stopFlag == 0)    //δͣ��
         {
           if(RunMode == TIMING_SPEED_RUN) {
               ENABLE_MOTOR;
               SPD.aimSpeed = ConstantSpeed;
               Motro_Shifting();
           }
           else if(RunMode == QUADRA_RUN){
              ENABLE_MOTOR;
              squareFomulTest();
              Motro_Shifting();      //���㲢���PWM������PID�ٶȲ���     
           }
           else{
              ENABLE_MOTOR; 
              Motro_Shifting();      //���㲢���PWM������PID�ٶȲ���  
           }          
         }
         else{                  //ͣ��
           ENABLE_MOTOR; 
           stopCar();
         }     
}
  //top ��de ��while�и���
void Track_type_detect()
{  
    /****************************************************�����ٶ������ж�*************************************************************/
  if(IF.annulus) 
  {                            
    speed_type = 0;
    speedUpDelay = 0;
    if(IF.annulus&&LAST_IF.annulus==0) xf_flag=1;
  }
  else if(IF.obstacle) 
  {                            
    speed_type = 6;
    speedUpDelay = 0;
    if(IF.obstacle&&LAST_IF.obstacle==0) xf_flag=1;
  }
  else if(IF.ramp) 
  {
    speed_type = 0;
    speedUpDelay = 0;
    if(IF.ramp&&LAST_IF.ramp==0) xf_flag=1;
  }
  else if(top>=79&&other_top>=79&&absde[0]<=zhidao_ade&&absde[1]<=zhidao_ade&&absde[2]<=zhidao_ade&&speed_type!=2&&IF.yhds>=70&&IF.annulus==0)//��ֱ��������������
  {
    ++speedUpDelay;
  } 
  else if((top<79||absde[0]>=zhidao_ade||other_top<=set_cothertop||IF.yhds<70)&&(speed_type==1||speed_type==6)&& SPD.nowSpeed > bra_speed1 && (brake_test==0))//��ֱ��ɲ����������(�ﵽ�ϸ��ٶ�)160 ||speed_type==3
  {                           
    speed_type = 2;
    xf_flag=1;
    stop_type=1;
    nono_flag=1;
    speedUpDelay = 0;
  }
  else if((top<=76||absde[0]>=zhidao_ade||other_top<=76||IF.yhds<62)&&(speed_type==1||speed_type==6)&& SPD.nowSpeed > bra_speed2 && (brake_test<=1))//��ֱ��ɲ����������(�ﵽ�ϸ��ٶ�)//140 ||speed_type==3
  {                             
    speed_type = 2;
    xf_flag=1;
    stop_type=1;
    nono_flag=1;
    speedUpDelay = 0;
  }
  else if( (top<=73||absde[0]>=zhidao_ade||other_top<=73 ||IF.yhds<59)&&(speed_type==1||speed_type==6) && SPD.nowSpeed > bra_speed3 && (brake_test<=2))//��ֱ��ɲ����������(�ٶ�δ�ﵽ)120 ||speed_type==3
  {
      speed_type = 2;
      xf_flag=1;
      stop_type=1;        //����ֱ��ɲ����Ŀ���ٶ�
      speedUpDelay = 0;
  }
 else if( (top<=70||absde[0]>=zhidao_ade||other_top<=69 ||IF.yhds<59)&&(speed_type==1||speed_type==6)&& SPD.nowSpeed > bra_speed4 && (brake_test<=3))//��ֱ��ɲ����������(�ٶ�δ�ﵽ)//90 ||speed_type==3
  {
      speed_type = 2;
      xf_flag=1;
      stop_type=1;        //����ֱ��ɲ����Ŀ���ٶ�
      speedUpDelay = 0;
  }
// else if( (top<=65||absde[0]>=25||other_top<=63 ||IF.yhds<59)&&(speed_type==1||speed_type==6) && SPD.nowSpeed >255)//��ֱ��ɲ����������(�ٶ�δ�ﵽ)//60 //||speed_type==3&&dede[0]>0&&dede[1]>0
//  {
//      speed_type = 2;
//      xf_flag=1;
//      stop_type=3;        //����ֱ��ɲ����Ŀ���ٶ�
//      speedUpDelay = 0;
//  }
  else if( (top<=59||absde[0]>=zhidao_ade||other_top<=56 ||IF.yhds<59)&&(speed_type==1||speed_type==6) && (brake_test<=4))//��ֱ��ɲ����������(�ٶ�δ�ﵽ)//40
  {
      speed_type = 2;
      xf_flag=1;
      stop_type=1;        //����ֱ��ɲ����Ŀ���ٶ�
      speedUpDelay = 0;
  }
 else if((absde[0]>=wanru_ade&&dede[0]>0&&dede[1]>0||IF.yhds<68)&&speed_type==3 )//&& SPD.nowSpeed>=250)//������������� (�ٶȴﵽ,��������ʱ)
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ���� wanru_ade�ɵ�
    stop_type=2;
    xf_flag=1;
  } 
  else if(( (absde[0]>=wanru_ade||top<=68||other_top<=66)&&dede[0]>0&&dede[1]>0||IF.yhds<68)&&speed_type==3 && SPD.nowSpeed >275)//&& SPD.nowSpeed>=250)//������������� (�ٶȴﵽ,��������ʱ)
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ���� wanru_ade�ɵ�
    stop_type=2;
    xf_flag=1;
  }
  else if(absde[0]<=wan_ade && dede[0] < 0 && dede[1]<0&&(speed_type==0 ||speed_type==7))//&& IF.annulus==0)// ���������������
  {
    speed_type=3;//���ٶ����ͱ�־��Ϊ�������  base_speed      wan_ade�ɵ�
    speedUpDelay = 0;
  }
  else if(top>=73&&other_top>=73&&absde[0]<=25&&speed_type!=1&&speed_type!=2&&nono_flag==0 && IF.annulus==0)//����������ֱ��
  {
    speed_type=6;//���ٶ����ͱ�־��Ϊ����������ֱ�� dzhi_top �ɵ�
  }
  if(speedUpDelay>=3) speed_type=1;
  
  LAST_IF=IF;
 //last_speed_type = speed_type;  //�����ϴ��ٶ�����
}


void Speed_Set()
{
  
/***************************����ֱ��ɲ�����ٶ�****************************/ 
  if(speed_type!=2) bumpDeal=0;
  if(stop_type==1)//���㳤ֱ��ɲ�����ٶ�
  {
    if( GET_SWITCH5() == SWITCH_ON)
    {
      stopaim_speed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k2*speed_k2)*de_k*de_k)-extraStopVal_Long);
     if(stopaim_speed<min_speed-extraStopVal_Long) stopaim_speed=(int)min_speed-extraStopVal_Long;
      if(stopaim_speed>(max2_speed-extraStopVal_Long))stopaim_speed=(int)(max2_speed-extraStopVal_Long); 
    }
    else stopaim_speed = (int)min_speed;
    if(IF.bump) 
    {
      bumpDeal=1;
    }
    if(bumpDeal) stopaim_speed=bump_speed;
  }
  else if(stop_type==3)//�����ֱ������ɲ�����ٶ�
  {
   if( GET_SWITCH5() == SWITCH_ON)
    {
      stopaim_speed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k2*speed_k2)*de_k*de_k)-extraStopVal_Short);
      if(stopaim_speed<min_speed-extraStopVal_Short)stopaim_speed=(int)min_speed-extraStopVal_Short;
     if(stopaim_speed>(max2_speed-extraStopVal_Short))stopaim_speed=(int)(max2_speed-extraStopVal_Short); 
    }
    else  stopaim_speed=(int)(min_speed);
  }
  else if(stop_type==2)//�������������ɲ�����ٶ�
  {
      
    if(GET_SWITCH5() == SWITCH_ON)
    {
      stopaim_speed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k2*speed_k2)*de_k*de_k)-extraStopVal_Turnin);
      if(stopaim_speed<min_speed-extraStopVal_Turnin)stopaim_speed=(int)min_speed-extraStopVal_Turnin;
      if(stopaim_speed>(max2_speed-extraStopVal_Turnin))stopaim_speed=(int)(max2_speed-extraStopVal_Turnin); 
    }
    else  stopaim_speed=(int)min_speed;
  }
  else if(stop_type == 11){
    stopaim_speed = 0;
  }
  //********************************��������ɲ���ٶ�**************************************//
  
  //********************************�ж�ɲ���Ƿ����***************************************//
  if(SPD.nowSpeed<=stopaim_speed&&speed_type==2)//������ɵ���ʱ�˲�
  {
    stopaim_cont++;
  }
  if(stopaim_cont>=stoptime)//�ﵽɲ����ɱ�־
  {
    stopaim_cont=0;     //�����
    
    if(absde[0]>38)//30
      { 
        nono_flag=0;
        speed_type=0;//δ������������ͽ�����ι�ʽ
      }
      else 
      {
        speed_type=7;
      }
  }
  
  if(absde[0]>38&&speed_type==7)
  {
    speed_type=0;
    nono_flag=0;
  } 
  //*******************************�����ж�ɲ���Ƿ����****************************************//
  
  /*************************************************�����ٶ�����ʵ�ֿ���*********************************************/
  
  if(speed_type==1)//ֱ�����
  {
      SPD.aimSpeed=(int)max_speed;   
  }
  else if(speed_type==0||speed_type==7)//���ι�ʽ����
  {
  //  static float maxDek=0;
    if(GET_SWITCH6() == SWITCH_ON && !IF.annulus  )
    {
      //maxDek=0;
      SPD.aimSpeed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k*speed_k)*de_k*de_k));
      if(SPD.aimSpeed<min_speed)SPD.aimSpeed=(int)min_speed;
      if(SPD.aimSpeed>(max2_speed))SPD.aimSpeed=(int)(max2_speed); 
    }
    else if( IF.annulus )
    {
      if(IF.annulus!=AL1 && IF.annulus!=AR1)
      {
        SPD.aimSpeed=(int)((annulus_speed-(annulus_speed-ann_min_speed)/((float)ann_speed_k*ann_speed_k)*de_k*de_k));
        if(SPD.aimSpeed<ann_min_speed)SPD.aimSpeed=(int)ann_min_speed;
        if(SPD.aimSpeed>(max2_speed))SPD.aimSpeed=(int)(max2_speed); 
      }
      else
      {
        SPD.aimSpeed=ann_min_speed;
      }
    }
    else
    {
       //maxDek=0;
      SPD.aimSpeed=(int)min_speed;
    }
  }
  else if(speed_type==2)//����
  {
    SPD.aimSpeed = stopaim_speed;
  }
  else if(speed_type==3)//�������
  {
    SPD.aimSpeed=SPD.aimSpeed+(int)add_speed;
   if(SPD.aimSpeed>base_speed)SPD.aimSpeed=(int)base_speed;//�޷�

  }
  else if(speed_type==6)//����������ֱ��
  {
    SPD.aimSpeed=(int)outm_speed;
    if(IF.obstacle) SPD.aimSpeed=(int)obcutsp;
  }
  if(IF.ramp) SPD.aimSpeed=(int)ra_speed;

  
  if(SPD.aimSpeed>max_speed)SPD.aimSpeed=(int)max_speed;//�޷�
  if(brake_test&&(speed_type==6||speed_type==1))
  {
    if(brake_test==1) SPD.aimSpeed=(int)bra_speed1;
    if(brake_test==2) SPD.aimSpeed=(int)bra_speed2;
    if(brake_test==3) SPD.aimSpeed=(int)bra_speed3;
    if(brake_test==4) SPD.aimSpeed=(int)bra_speed4;
  }
  //if(IF.annulus&&IF.annulus!=AR1&&IF.annulus!=AL1&&de_k<100) SPD.aimSpeed=245;
  
}


  //�����ٶ��ж���Ϣ
void sendSpeedInfo(){
  int speedTypeWave;

  if(speed_type==1) speedTypeWave = 1500;       //��ֱ��
  else if(speed_type == 6) speedTypeWave = 1200; //�����ֱ��
  else if(speed_type == 3) speedTypeWave = 900; //�������
  else if(speed_type == 0 ) speedTypeWave = 600;      //���ι�ʽ
  else if(speed_type == 2 ) speedTypeWave = 400;
  speedInfo[4] = speedTypeWave;
  //speedInfo[4] = IF.yhtop;
  speedInfo[3] = absde[0]*100;
  speedInfo[2] = other_top*100;
  speedInfo[1] = top*100;
  speedInfo[0] = (int)deviation;
  vcan_sendware(speedInfo,sizeof(speedInfo));

  
}
void squareFomulTest(){  //���ι�ʽ�ٶȲ�������
  
      SPD.aimSpeed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k*speed_k)*de_k*de_k));
      if(SPD.aimSpeed<min_speed)SPD.aimSpeed=(int)min_speed;
      if(SPD.aimSpeed>(max2_speed))SPD.aimSpeed=(int)(max2_speed); 
}
/*
 *  @brief      ��ʱ��
 *  @param      None
 *  @return     None
*/

void Timing_Driving()   //ʹ��DMP��ʼ����ʱ
{
  static uint8 TimingCarState = 0;       //0��ʾδ������1��ʾ��ʱ�У�2��ʾ������
  if(keymsg.key == KEY_START && TimingCarState == 0)      //��������
  { 
    TimingCount = 0;    //��������ʱ��
    LCD_clear(WHITE);   //���� 
    ENABLE_MOTOR;       //ʹ�ܵ��
    TimingCarState = 1;
  }
  
    //��ʾ����ʱ   //��������
  if(keymsg.key == KEY_START && TimingCarState == 1)
  {   
      if(DMP_Delay_Status == DELAY_TIME_UP)
      {
        TimingCarState = 2;
        DMP_Delay_Status = DMP_READ;
        stopFlag = 0;
        TimingCount = 0;
      }
      else
        DMP_Delay_Status = START_INIT;

  }
   if(TimingCarState == 2 && (RuningTime) * 100 <= TimingCount){  //������ ʱ�䵽��
      stopFlag = 1;     //����
      stopReason = TimeUp;
      TimingCount = 0;
      keymsg.key = KEY_U;
      TimingCarState = 0;
    }
}

/*      //ʡ����
void Timing_Driving()
{
  static uint8 TimingCarState = 0;       //0��ʾδ������1��ʾ��ʱ�У�2��ʾ������
  if(keymsg.key == KEY_START && TimingCarState == 0)      //��������
  { 
    TimingCount = 0;    //��������ʱ��
    LCD_clear(WHITE);   //���� 
    ENABLE_MOTOR;       //ʹ�ܵ��
    TimingCarState = 1;
  }
  
    //��ʾ����ʱ   //��������
  if(keymsg.key == KEY_START && TimingCarState == 1 && (TimingCount/100) <= 2)
  {  
    if(TimingCount <= 50){
    //  mLCD_str(70,40,"Timing",BLUE,WHITE);
      mLCD_str(88,60,"3 ! ",BLUE,WHITE);
    }
    else if(TimingCount <= 100)
      mLCD_str(88,60,"2 ! ",BLUE,WHITE);
    else if(TimingCount <= 150)
      mLCD_str(88,60,"1 ! ",BLUE,WHITE);
    else{
      mLCD_str(88,60,"GO !",BLUE,WHITE);
    }  
    
    if(TimingCount >= 200){
      TimingCarState = 2;
      stopFlag = 0;
    }
//    else{
//      mLCD_str(5,30,"Timecnt:",BLUE,WHITE);
//      mLCD_num(75,30,TimingCount,5,BLUE,WHITE);
//    }
  }
   if(TimingCarState == 2 && (RuningTime+2) * 100 <= TimingCount){  //������ ʱ�䵽��
      stopFlag = 1;     //����
      stopReason = TimeUp;
      TimingCount = 0;
      keymsg.key = KEY_U;
      TimingCarState = 0;
    }
}
*/

void Normal_Driving(){  //ʹ��DMP��ʼ����ʱ
  if(CarState == 0 && keymsg.key == KEY_START){  //δ��ʼ�����°�������2
      
      if(DMP_Delay_Status == DELAY_TIME_UP)
      {
        CarState = 2;
        DMP_Delay_Status = DMP_READ;
        stopFlag = 0;      
      }
      else
        DMP_Delay_Status = START_INIT;
  }
  
 if(CarState == 2 )    
  { 
     ENABLE_MOTOR;       //ʹ�ܵ��
     StartDelayCount = 0;      //�������������ʱ���ͣ����
     LCD_clear(WHITE);   //���� 
     StopLineNoDetect = 1;//������һ��ʱ�䲻���������
     CarState = 1;        //��1��ʾ�ѿ���
     stopFlag = 0;       //��ͣ����־λ
  }
  
  if(CarState == 1)   //�ѿ�������£���ʱ3����ټ��ͣ����
  {
    if(StartDelay_s(3)){ 
      StopLineNoDetect = 0; 
      CarState = 3; 
     
    }
    else{ 
      StopLineNoDetect = 1;
      StopDelayCount = 0;
    }
  }
  
    if(CarState == 3 && StopLineNoDetect == 0 && stopLineDetected == 1 ){    //������ͣ���ߣ���⵽ͣ���ߣ����ѿ���             
      stopRemember = 1;     //��ס��⵽ͣ��  
    }
     
  if(stopRemember == 1){     //��ס��            
    if(StopDelay_10ms(10)){  //��ʱ3��ͣ�������÷�����־λ��������Ϣ����������
      stopFlag = 1;
      stopReason = StopLineDetected;
      CarState = 0;
      keymsg.key = KEY_U;      
      StartDelayCount = 0; 
      StopDelayCount = 0;
      stopRemember = 0;
    }
  }
}

      //��������ʱ
int StartDelay_s(int seconds)
{
   if(StartDelayCount/100 >= seconds) return 1;
   else return 0;
}
    //ͣ������ʱ������stopFlag��0��ſ�ʹ��
int StopDelay_10ms(int mil_seconds)
{
    if(StopDelayCount >= mil_seconds) return 1;
    else return 0;
}

//void Normal_Driving()         //����������ʱ
//{
//  if(CarState == 0 && keymsg.key == KEY_START)  //δ��ʼ�����°�������2
//  {   
//    if (StartDelayCount <= 50)       //��ʱ��ʾLCD����󷢳�
//      mLCD_str(88,60,"3 ! ",BLUE,WHITE);
//    else if (StartDelayCount <= 100) 
//      mLCD_str(88,60,"2 ! ",BLUE,WHITE);
//    else if (StartDelayCount <= 150) 
//      mLCD_str(88,60,"1 ! ",BLUE,WHITE);
//    else{
//      mLCD_str(88,60,"GO !",BLUE,WHITE);
//      CarState = 2;
//    }
//  } 
//  
//  if(CarState == 2 )    //���º󿪳�,�˺�TimingCount++
//  { 
//     ENABLE_MOTOR;       //ʹ�ܵ��
//     LCD_clear(WHITE);   //���� 
//     CarState = 1;        //��1��ʾ�ѿ���
//     stopFlag = 0;       //��ͣ����־λ
//  }
//  
//  if(CarState == 1 && stopLineDetected == 1)   //�ѿ�������£���ʱ3����ټ��ͣ����
//  {
//    stopRemember = 1;  
//  }
//  
//  if(stopRemember == 1){ 
//    //��ס��            
//    if( StopDelay_10ms(25) ){  //��ʱ3��ͣ�������÷�����־λ��������Ϣ����������
//      stopFlag = 1;
//      stopReason = StopLineDetected;
//      CarState = 0;
//      keymsg.key = KEY_U;      
//      StartDelayCount = 0; 
//      StopDelayCount = 0;
//      stopRemember = 0;
//    }
//  }
//}

/*      ����������ʱ_ʡ���õ�
void Normal_Driving()
{
  //static uint8 CarState = 0;            // 0����δ��ʼ��2������start��1�����ѿ�
  
  if(CarState == 0 && keymsg.key == KEY_START)  //δ��ʼ�����°�������2
  {  
   // mLCD_str(5,30,"Timecnt:",BLUE,WHITE);
   // mLCD_num(75,30,StartDelayCount,5,BLUE,WHITE);
    
    if (StartDelayCount <= 50) {       //��ʱ��ʾLCD����󷢳�
      //mLCD_str(70,40,"Normal",BLUE,WHITE);
      mLCD_str(88,60,"3 ! ",BLUE,WHITE);
    }
    else if (StartDelayCount <= 100) 
      mLCD_str(88,60,"2 ! ",BLUE,WHITE);
    else if (StartDelayCount <= 150) 
      mLCD_str(88,60,"1 ! ",BLUE,WHITE);
    else{
      mLCD_str(88,60,"GO !",BLUE,WHITE);
      CarState = 2;
      //setTimeBeep_ms(HIGH,200);
    }
  } 
  
  if(CarState == 2 )    //���º󿪳�,�˺�TimingCount++
  { 
     ENABLE_MOTOR;       //ʹ�ܵ��
     StartDelayCount = 0;      //�������������ʱ���ͣ����
     LCD_clear(WHITE);   //���� 
     StopLineNoDetect = 1;//������һ��ʱ�䲻���������
     CarState = 1;        //��1��ʾ�ѿ���
     stopFlag = 0;       //��ͣ����־λ
  }
  
  if(CarState == 1)   //�ѿ�������£���ʱ3����ټ��ͣ����
  {
    if(StartDelay_s(3)){ 
      StopLineNoDetect = 0; 
      CarState = 3; 
     // setTimeBeep_ms(HIGH,1000);
    }
    else{ 
      StopLineNoDetect = 1;
      StopDelayCount = 0;
    }
  }
  
    if(CarState == 3 && StopLineNoDetect == 0 && stopLineDetected == 1 ){    //������ͣ���ߣ���⵽ͣ���ߣ����ѿ���             
      stopRemember = 1;     //��ס��⵽ͣ�� 
     // StopDelayCount = 0;
    }
      //
  if(stopRemember == 1){     //��ס��            
    if(StopDelay_10ms(10)){  //��ʱ3��ͣ�������÷�����־λ��������Ϣ����������
      stopFlag = 1;
      stopReason = StopLineDetected;
      CarState = 0;
      keymsg.key = KEY_U;      
      StartDelayCount = 0; 
      StopDelayCount = 0;
      stopRemember = 0;
    }
  }
}
*/

void testDriving(){
  if(keymsg.key == KEY_START && stopLineDetected == 0 && stopReason != RunOutLine)
    stopFlag = 0;
  else if(stopLineDetected == 1){
    stopFlag = 1;
    stopReason = StopLineDetected;
    keymsg.key = KEY_U;
  }
}