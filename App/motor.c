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
int sramp_time=0;//用于无法识别小坡的特殊处理的计时
int sramp_cutflag=0;//0表示无需限制直道速度与关闭坡道，1表示需限制直道速与关闭坡道
float sramp_maxspeed=310;    //20s后开启特殊坡道处理时的直道速度
float oblow_flag=0;          //0关闭，1先关降速，2先开降速
float oblow_delay=2000;      //20s后开启特殊降速处理


//uint8 stopFlagSave[2] = {1,1};  //停车标志位记录
uint16 RuningTime = 15;
uint16 ConstantSpeed = 200;        //恒速定时跑的速度
uint8 stopLineDetected = 0;     //停车线检测 1表示检测到
int StartDelayCount = 0;        //发车延时计数器
int StopDelayCount = 0;         //停车延时计数器
uint8 StopLineNoDetect = 1;     //不检测起跑线标志位，0表示检测，1表示不检测
uint8 stopRemember = 0;        //用于记住检测到停车线,0表示未检测到
uint8 CarState = 0;    
uint8 touchCnt = 0;
uint16 encoderThreshold = 90;  //编码器限幅
uint16 bumpEncoderThreshold = 60;
uint16 annulus_speed = 270;     //圆环速度参数
uint16 ann_speed_k = 130;

//用于计算的速度
uint16 speed_k=130;           //二次公式系数
uint16 speed_k2=100;           //二次公式系数
uint8 speedUpDelay = 0;    //长直道加速的延时，防止误判长直道频繁刹车
int16 realEncoderVal2 = 0;
//直道速度变量
uint16 max_speed=420;  

//弯道速度变量
uint16 wan_speed=285;         //二次公式的常数项
uint16 min_speed=225;         //弯道最小速度
uint16 ann_min_speed = 250;

//加速速度变量
uint16 base_speed=273;        //出弯基础速度
uint16 outm_speed=370;        //出弯加速的最大速度
uint16 add_speed=3;//4        //加速
//刹车速度变量
uint16 extraStopVal_Short=10;       //短入弯减速多减得余量
uint16 extraStopVal_Turnin=10;       //弯入弯减速多减得余量
uint16 extraStopVal_Long=10;        //直到减速多减得余量

uint16 bra_speed1 = 385;
uint16 bra_speed2 = 350;
uint16 bra_speed3 = 310;
uint16 bra_speed4 = 275;

//其他速度变量
uint16 ra_speed=230;         //坡道速度
uint16 obcutsp=240;          //障碍减掉的速度
uint16 max2_speed=260;        //二次公式的最大速度
uint16 bump_speed = 140;         //阈值速度
//加速判断变量
uint16 zhidao_ade=50;         //入直道的判断
uint16 wan_ade=63;            //出弯加速的判断

//刹车判断变量
uint16 dzhi_top=73;           //直道刹车的判断
uint16 set_othertop=76;       //另一顶端的设置
uint16 set_cothertop=75;      //另一顶端的设置
uint16 wanru_ade=30;          //弯道减速的判断
uint16 set_yh_top = 76;
uint16 stoptime=12;           //刹车完成的时间

//刹车完成判断
int stopaim_speed = 0;            //用于减速完成的判断
int stopaim_cont=0;           //完成减速的周期数计数

//赛道类型变量
int stop_type=0;        //用于判断刹车种类
int nono_flag=0;        //用于防止长直到刹车完后还会进入到短直道刹车
int speed_type=0;       //速度控制类型标志，0表示二次公式循迹，1表示直线加速，2表示入弯减速
int speedInfo[5]={0};     //速度信息
//其他
//uint8 sendNum=0;
uint8 xf_flag=0;
uint16 brake_test=0;
uint8 bumpDeal=0;
extern uint16 absde[3];
extern int dede[3];    //偏差，偏差的变化
extern int other_top;           //入弯的其他辅助条件
extern uint8 top,top_save[2];
extern float  de_k;
extern float deviation;
extern uint8 stopFlag;          //停车标志位
extern RUNMode_e RunMode;       //
extern IMG_FLAGS IF;
extern IMG_FLAGS LAST_IF;
extern void setTimeBeep_ms(uint8 timeSet, uint8 vt);
extern int TimingCount;            //定时运行计数器
extern uint8 DMPInitSet;
void Car_go()
{
      //  ++sendNum;
        GetEncoder();          //获取编码器值,即当前速度       
        Speed_Set();           //根据标志位设定目标速度

         if (stopFlag == 0)    //未停车
         {
           if(RunMode == TIMING_SPEED_RUN) {
               ENABLE_MOTOR;
               SPD.aimSpeed = ConstantSpeed;
               Motro_Shifting();
           }
           else if(RunMode == QUADRA_RUN){
              ENABLE_MOTOR;
              squareFomulTest();
              Motro_Shifting();      //计算并输出PWM，更新PID速度参数     
           }
           else{
              ENABLE_MOTOR; 
              Motro_Shifting();      //计算并输出PWM，更新PID速度参数  
           }          
         }
         else{                  //停车
           ENABLE_MOTOR; 
           stopCar();
         }     
}
  //top 与de 在while中更新
void Track_type_detect()
{  
    /****************************************************进行速度类型判断*************************************************************/
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
  else if(top>=79&&other_top>=79&&absde[0]<=zhidao_ade&&absde[1]<=zhidao_ade&&absde[2]<=zhidao_ade&&speed_type!=2&&IF.yhds>=70&&IF.annulus==0)//长直道加速条件满足
  {
    ++speedUpDelay;
  } 
  else if((top<79||absde[0]>=zhidao_ade||other_top<=set_cothertop||IF.yhds<70)&&(speed_type==1||speed_type==6)&& SPD.nowSpeed > bra_speed1 && (brake_test==0))//长直道刹车条件满足(达到较高速度)160 ||speed_type==3
  {                           
    speed_type = 2;
    xf_flag=1;
    stop_type=1;
    nono_flag=1;
    speedUpDelay = 0;
  }
  else if((top<=76||absde[0]>=zhidao_ade||other_top<=76||IF.yhds<62)&&(speed_type==1||speed_type==6)&& SPD.nowSpeed > bra_speed2 && (brake_test<=1))//长直道刹车条件满足(达到较高速度)//140 ||speed_type==3
  {                             
    speed_type = 2;
    xf_flag=1;
    stop_type=1;
    nono_flag=1;
    speedUpDelay = 0;
  }
  else if( (top<=73||absde[0]>=zhidao_ade||other_top<=73 ||IF.yhds<59)&&(speed_type==1||speed_type==6) && SPD.nowSpeed > bra_speed3 && (brake_test<=2))//长直道刹车条件满足(速度未达到)120 ||speed_type==3
  {
      speed_type = 2;
      xf_flag=1;
      stop_type=1;        //给短直道刹车的目标速度
      speedUpDelay = 0;
  }
 else if( (top<=70||absde[0]>=zhidao_ade||other_top<=69 ||IF.yhds<59)&&(speed_type==1||speed_type==6)&& SPD.nowSpeed > bra_speed4 && (brake_test<=3))//长直道刹车条件满足(速度未达到)//90 ||speed_type==3
  {
      speed_type = 2;
      xf_flag=1;
      stop_type=1;        //给短直道刹车的目标速度
      speedUpDelay = 0;
  }
// else if( (top<=65||absde[0]>=25||other_top<=63 ||IF.yhds<59)&&(speed_type==1||speed_type==6) && SPD.nowSpeed >255)//长直道刹车条件满足(速度未达到)//60 //||speed_type==3&&dede[0]>0&&dede[1]>0
//  {
//      speed_type = 2;
//      xf_flag=1;
//      stop_type=3;        //给短直道刹车的目标速度
//      speedUpDelay = 0;
//  }
  else if( (top<=59||absde[0]>=zhidao_ade||other_top<=56 ||IF.yhds<59)&&(speed_type==1||speed_type==6) && (brake_test<=4))//长直道刹车条件满足(速度未达到)//40
  {
      speed_type = 2;
      xf_flag=1;
      stop_type=1;        //给短直道刹车的目标速度
      speedUpDelay = 0;
  }
 else if((absde[0]>=wanru_ade&&dede[0]>0&&dede[1]>0||IF.yhds<68)&&speed_type==3 )//&& SPD.nowSpeed>=250)//出弯再入弯减速 (速度达到,不进行延时)
  {
    speed_type=2;//将速度类型标志置为刹车档 wanru_ade可调
    stop_type=2;
    xf_flag=1;
  } 
  else if(( (absde[0]>=wanru_ade||top<=68||other_top<=66)&&dede[0]>0&&dede[1]>0||IF.yhds<68)&&speed_type==3 && SPD.nowSpeed >275)//&& SPD.nowSpeed>=250)//出弯再入弯减速 (速度达到,不进行延时)
  {
    speed_type=2;//将速度类型标志置为刹车档 wanru_ade可调
    stop_type=2;
    xf_flag=1;
  }
  else if(absde[0]<=wan_ade && dede[0] < 0 && dede[1]<0&&(speed_type==0 ||speed_type==7))//&& IF.annulus==0)// 出弯加速条件满足
  {
    speed_type=3;//将速度类型标志置为出弯加速  base_speed      wan_ade可调
    speedUpDelay = 0;
  }
  else if(top>=73&&other_top>=73&&absde[0]<=25&&speed_type!=1&&speed_type!=2&&nono_flag==0 && IF.annulus==0)//出弯遇到短直道
  {
    speed_type=6;//将速度类型标志置为出弯遇到短直道 dzhi_top 可调
  }
  if(speedUpDelay>=3) speed_type=1;
  
  LAST_IF=IF;
 //last_speed_type = speed_type;  //更新上次速度类型
}


void Speed_Set()
{
  
/***************************计算直道刹车的速度****************************/ 
  if(speed_type!=2) bumpDeal=0;
  if(stop_type==1)//计算长直道刹车的速度
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
  else if(stop_type==3)//计算短直道入弯刹车的速度
  {
   if( GET_SWITCH5() == SWITCH_ON)
    {
      stopaim_speed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k2*speed_k2)*de_k*de_k)-extraStopVal_Short);
      if(stopaim_speed<min_speed-extraStopVal_Short)stopaim_speed=(int)min_speed-extraStopVal_Short;
     if(stopaim_speed>(max2_speed-extraStopVal_Short))stopaim_speed=(int)(max2_speed-extraStopVal_Short); 
    }
    else  stopaim_speed=(int)(min_speed);
  }
  else if(stop_type==2)//计算出弯再入弯刹车的速度
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
  //********************************结束计算刹车速度**************************************//
  
  //********************************判断刹车是否完成***************************************//
  if(SPD.nowSpeed<=stopaim_speed&&speed_type==2)//减速完成的延时滤波
  {
    stopaim_cont++;
  }
  if(stopaim_cont>=stoptime)//达到刹车完成标志
  {
    stopaim_cont=0;     //清计数
    
    if(absde[0]>38)//30
      { 
        nono_flag=0;
        speed_type=0;//未满足加速条件就进入二次公式
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
  //*******************************结束判断刹车是否完成****************************************//
  
  /*************************************************根据速度类型实现控速*********************************************/
  
  if(speed_type==1)//直达加速
  {
      SPD.aimSpeed=(int)max_speed;   
  }
  else if(speed_type==0||speed_type==7)//二次公式控制
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
  else if(speed_type==2)//减速
  {
    SPD.aimSpeed = stopaim_speed;
  }
  else if(speed_type==3)//出弯加速
  {
    SPD.aimSpeed=SPD.aimSpeed+(int)add_speed;
   if(SPD.aimSpeed>base_speed)SPD.aimSpeed=(int)base_speed;//限幅

  }
  else if(speed_type==6)//出弯遇到短直到
  {
    SPD.aimSpeed=(int)outm_speed;
    if(IF.obstacle) SPD.aimSpeed=(int)obcutsp;
  }
  if(IF.ramp) SPD.aimSpeed=(int)ra_speed;

  
  if(SPD.aimSpeed>max_speed)SPD.aimSpeed=(int)max_speed;//限幅
  if(brake_test&&(speed_type==6||speed_type==1))
  {
    if(brake_test==1) SPD.aimSpeed=(int)bra_speed1;
    if(brake_test==2) SPD.aimSpeed=(int)bra_speed2;
    if(brake_test==3) SPD.aimSpeed=(int)bra_speed3;
    if(brake_test==4) SPD.aimSpeed=(int)bra_speed4;
  }
  //if(IF.annulus&&IF.annulus!=AR1&&IF.annulus!=AL1&&de_k<100) SPD.aimSpeed=245;
  
}


  //发送速度判断信息
void sendSpeedInfo(){
  int speedTypeWave;

  if(speed_type==1) speedTypeWave = 1500;       //长直道
  else if(speed_type == 6) speedTypeWave = 1200; //出弯短直道
  else if(speed_type == 3) speedTypeWave = 900; //出弯加速
  else if(speed_type == 0 ) speedTypeWave = 600;      //二次公式
  else if(speed_type == 2 ) speedTypeWave = 400;
  speedInfo[4] = speedTypeWave;
  //speedInfo[4] = IF.yhtop;
  speedInfo[3] = absde[0]*100;
  speedInfo[2] = other_top*100;
  speedInfo[1] = top*100;
  speedInfo[0] = (int)deviation;
  vcan_sendware(speedInfo,sizeof(speedInfo));

  
}
void squareFomulTest(){  //二次公式速度参数测试
  
      SPD.aimSpeed=(int)((wan_speed-(wan_speed-min_speed)/((float)speed_k*speed_k)*de_k*de_k));
      if(SPD.aimSpeed<min_speed)SPD.aimSpeed=(int)min_speed;
      if(SPD.aimSpeed>(max2_speed))SPD.aimSpeed=(int)(max2_speed); 
}
/*
 *  @brief      定时跑
 *  @param      None
 *  @return     None
*/

void Timing_Driving()   //使用DMP初始化延时
{
  static uint8 TimingCarState = 0;       //0表示未发车，1表示延时中，2表示运行中
  if(keymsg.key == KEY_START && TimingCarState == 0)      //按键按下
  { 
    TimingCount = 0;    //重置运行时间
    LCD_clear(WHITE);   //清屏 
    ENABLE_MOTOR;       //使能电机
    TimingCarState = 1;
  }
  
    //显示倒计时   //按键弹起
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
   if(TimingCarState == 2 && (RuningTime) * 100 <= TimingCount){  //运行中 时间到后
      stopFlag = 1;     //重置
      stopReason = TimeUp;
      TimingCount = 0;
      keymsg.key = KEY_U;
      TimingCarState = 0;
    }
}

/*      //省赛用
void Timing_Driving()
{
  static uint8 TimingCarState = 0;       //0表示未发车，1表示延时中，2表示运行中
  if(keymsg.key == KEY_START && TimingCarState == 0)      //按键按下
  { 
    TimingCount = 0;    //重置运行时间
    LCD_clear(WHITE);   //清屏 
    ENABLE_MOTOR;       //使能电机
    TimingCarState = 1;
  }
  
    //显示倒计时   //按键弹起
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
   if(TimingCarState == 2 && (RuningTime+2) * 100 <= TimingCount){  //运行中 时间到后
      stopFlag = 1;     //重置
      stopReason = TimeUp;
      TimingCount = 0;
      keymsg.key = KEY_U;
      TimingCarState = 0;
    }
}
*/

void Normal_Driving(){  //使用DMP初始化延时
  if(CarState == 0 && keymsg.key == KEY_START){  //未开始，按下按键后置2
      
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
     ENABLE_MOTOR;       //使能电机
     StartDelayCount = 0;      //清计数，用于延时检测停车线
     LCD_clear(WHITE);   //清屏 
     StopLineNoDetect = 1;//开车后一段时间不检测起跑线
     CarState = 1;        //置1表示已开车
     stopFlag = 0;       //清停车标志位
  }
  
  if(CarState == 1)   //已开车情况下，延时3秒后，再检测停车线
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
  
    if(CarState == 3 && StopLineNoDetect == 0 && stopLineDetected == 1 ){    //允许检测停车线，检测到停车线，且已开车             
      stopRemember = 1;     //记住检测到停车  
    }
     
  if(stopRemember == 1){     //记住后            
    if(StopDelay_10ms(10)){  //延时3秒停车，重置发车标志位、按键消息，计数清零
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

      //发车用延时
int StartDelay_s(int seconds)
{
   if(StartDelayCount/100 >= seconds) return 1;
   else return 0;
}
    //停车用延时，仅在stopFlag置0后才可使用
int StopDelay_10ms(int mil_seconds)
{
    if(StopDelayCount >= mil_seconds) return 1;
    else return 0;
}

//void Normal_Driving()         //无起跑线延时
//{
//  if(CarState == 0 && keymsg.key == KEY_START)  //未开始，按下按键后置2
//  {   
//    if (StartDelayCount <= 50)       //延时显示LCD，最后发车
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
//  if(CarState == 2 )    //按下后开车,此后TimingCount++
//  { 
//     ENABLE_MOTOR;       //使能电机
//     LCD_clear(WHITE);   //清屏 
//     CarState = 1;        //置1表示已开车
//     stopFlag = 0;       //清停车标志位
//  }
//  
//  if(CarState == 1 && stopLineDetected == 1)   //已开车情况下，延时3秒后，再检测停车线
//  {
//    stopRemember = 1;  
//  }
//  
//  if(stopRemember == 1){ 
//    //记住后            
//    if( StopDelay_10ms(25) ){  //延时3秒停车，重置发车标志位、按键消息，计数清零
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

/*      带起跑线延时_省赛用的
void Normal_Driving()
{
  //static uint8 CarState = 0;            // 0代表未开始，2代表按下start，1代表已开
  
  if(CarState == 0 && keymsg.key == KEY_START)  //未开始，按下按键后置2
  {  
   // mLCD_str(5,30,"Timecnt:",BLUE,WHITE);
   // mLCD_num(75,30,StartDelayCount,5,BLUE,WHITE);
    
    if (StartDelayCount <= 50) {       //延时显示LCD，最后发车
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
  
  if(CarState == 2 )    //按下后开车,此后TimingCount++
  { 
     ENABLE_MOTOR;       //使能电机
     StartDelayCount = 0;      //清计数，用于延时检测停车线
     LCD_clear(WHITE);   //清屏 
     StopLineNoDetect = 1;//开车后一段时间不检测起跑线
     CarState = 1;        //置1表示已开车
     stopFlag = 0;       //清停车标志位
  }
  
  if(CarState == 1)   //已开车情况下，延时3秒后，再检测停车线
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
  
    if(CarState == 3 && StopLineNoDetect == 0 && stopLineDetected == 1 ){    //允许检测停车线，检测到停车线，且已开车             
      stopRemember = 1;     //记住检测到停车 
     // StopDelayCount = 0;
    }
      //
  if(stopRemember == 1){     //记住后            
    if(StopDelay_10ms(10)){  //延时3秒停车，重置发车标志位、按键消息，计数清零
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