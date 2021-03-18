#include "common.h"
#include "MK60_gpio.h"
#include "MK60_ftm.h"
#include "m_Motro.h"
#include "m_Switch.h"
#include "control.h"
#define MBB 0
#define LBB 1
#define PID 2


/* 全局变量 */
IncrementalPID_st Motro_PID;          //电机控制PID结构体
SPEEDInfo_st SPD = {0};    		//速度信息结构体

int PreREF = 0;
uint16 turnILimitsub = 50;       //入弯限积分
uint16 change_ki=4;        //存在flash中，由按键调节 //变速积分的基础ki
uint16 change_kib=6; 
float kib=25;

int change_aimspeed =0;
int Bob_flag = 0;
float U_D = 0;
       
float kp=180;    //bb 120 40 
uint16 ki=11;
float kd=10; 
int Pre_speed_type = 0;
float V_battery;
uint8 stallProtectFlag = 0;
/* 内部函数声明 */
static void motroPIDs_Init();
static int16 calc_vPID(IncrementalPID_st *pp);

extern int now_speed;
//extern int aim_speed;

extern int16 realEncoderVal2;
extern int speed_type;
extern uint8 stopFlag;  
extern uint16 absde[3];
int16 xf_delay=0;
int16 gs_delay=0;
//int16 NoPwmTime=0;
extern uint16 encoderThreshold ;
extern uint16 bumpEncoderThreshold;
extern IMG_FLAGS IF;
extern uint8 xf_flag;
extern uint8 bumpDeal;
/********************************PID结构体初始化******************************
 *      
 *     函数功能：PID结构体内部参数初始化
 *
*******************************************************************************/
void motroPIDs_Init()
{
	/* PID结构体初始化 */
    Motro_PID.REF = 0;                          //目标速度
    Motro_PID.FeedBack = 0;                     //当前速度
  
    Motro_PID.PreError = 0;
    Motro_PID.PrePreError = 0;
    Motro_PID.FeedBackSave[0] = Motro_PID.FeedBackSave[1] = Motro_PID.FeedBackSave[2] = 0;
    
  
    Motro_PID.Kp = (uint16)(kp);    
    Motro_PID.Ki = (uint16)(ki);
    Motro_PID.Kd = (uint16)(kd);
  
    Motro_PID.PID_Out = 0; 
}

/***********************************电机,编码器初始化*********************************/
void Motro_Init()
{
    motroPIDs_Init();
    
    // IO口初始化(电机驱动使能)
    gpio_init(MOTOR_EN,GPO,HIGH);

    // 初始化电机PWM
//    ftm_pwm_init(MOTOR_FTM, MOTOR_PWM1_CH,MOTOR_HZ,0);      
  //  ftm_pwm_init(MOTOR_FTM, MOTOR_PWM2_CH,MOTOR_HZ,0);  
    ftm_pwm_init(MOTOR_FTM, MOTOR_PWM3_CH,MOTOR_HZ,0);      
    ftm_pwm_init(MOTOR_FTM, MOTOR_PWM4_CH,MOTOR_HZ,0);        //前转
    
	// 初始化编码器正交解码
    ftm_quad_init(QUAD_FTM);
}

/************************************电机停止**********************************/
void StopMotro()
{   

    FTM_CnV_REG(FTMN[MOTOR_FTM], MOTOR_PWM3_CH) = 0;
    FTM_CnV_REG(FTMN[MOTOR_FTM], MOTOR_PWM4_CH) = 0;
}

/*******************************************************************************
 *     函数功能：获取右侧编码器值
 *     返回值：编码器在一个控制周期内的脉冲值 
*******************************************************************************/
void GetEncoder()
{   
  /*弯道过程中出现的出现的空转由于pwm够大不会其进行向上限幅*/
	SPD.var[2] = SPD.var[1];
	SPD.var[1] = SPD.var[0];
	SPD.speed = -(int16)((4500.0/4096.0)*ftm_quad_get(QUAD_FTM));
        ftm_quad_clean(QUAD_FTM);         //清FTM
#if BOOM_SD_SAVE
        SPD.bmqSpeed[2] = SPD.bmqSpeed[1];
	SPD.bmqSpeed[1] = SPD.bmqSpeed[0];
        SPD.bmqSpeed[0]=SPD.speed ;
#endif
        float speedCut;
        if(bumpDeal) speedCut=bumpEncoderThreshold/10.0;
        else speedCut=encoderThreshold/10.0;  
        if((GET_SWITCH4() == SWITCH_OFF||bumpDeal&&speed_type==2)&&xf_flag&&(stopFlag==1||Motro_PID.PID_Out<0)&&SPD.var[1]-SPD.speed>speedCut) //stopFlag==1||
        {
          SPD.var[0]=SPD.var[1]-speedCut;
          xf_delay=10;
        }               
        else if(xf_delay&&SPD.var[1]-SPD.speed>speedCut)
        {
          SPD.var[0]=SPD.var[1]-speedCut;
          --xf_delay;
          xf_flag=0;
          gs_delay=10;
        }
        else
          SPD.var[0] = (SPD.speed+SPD.var[1]+SPD.var[2]) / 3; 

        SPD.nowSpeed = (int16) SPD.var[0];
        
 
        
}
void decimalAcc(float *integerPart, float *decimalPart){      //累计小数
  
  float temp = *integerPart;
  *decimalPart +=( temp - (int)temp );        //小数部分累加
  *integerPart = (int)(*integerPart);               //取整
  
  if(*decimalPart>=1){
    *decimalPart -= 1;
    *integerPart += 1;
  }
}       
/*********************************增量式PID算法*********************************
 *      
 *     函数功能：增量式PID计算电机输出PWM值
 *     参数：结构体 PIDM 指针 
 *     返回值：电机PWM值
*******************************************************************************/
int16 calc_vPID(IncrementalPID_st *pp)
{
  int16 error;
  float duty = 0.0;
  float once_i;
  float kp_t,ki_index,kd_t;
  static uint8 turnInRem = 0;  //对编码器限幅只减一次
  int16 turnInJudge[2] = {0};       
  
  kp_t = (float)( (pp->Kp)/10.0 );   //取出P I D系数
  kd_t = (float)( (pp->Kd)/10.0 );
 // if(xf_delay)kp_t=0.5*kp_t;
  error = pp->REF - pp->FeedBack;
  if(error + pp->PreError>= 0)
    ki_index = (change_ki/10.0) - (change_ki/10.0) / (1 + exp(change_kib - 0.2*abs(error)));    //变积分控制
  else
    ki_index = ki/10.0;//0.9;//ki - ki / (1 + exp(kib - 0.2*abs(error)));
  
  U_D = kd_t *(error - pp->PreError)*0.5+0.5*U_D;
  once_i = 0.5* ki_index * (error + pp->PreError);
  duty = SPD.AC_P + kp_t *error + U_D;
  
  if(duty>-999&&duty<999){
    duty += once_i;
    if(duty>999){
      float temp;
      temp = duty-999;
      once_i-=temp;
      duty=999;
    }
    if(duty<-999){
      float temp;
      temp = duty+999;
      once_i-=temp;
      duty=-999;
    }
    SPD.AC_P += once_i;
  }
  else if((duty>=999&&once_i<0)||(duty<=-999&&once_i>0)){
    SPD.AC_P += once_i;
    duty += once_i;
  }
  
      /*        入弯限积分部分         */  
  turnInJudge[0] = Servo_PID.error;     //error
  turnInJudge[1] = Servo_PID.error - Servo_PID.preError;        //d_error
  
  if( (turnInJudge[1]<0 && turnInJudge[0]<0)||(turnInJudge[1]>0 && turnInJudge[0]>0) && absde[0] >= 59 && absde[0] <= 77 && SPD.AC_P > 100 && turnInRem !=2)      //入弯情况 ,absde>=62,积分>100,且还未减过积分
    turnInRem = 1;      //置1 表示需要减积分
  
  if(turnInRem == 1){
    turnInRem = 2;     //减过后置2
    SPD.AC_P -= turnILimitsub;          // 减额外裕量
  }
  
  if( (turnInJudge[1]>0 && turnInJudge[0]<0)||(turnInJudge[1]<0 && turnInJudge[0]>0) )
    turnInRem = 0;  //出弯时清标志位
  
     /*        入弯限积分部分         */  
  
//  if(xf_flag){
//    SPD.AC_P =300;
//    duty = SPD.AC_P + kp_t *error + U_D;
//  }
  
  //duty += bra_speed3*abs(Servo_PID.PID_Out)/1100.0; 
  
  pp->PrePreError=pp->PreError;
  pp->PreError = error;
  
  /*
  float temp=0;
  for(int j=0;j<5;j++)
    temp += adc_once(ADC0_DP1, ADC_10bit);
  
  V_battery= 0.16*temp+0.2*V_battery;
  */
  
  pp->PID_Out = (int16)(duty/*850/V_battery*/);
  return (pp->PID_Out);
    
}
 
/***********************************电机控制************************************
 *      
 *     函数功能：PWM控制电机转速，更新电机控制PID中的目标速度与当前速度
 *     参数：设定的目标速度（编码器值）
 *     返回值：void
*******************************************************************************/
void Motro_Shifting()
{   
  static uint16 stallCount = 0;       //堵转保护
    Motro_PID.REF = SPD.aimSpeed;
    
    Motro_PID.FeedBack = SPD.nowSpeed;    //读取编码器值

    Motro_PID.FeedBackSave[2] = Motro_PID.FeedBackSave[1];
    Motro_PID.FeedBackSave[1] = Motro_PID.FeedBackSave[0];
    Motro_PID.FeedBackSave[0] = SPD.nowSpeed;              //记录前三次速度
    
    SPD.MotroPWM = calc_vPID(&Motro_PID);
    //SPD.MotroPWM +=bra_speed3*abs(Servo_PID.PID_Out)/1100.0; 
    if(SPD.nowSpeed < 80 && SPD.MotroPWM > 900){       //计数
      stallCount++;
      if(stallCount > 500) stallCount = 500;
    }
    else{
      stallCount = 0;
    }
    
    if(stallCount >= 50) stallProtectFlag = 1;  //超0.5s，停电机
    if(stallProtectFlag != 1){
      if(SPD.MotroPWM >= 0){
            if(SPD.MotroPWM >999)
              SPD.MotroPWM =999;

            ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM3_CH, 0);
            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,SPD.MotroPWM); 
          }
      else{
            if(SPD.MotroPWM <-999)
              SPD.MotroPWM =-999;
//            if(IF.bump&&xf_delay)
//             ABSControl(-SPD.MotroPWM);
//            else
         //   {
              ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,0);
              ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM3_CH,-SPD.MotroPWM);
         //   }
          }
    }
    else{
      ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,0);
      ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM3_CH,0);
    }
   
}
void stopCar()
{   
//      if(SPD.speed >SPD.nowSpeed*0.2&&SPD.nowSpeed>15)
//          {
// 
//            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,0);
//            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM3_CH,500);
//          }
//      else
//          {
//            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,0);
//            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM3_CH,0);
//          }
//   
        static int16 time=0;
        static uint8 noPwm=0;
        if(SPD.nowSpeed>20&&noPwm==0)
          {
            time=100;
            uint16 pwm=SPD.nowSpeed*5;
            if(pwm>999) pwm=999;
            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,0);
            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM3_CH,pwm);
          }
      else
          {
            if(time>0) 
            {
              noPwm=1;
              time--;
            }
            else
              noPwm=0;
            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM4_CH,0);
            ftm_pwm_duty(MOTOR_FTM,MOTOR_PWM3_CH,0);
          }
}
void ABSControl(int16 pwm)
{
	if (SPD.speed >= SPD.nowSpeed*0.8)
	{

		ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM4_CH, 0);
		ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM3_CH, pwm);
	}
	//反转
	else
	{
		ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM4_CH, 0);
		ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM3_CH, pwm/5);
                SPD.MotroPWM =SPD.MotroPWM/5;
	}
}