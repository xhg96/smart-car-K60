#include "common.h"
#include "MK60_gpio.h"
#include "MK60_ftm.h"
#include "m_Motro.h"
#include "m_Switch.h"
#include "control.h"
#define MBB 0
#define LBB 1
#define PID 2


/* ȫ�ֱ��� */
IncrementalPID_st Motro_PID;          //�������PID�ṹ��
SPEEDInfo_st SPD = {0};    		//�ٶ���Ϣ�ṹ��

int PreREF = 0;
uint16 turnILimitsub = 50;       //�����޻���
uint16 change_ki=4;        //����flash�У��ɰ������� //���ٻ��ֵĻ���ki
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
/* �ڲ��������� */
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
/********************************PID�ṹ���ʼ��******************************
 *      
 *     �������ܣ�PID�ṹ���ڲ�������ʼ��
 *
*******************************************************************************/
void motroPIDs_Init()
{
	/* PID�ṹ���ʼ�� */
    Motro_PID.REF = 0;                          //Ŀ���ٶ�
    Motro_PID.FeedBack = 0;                     //��ǰ�ٶ�
  
    Motro_PID.PreError = 0;
    Motro_PID.PrePreError = 0;
    Motro_PID.FeedBackSave[0] = Motro_PID.FeedBackSave[1] = Motro_PID.FeedBackSave[2] = 0;
    
  
    Motro_PID.Kp = (uint16)(kp);    
    Motro_PID.Ki = (uint16)(ki);
    Motro_PID.Kd = (uint16)(kd);
  
    Motro_PID.PID_Out = 0; 
}

/***********************************���,��������ʼ��*********************************/
void Motro_Init()
{
    motroPIDs_Init();
    
    // IO�ڳ�ʼ��(�������ʹ��)
    gpio_init(MOTOR_EN,GPO,HIGH);

    // ��ʼ�����PWM
//    ftm_pwm_init(MOTOR_FTM, MOTOR_PWM1_CH,MOTOR_HZ,0);      
  //  ftm_pwm_init(MOTOR_FTM, MOTOR_PWM2_CH,MOTOR_HZ,0);  
    ftm_pwm_init(MOTOR_FTM, MOTOR_PWM3_CH,MOTOR_HZ,0);      
    ftm_pwm_init(MOTOR_FTM, MOTOR_PWM4_CH,MOTOR_HZ,0);        //ǰת
    
	// ��ʼ����������������
    ftm_quad_init(QUAD_FTM);
}

/************************************���ֹͣ**********************************/
void StopMotro()
{   

    FTM_CnV_REG(FTMN[MOTOR_FTM], MOTOR_PWM3_CH) = 0;
    FTM_CnV_REG(FTMN[MOTOR_FTM], MOTOR_PWM4_CH) = 0;
}

/*******************************************************************************
 *     �������ܣ���ȡ�Ҳ������ֵ
 *     ����ֵ����������һ�����������ڵ�����ֵ 
*******************************************************************************/
void GetEncoder()
{   
  /*��������г��ֵĳ��ֵĿ�ת����pwm���󲻻�����������޷�*/
	SPD.var[2] = SPD.var[1];
	SPD.var[1] = SPD.var[0];
	SPD.speed = -(int16)((4500.0/4096.0)*ftm_quad_get(QUAD_FTM));
        ftm_quad_clean(QUAD_FTM);         //��FTM
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
void decimalAcc(float *integerPart, float *decimalPart){      //�ۼ�С��
  
  float temp = *integerPart;
  *decimalPart +=( temp - (int)temp );        //С�������ۼ�
  *integerPart = (int)(*integerPart);               //ȡ��
  
  if(*decimalPart>=1){
    *decimalPart -= 1;
    *integerPart += 1;
  }
}       
/*********************************����ʽPID�㷨*********************************
 *      
 *     �������ܣ�����ʽPID���������PWMֵ
 *     �������ṹ�� PIDM ָ�� 
 *     ����ֵ�����PWMֵ
*******************************************************************************/
int16 calc_vPID(IncrementalPID_st *pp)
{
  int16 error;
  float duty = 0.0;
  float once_i;
  float kp_t,ki_index,kd_t;
  static uint8 turnInRem = 0;  //�Ա������޷�ֻ��һ��
  int16 turnInJudge[2] = {0};       
  
  kp_t = (float)( (pp->Kp)/10.0 );   //ȡ��P I Dϵ��
  kd_t = (float)( (pp->Kd)/10.0 );
 // if(xf_delay)kp_t=0.5*kp_t;
  error = pp->REF - pp->FeedBack;
  if(error + pp->PreError>= 0)
    ki_index = (change_ki/10.0) - (change_ki/10.0) / (1 + exp(change_kib - 0.2*abs(error)));    //����ֿ���
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
  
      /*        �����޻��ֲ���         */  
  turnInJudge[0] = Servo_PID.error;     //error
  turnInJudge[1] = Servo_PID.error - Servo_PID.preError;        //d_error
  
  if( (turnInJudge[1]<0 && turnInJudge[0]<0)||(turnInJudge[1]>0 && turnInJudge[0]>0) && absde[0] >= 59 && absde[0] <= 77 && SPD.AC_P > 100 && turnInRem !=2)      //������� ,absde>=62,����>100,�һ�δ��������
    turnInRem = 1;      //��1 ��ʾ��Ҫ������
  
  if(turnInRem == 1){
    turnInRem = 2;     //��������2
    SPD.AC_P -= turnILimitsub;          // ������ԣ��
  }
  
  if( (turnInJudge[1]>0 && turnInJudge[0]<0)||(turnInJudge[1]<0 && turnInJudge[0]>0) )
    turnInRem = 0;  //����ʱ���־λ
  
     /*        �����޻��ֲ���         */  
  
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
 
/***********************************�������************************************
 *      
 *     �������ܣ�PWM���Ƶ��ת�٣����µ������PID�е�Ŀ���ٶ��뵱ǰ�ٶ�
 *     �������趨��Ŀ���ٶȣ�������ֵ��
 *     ����ֵ��void
*******************************************************************************/
void Motro_Shifting()
{   
  static uint16 stallCount = 0;       //��ת����
    Motro_PID.REF = SPD.aimSpeed;
    
    Motro_PID.FeedBack = SPD.nowSpeed;    //��ȡ������ֵ

    Motro_PID.FeedBackSave[2] = Motro_PID.FeedBackSave[1];
    Motro_PID.FeedBackSave[1] = Motro_PID.FeedBackSave[0];
    Motro_PID.FeedBackSave[0] = SPD.nowSpeed;              //��¼ǰ�����ٶ�
    
    SPD.MotroPWM = calc_vPID(&Motro_PID);
    //SPD.MotroPWM +=bra_speed3*abs(Servo_PID.PID_Out)/1100.0; 
    if(SPD.nowSpeed < 80 && SPD.MotroPWM > 900){       //����
      stallCount++;
      if(stallCount > 500) stallCount = 500;
    }
    else{
      stallCount = 0;
    }
    
    if(stallCount >= 50) stallProtectFlag = 1;  //��0.5s��ͣ���
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
	//��ת
	else
	{
		ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM4_CH, 0);
		ftm_pwm_duty(MOTOR_FTM, MOTOR_PWM3_CH, pwm/5);
                SPD.MotroPWM =SPD.MotroPWM/5;
	}
}