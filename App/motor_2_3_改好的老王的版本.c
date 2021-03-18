#include "motor.h"
#include "math.h"
#include "MyFlash.h"
#include "lcd_key.h"
#include "data_transfer.h"
#include "lcd_key.h"
#include "common.h"

extern SPEEDInfo_st SpeedInfo;    		//�ٶ���Ϣ�ṹ��
extern uint8 stopFlag;          //ͣ����־λ

int aim_speed=100;    //Ŀ���ٶ�


//�����ٶȱ���
uint16 alt_speed_w=180;
uint16 alt_speed_z=168;
uint16 alt_speed_m=170;
uint16 obs_speed=130;
uint16 ramp_up_speed=120;
uint16 ramp_down_speed=140;
uint16 ramp_s_up_speed=120;
uint16 ramp_s_down_speed=120;


//ֱ���ٶȱ���
uint16 max_speed=350;  

//����ٶȱ���
uint16 wan_speed=250;        //����flash�У��ɰ������� //���ι�ʽ�ĳ�����
uint16 min_speed=200;        //����flash�У��ɰ������� //�����С�ٶ�

//�����ٶȱ���
uint16 base_speed=360;       //����flash�У��ɰ������� //��������ٶ�
uint16 outm_speed=370;       //����flash�У��ɰ������� //������ٵ�����ٶ�
uint16 add_speed=4;//4          //����flash�У��ɰ������� //����
uint16 dec_add_flag=0;

//ɲ���ٶȱ���
uint16 zd_stop_speed=270;
uint16 dz_stop_speed=270;
uint16 wr_stop_speed=270;

//�����ٶȱ���
uint16 ramp_speed=200;


//�����жϱ���
uint16 zhidao_ade=21;        //����flash�У��ɰ������� //��ֱ�����ж�
uint16 wan_ade=42; //50          //����flash�У��ɰ������� //������ٵ��ж�
uint16 out_wan_ade=20;
uint16 wan_zhong_ade=30;

//ɲ���жϱ���
uint16 dzhi_top=76;          //����flash�У��ɰ������� //ֱ��ɲ�����ж�
uint16 set_othertop=76;      //����flash�У��ɰ������� //��һ���˵�����

uint16 chu_ru=20;
uint16 wanru_ade=19;         //����flash�У��ɰ������� //������ٵ��ж�

uint16 stoptime=4;           //ɲ����ɵ�ʱ��

//ɲ������ж�
float stopaim_speed;            //���ڼ�����ɵ��ж�
int stopaim_cont=0;             //��ɼ��ٵ�����������

//�������ͱ���
char out_wan=0;
int stop_type=0;//�����ж�ɲ������
int nono_flag=0;//���ڷ�ֹ��ֱ��ɲ����󻹻���뵽��ֱ��ɲ��
int speed_type=0;//�ٶȿ������ͱ�־��0��ʾ���ι�ʽѭ����1��ʾֱ�߼��٣�2��ʾ�������


char START_VAR_CLEAR=0;


uint8 tran_type;
unsigned char last_speed_type;

extern uint16 absde[3],dede[3];
extern float ave_de;
extern int other_top;  //�����������������
extern uint8 top,top_save[2];
extern float deviation;
extern float  de_k;
extern uint8 ob_direction;
extern uint8 ramp_flag;


extern var_struct_2 otherVal[FPNUM][SPNUM-1];
extern char control_line;//0 ˫�� 1 ���� 2 ���� 3 �����߶���Ҫ

extern int start_flag;

extern int cnt;

extern uint8 run_type;

extern uint8 obp_flag;

extern uint8 sramp_flag;;



//���ͳ���
//�ܿ��ص�����һ��
/*
 1��   gpio_init(PTE24,GPI,0); //��ʼ�� 1�ܽ� GPI 
 2��   gpio_init(PTE25,GPI,0); //��ʼ�� 2�ܽ� GPI  
 3��   gpio_init(PTE11,GPI,0); //��ʼ�� 3�ܽ� GPI     
 4��   gpio_init(PTE10,GPI,0); //��ʼ�� 4�ܽ� GPI     
 5��   gpio_init(PTE9,GPI,0); //��ʼ�� 5�ܽ� GPI 
 6��   gpio_init(PTE8,GPI,0); //��ʼ�� 6�ܽ� GPI  
 7��   gpio_init(PTE7,GPI,0); //��ʼ�� 7�ܽ� GPI  
 8��   gpio_init(PTE6,GPI,0); //��ʼ�� 8�ܽ� GPI 
*/
void send_speedval()
{
  //if(stopflag==0&&startcar_flag==1)
  //{
   //  Send2int((int)(speed_type*10),(int)(stop_type*10),0xF4);
    // Send2int((int)aim_lspeed,(int)now_speed_l,0xF1);
    // Send2int((int)aim_rspeed,(int)now_speed_r,0xF2);
   //  Send2int((int)(Speed_PWM_L/10),(int)(Speed_PWM_R/10),0xF3);
    // Send2int((int)(absde[0]),(int)(absde[1]),0xF3);
   //  Send_int((int)(ramp_flag*10),0xF7);
   //  Send_int((int)(yhbz*10),0xF8);
    // Send2int(dede[0],dede[1],0xF8);
   //  Send_int((int)(yh_count*10),0xF9);
   //  Send_int((int)(10*ring_flag),0xFA);
 //    Send_int((int)(stop_cnt),0xFA);      
    //  Send2int((int)(top),(int)(other_top),0xF3);
  //}

}


void clear()
{     
      aim_speed=0;

      START_VAR_CLEAR=0;

}


void Car_go()
{
        GetEncoder();      //��ȡ������ֵ
 
        if(START_VAR_CLEAR==0)//������ϳ�������������Ҫ�ı�־����������(��Щ������Ҫ����ǰ�壬������ͣ������)
        {
          ramp_flag=0; 
        }
        START_VAR_CLEAR=1;//��������
        
         Speed_Set();           //���ݱ�־λ�趨Ŀ���ٶ�
         
         if (stopFlag == 0)    //δͣ��
            Motro_Shifting();      //���㲢���PWM������PID�ٶȲ���
         else{                  //ͣ��
           aim_speed = 0;
           Motro_Shifting();      //���㲢���PWM������PID�ٶȲ���
         }
      
}



void count_stop_top()
{
  dzhi_top=(SpeedInfo.nowSpeed-170)*8/(180-170)+70;
  set_othertop=(SpeedInfo.nowSpeed-170)*10/(180-170)+68;
  if(dzhi_top>78) dzhi_top=78;
  else if(dzhi_top<70)  dzhi_top=70;
  if(set_othertop>78) set_othertop= 78;
  else if(set_othertop<68) set_othertop= 68;
}

int Ck=0;
float KK[4];

void Track_type_detect()
{  
    /****************************************************�����ٶ������ж�*************************************************************/
 

 if(top>78&&other_top>78&&absde[0]<=zhidao_ade&&absde[1]<=zhidao_ade&&absde[2]<=zhidao_ade&&speed_type!=2)//&&absde[2]<=zhidao_ade
  {
   // speed_type=1;//���ٶ����ͱ�־��Ϊ���ٵ�
    tran_type++;
  } 
  else if((top<=78||other_top<=78||absde[0]>(zhidao_ade)) && speed_type==1&&(SpeedInfo.nowSpeed > 195))//��ֱ��ɲ����������
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=1;
    nono_flag=1;
    tran_type=0;
  } 
  else if((top<=dzhi_top||other_top<=set_othertop||absde[0]>(zhidao_ade))&&speed_type==1&&(SpeedInfo.nowSpeed>alt_speed_z))//��ֱ��ɲ����������
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=1;
    nono_flag=1;
    tran_type=0;
  } 
  else if((absde[0]>(chu_ru)||other_top<=71||top<=75)&&(speed_type==1)&&(SpeedInfo.nowSpeed>alt_speed_w))//��ֱ��֮�����䣬ƫ�����ͨ�����һЩ�ſ�ʼɲ�������ü�
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=1;
    nono_flag=1;
    tran_type=0;
  }
  else if((absde[0]>(chu_ru)||other_top<=65||top<=71)&&(speed_type==1))
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=1;
    nono_flag=1;
    tran_type=0; 
  }
 
 
  else if(absde[0]<=(wan_ade)&&dede[0]<0&&dede[1]<0&&(speed_type==0||speed_type==7))//���������������//&&(speed_type==0||speed_type==7)
  {               //ǰһ���������������
    speed_type=3;//���ٶ����ͱ�־��Ϊ�������
    tran_type=0;
  }
  else if((other_top<=68||top<=72)&&(SpeedInfo.nowSpeed>alt_speed_m)&&speed_type==3)
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=2;
    tran_type=0; 
  }
  else if((((absde[0]>=chu_ru)&&(dede[0]>0)&&(dede[1]>0))||((other_top<=63||top<=70)&&(SpeedInfo.nowSpeed>alt_speed_w)))&&speed_type==3)//&&dede[1]>&&dede[1]>00&&dede[1]>0&&&dede[1]>0&gpio_get(PTE7))//���������������&&dede[1]>0
  {                //25 ����������  ǰһ�������ǳ������
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=2;
    tran_type=0;
  }                                //

  else if(other_top>68&&top>72&&absde[0]<=(chu_ru)&&absde[1]<=(chu_ru)&&absde[2]<=(chu_ru)&&speed_type!=1&&speed_type!=2&&nono_flag==0)//����������ֱ��//&&dede[0]<0&&dede[1]<0
  {
    speed_type=6;//���ٶ����ͱ�־��Ϊ����������ֱ��
  }                     //
  else if((absde[0]>(chu_ru)||other_top<=68||top<=72)&&(SpeedInfo.nowSpeed>alt_speed_m)&&(speed_type==6))// &&dede[0]>0 //��ֱ��֮�����䣬ƫ�����ͨ�����һЩ�ſ�ʼɲ�������ü�
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=3;
    tran_type=0;
  }
  else if((absde[0]>(chu_ru)||(other_top<=63||top<=70))&&(speed_type==6))
  {
    speed_type=2;//���ٶ����ͱ�־��Ϊɲ����
    stop_type=3;
    tran_type=0;
  }
      
 
  if(tran_type>=3) speed_type=1;
 
  if((ramp_flag>0&&ramp_flag<4)) speed_type=6;
  if(obp_flag) 
  {
    speed_type=1;
   // stop_type=4;
  }
  if(last_speed_type==2&&speed_type==2&&stop_type!=4)
  {
    if(SpeedInfo.nowSpeed<=stopaim_speed)
    {
      stopaim_cont++;
    }

    if(stopaim_cont>=stoptime)//�ﵽɲ����ɱ�־
    {
      stopaim_cont=0;
  
      if(absde[0]>48)//30
      {
        nono_flag=0;
        speed_type=0;//δ������������ͽ�����ι�ʽ
      }
      else 
      {
        speed_type=7;
        dec_add_flag=1;
      }
    }
  }
  else if(last_speed_type==2&&speed_type==2&&stop_type==4)
  {
    if(SpeedInfo.nowSpeed<=stopaim_speed+2)
    {
      stopaim_cont++;
    }

    if(stopaim_cont>=12)//�ﵽɲ����ɱ�־
    {
      stopaim_cont=0;
    //  Tack_Second_detect();
      if(absde[0]>48)//30
      {
        nono_flag=0;
        speed_type=0;//δ������������ͽ�����ι�ʽ
      }
      else 
      {
        speed_type=7;
        dec_add_flag=1;
      }
    }  
  }
  else 
  {
    stopaim_cont=0;
  }
  
  /*********************�������**********************/
  if(absde[0]>48&&speed_type==7)
  {
    speed_type=0;
  } 
  
 
  if(speed_type!=7)
  {
    dec_add_flag=0;
  }

  if(speed_type==2&&stop_type==1&&absde[0]>54)
    speed_type=0;
  
  last_speed_type=speed_type;
  
}

void Speed_Set()
{
/***************************����ֱ��ɲ�����ٶ�****************************/ 
  if(stop_type==1)
  {
    if(Switch_Get(7))
    {
      stopaim_speed=wan_speed-(wan_speed-min_speed)/(1.3*1.3)*de_k*de_k;
      if(stopaim_speed<min_speed) stopaim_speed=min_speed;
    }
    else
      stopaim_speed=min_speed;
  }
  else if(stop_type==3)
  {
    if(Switch_Get(6))
    {
      stopaim_speed=wan_speed-(wan_speed-min_speed)/(1.3*1.3)*de_k*de_k;
      if(stopaim_speed<min_speed) stopaim_speed=min_speed;
    }
    else
      stopaim_speed=min_speed;
      //stopaim_speed=dz_stop_speed;
  }
  else if(stop_type==2)
  {
    if(Switch_Get(5))
    {
      stopaim_speed=wan_speed-(wan_speed-min_speed)/(1.3*1.3)*de_k*de_k;
      if(stopaim_speed<min_speed) stopaim_speed=min_speed;
    }
    else
      stopaim_speed=min_speed;
      //stopaim_speed=dz_stop_speed;
  }
  else if(stop_type==4)
  {
    stopaim_speed=obs_speed;
  }
 /***************************�����ٶ�����ʵ�ֿ���****************************/ 
  if(speed_type==1)//ֱ������
  {
    aim_speed=max_speed;
  }
  else if(speed_type==0||speed_type==7)//���ι�ʽ����
  {
    aim_speed=wan_speed-(wan_speed-min_speed)/(1.3*1.3)*de_k*de_k;
    if(aim_speed<=min_speed) aim_speed=min_speed;
  }
  else if(speed_type==2)//����
  { 
    aim_speed=stopaim_speed;  //�õ��˼�����˵�ɲ���ٶ�
  }
  else if(speed_type==3)//�������
  {
    aim_speed=aim_speed+add_speed;

    if(aim_speed>outm_speed)aim_speed=outm_speed;//�޷�
    if(aim_speed<=min_speed)aim_speed=min_speed;
  }
  else if(speed_type==6)//����������ֱ��
  {
    aim_speed=base_speed;
    if(aim_speed>base_speed)aim_speed=base_speed;//�޷�
    if(aim_speed<=min_speed)aim_speed=min_speed;
  }
  
  
  /*******�µ�*******/
  if(ramp_flag>0&&ramp_flag<4)
  {
    if(ramp_flag==1||ramp_flag==2)
    {
      if(sramp_flag)
        aim_speed=ramp_s_up_speed;
      else
        aim_speed=ramp_up_speed;
    }
    else if(ramp_flag==3)
    {
      if(sramp_flag)
        aim_speed=ramp_s_down_speed;
      else
        aim_speed=ramp_down_speed;
    }
  }

  /*****�ϰ���********/
  else if(obp_flag)
  {
    aim_speed=obs_speed;
  }
 
  
}

