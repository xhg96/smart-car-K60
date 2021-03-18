#include "motor.h"
#include "math.h"
#include "MyFlash.h"
#include "lcd_key.h"
#include "data_transfer.h"
#include "lcd_key.h"
#include "common.h"

extern SPEEDInfo_st SpeedInfo;    		//速度信息结构体
extern uint8 stopFlag;          //停车标志位

int aim_speed=100;    //目标速度


//其他速度变量
uint16 alt_speed_w=180;
uint16 alt_speed_z=168;
uint16 alt_speed_m=170;
uint16 obs_speed=130;
uint16 ramp_up_speed=120;
uint16 ramp_down_speed=140;
uint16 ramp_s_up_speed=120;
uint16 ramp_s_down_speed=120;


//直道速度变量
uint16 max_speed=350;  

//弯道速度变量
uint16 wan_speed=250;        //存在flash中，由按键调节 //二次公式的常数项
uint16 min_speed=200;        //存在flash中，由按键调节 //弯道最小速度

//加速速度变量
uint16 base_speed=360;       //存在flash中，由按键调节 //出弯基础速度
uint16 outm_speed=370;       //存在flash中，由按键调节 //出弯加速的最大速度
uint16 add_speed=4;//4          //存在flash中，由按键调节 //加速
uint16 dec_add_flag=0;

//刹车速度变量
uint16 zd_stop_speed=270;
uint16 dz_stop_speed=270;
uint16 wr_stop_speed=270;

//其他速度变量
uint16 ramp_speed=200;


//加速判断变量
uint16 zhidao_ade=21;        //存在flash中，由按键调节 //入直道的判断
uint16 wan_ade=42; //50          //存在flash中，由按键调节 //出弯加速的判断
uint16 out_wan_ade=20;
uint16 wan_zhong_ade=30;

//刹车判断变量
uint16 dzhi_top=76;          //存在flash中，由按键调节 //直道刹车的判断
uint16 set_othertop=76;      //存在flash中，由按键调节 //另一顶端的设置

uint16 chu_ru=20;
uint16 wanru_ade=19;         //存在flash中，由按键调节 //弯道加速的判断

uint16 stoptime=4;           //刹车完成的时间

//刹车完成判断
float stopaim_speed;            //用于减速完成的判断
int stopaim_cont=0;             //完成减速的周期数计数

//赛道类型变量
char out_wan=0;
int stop_type=0;//用于判断刹车种类
int nono_flag=0;//用于防止长直到刹车完后还会进入到短直道刹车
int speed_type=0;//速度控制类型标志，0表示二次公式循迹，1表示直线加速，2表示入弯减速


char START_VAR_CLEAR=0;


uint8 tran_type;
unsigned char last_speed_type;

extern uint16 absde[3],dede[3];
extern float ave_de;
extern int other_top;  //入弯的其他辅助条件
extern uint8 top,top_save[2];
extern float deviation;
extern float  de_k;
extern uint8 ob_direction;
extern uint8 ramp_flag;


extern var_struct_2 otherVal[FPNUM][SPNUM-1];
extern char control_line;//0 双线 1 左线 2 右线 3 两条线都不要

extern int start_flag;

extern int cnt;

extern uint8 run_type;

extern uint8 obp_flag;

extern uint8 sramp_flag;;



//发送程序
//总开关倒数第一个
/*
 1、   gpio_init(PTE24,GPI,0); //初始化 1管脚 GPI 
 2、   gpio_init(PTE25,GPI,0); //初始化 2管脚 GPI  
 3、   gpio_init(PTE11,GPI,0); //初始化 3管脚 GPI     
 4、   gpio_init(PTE10,GPI,0); //初始化 4管脚 GPI     
 5、   gpio_init(PTE9,GPI,0); //初始化 5管脚 GPI 
 6、   gpio_init(PTE8,GPI,0); //初始化 6管脚 GPI  
 7、   gpio_init(PTE7,GPI,0); //初始化 7管脚 GPI  
 8、   gpio_init(PTE6,GPI,0); //初始化 8管脚 GPI 
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
        GetEncoder();      //获取编码器值
 
        if(START_VAR_CLEAR==0)//如果马上车将开动，将必要的标志及计数清零(有些变量需要开车前清，而不是停车就清)
        {
          ramp_flag=0; 
        }
        START_VAR_CLEAR=1;//将车开动
        
         Speed_Set();           //根据标志位设定目标速度
         
         if (stopFlag == 0)    //未停车
            Motro_Shifting();      //计算并输出PWM，更新PID速度参数
         else{                  //停车
           aim_speed = 0;
           Motro_Shifting();      //计算并输出PWM，更新PID速度参数
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
    /****************************************************进行速度类型判断*************************************************************/
 

 if(top>78&&other_top>78&&absde[0]<=zhidao_ade&&absde[1]<=zhidao_ade&&absde[2]<=zhidao_ade&&speed_type!=2)//&&absde[2]<=zhidao_ade
  {
   // speed_type=1;//将速度类型标志置为加速档
    tran_type++;
  } 
  else if((top<=78||other_top<=78||absde[0]>(zhidao_ade)) && speed_type==1&&(SpeedInfo.nowSpeed > 195))//长直到刹车条件满足
  {
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=1;
    nono_flag=1;
    tran_type=0;
  } 
  else if((top<=dzhi_top||other_top<=set_othertop||absde[0]>(zhidao_ade))&&speed_type==1&&(SpeedInfo.nowSpeed>alt_speed_z))//长直到刹车条件满足
  {
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=1;
    nono_flag=1;
    tran_type=0;
  } 
  else if((absde[0]>(chu_ru)||other_top<=71||top<=75)&&(speed_type==1)&&(SpeedInfo.nowSpeed>alt_speed_w))//短直道之后入弯，偏差比普通入弯大一些才开始刹车，不用急
  {
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=1;
    nono_flag=1;
    tran_type=0;
  }
  else if((absde[0]>(chu_ru)||other_top<=65||top<=71)&&(speed_type==1))
  {
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=1;
    nono_flag=1;
    tran_type=0; 
  }
 
 
  else if(absde[0]<=(wan_ade)&&dede[0]<0&&dede[1]<0&&(speed_type==0||speed_type==7))//出弯加速条件满足//&&(speed_type==0||speed_type==7)
  {               //前一个条件是在弯道中
    speed_type=3;//将速度类型标志置为出弯加速
    tran_type=0;
  }
  else if((other_top<=68||top<=72)&&(SpeedInfo.nowSpeed>alt_speed_m)&&speed_type==3)
  {
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=2;
    tran_type=0; 
  }
  else if((((absde[0]>=chu_ru)&&(dede[0]>0)&&(dede[1]>0))||((other_top<=63||top<=70)&&(SpeedInfo.nowSpeed>alt_speed_w)))&&speed_type==3)//&&dede[1]>&&dede[1]>00&&dede[1]>0&&&dede[1]>0&gpio_get(PTE7))//入弯减速条件满足&&dede[1]>0
  {                //25 出弯又入弯  前一个条件是出弯加速
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=2;
    tran_type=0;
  }                                //

  else if(other_top>68&&top>72&&absde[0]<=(chu_ru)&&absde[1]<=(chu_ru)&&absde[2]<=(chu_ru)&&speed_type!=1&&speed_type!=2&&nono_flag==0)//出弯遇到短直道//&&dede[0]<0&&dede[1]<0
  {
    speed_type=6;//将速度类型标志置为出弯遇到短直道
  }                     //
  else if((absde[0]>(chu_ru)||other_top<=68||top<=72)&&(SpeedInfo.nowSpeed>alt_speed_m)&&(speed_type==6))// &&dede[0]>0 //短直道之后入弯，偏差比普通入弯大一些才开始刹车，不用急
  {
    speed_type=2;//将速度类型标志置为刹车档
    stop_type=3;
    tran_type=0;
  }
  else if((absde[0]>(chu_ru)||(other_top<=63||top<=70))&&(speed_type==6))
  {
    speed_type=2;//将速度类型标志置为刹车档
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

    if(stopaim_cont>=stoptime)//达到刹车完成标志
    {
      stopaim_cont=0;
  
      if(absde[0]>48)//30
      {
        nono_flag=0;
        speed_type=0;//未满足加速条件就进入二次公式
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

    if(stopaim_cont>=12)//达到刹车完成标志
    {
      stopaim_cont=0;
    //  Tack_Second_detect();
      if(absde[0]>48)//30
      {
        nono_flag=0;
        speed_type=0;//未满足加速条件就进入二次公式
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
  
  /*********************入弯控速**********************/
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
/***************************计算直道刹车的速度****************************/ 
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
 /***************************根据速度类型实现控速****************************/ 
  if(speed_type==1)//直道加速
  {
    aim_speed=max_speed;
  }
  else if(speed_type==0||speed_type==7)//二次公式控制
  {
    aim_speed=wan_speed-(wan_speed-min_speed)/(1.3*1.3)*de_k*de_k;
    if(aim_speed<=min_speed) aim_speed=min_speed;
  }
  else if(speed_type==2)//减速
  { 
    aim_speed=stopaim_speed;  //用到了计算好了的刹车速度
  }
  else if(speed_type==3)//出弯加速
  {
    aim_speed=aim_speed+add_speed;

    if(aim_speed>outm_speed)aim_speed=outm_speed;//限幅
    if(aim_speed<=min_speed)aim_speed=min_speed;
  }
  else if(speed_type==6)//出弯遇到短直到
  {
    aim_speed=base_speed;
    if(aim_speed>base_speed)aim_speed=base_speed;//限幅
    if(aim_speed<=min_speed)aim_speed=min_speed;
  }
  
  
  /*******坡道*******/
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

  /*****障碍物********/
  else if(obp_flag)
  {
    aim_speed=obs_speed;
  }
 
  
}

