#include "go.h"

uint8 mid=20;
extern uint8 basemap[YM][XM];
extern uint8 leftmap[YM][XM];
extern uint8 rightmap[YM][XM];
extern uint8 insidemap[YM][XM];
extern IMG_STATUS IS;
extern IMG_FLAGS IF;
extern IMG_FLAGS LAST_IF;
extern uint8 stopFlag;  
extern uint32 runTimeCnt;
extern uint8 imgbuff[CAMERA_SIZE]; 
extern void setTimeBeep_ms(uint8 timeSet, uint8 vt);
uint8 getDown(uint8 dir);

uint8 ramp_flag = 0;
uint16 lineAmend = 15;
uint16 speedLineK = 12;
uint16 speedLineB = 6;
uint8 saveImgFlag = 0;

uint16 rampDelayDown = 100;
uint16 rampDelayUp = 5000;  

uint8 outTimeRampDet = 0;

int Hgyro1[10]={0};
float HRoll[10]={0};
extern int top,top_temp,other_top,other_top_temp;
extern float deviation;
extern uint8 speedlline[YM],speedrline[YM];
extern float Pitch,Roll,Yaw;
extern short gyro[3];
void go()
{
  for(int i=8;i>=0;--i)
  {
      Hgyro1[i+1]=Hgyro1[i];
      HRoll[i+1]=HRoll[i];
  }
  Hgyro1[0]=gyro[1];
  HRoll[0]=Roll;
     statusReset();
  /*******/
      for(int i = 0; i < XM; ++ i)
      {
        if(mid - i >= 0 && !judge(mid - i, 0))
        {
          searchimg(mid - i, 0);
          break;
        }
        if(mid + i < XM && !judge(mid + i, 0))
        {
          searchimg(mid + i, 0);
          break;
        }
      }
  /*******/
  if(IF.ramp)
        IS.Ramp_line=1;
  uint8 step=STEP1;
  if(IF.annulus) 
  {
    uint8 down=getDown(IF.annulus);
    step=30<=down?30:down;
  }
  for(uint8 i = 0; i < step; ++ i)
    {
      if(basemap[i][0] && leftmap[i][0] != 1 && rightmap[i][0] != 1&&IS.num_lm<2)
      {
       searchleftmap(0, i);
       if(IS.num_lm==0) IS.down_lm=i;
       ++IS.num_lm;
      }
      if(basemap[i][XX] && leftmap[i][XX] != 1 && rightmap[i][XX] != 1 &&IS.num_rm<2)
      {
        searchrightmap(XX, i);
        if(IS.num_rm==0) IS.down_rm=i;
        ++IS.num_rm;
      }
    }
 
//  /*******/
    uint8 left = 0;
    uint8 right = XX;
    for(int i = mid; i >= 0; -- i)
    {
      if(basemap[0][i] != 0)
      {
        left = i;               //底部左边界位置
        break;
      }
    }
    for(uint8 i = mid; i < XM; ++ i)
    {
      if(basemap[0][i] != 0)
      {
        right = i;              //底部右边界位置
        break;
      }
    }
    mid = (left + right) / 2;     //下一次mid的位置

    
///***开始处理图像****/
    crossroadforleft();
    crossroadforright();
    deleteline();
    Get_insideMap();
    getTop();
    IF.annulus = goAnnulus();
    if(GET_SWITCH7() == SWITCH_ON)
      IF.bump=goBump();
    if(IF.annulus==0 && IF.obstacle==0)       //处在允许识别时间内
      IF.ramp=go_ramp();
    
    
    top=top_temp;//因为在环岛函数中会对top进行修改，防止电机中断提取未修改的的top，在这里才对top进行赋值，之前只对top_temp进行赋值
    other_top=other_top_temp;
    IF.yhds = czAnnulus();
    if(IF.annulus==0&&IF.ramp==0)
      IF.obstacle = Obstacle();
    if(IF.bump)//ramp()&&isBump()==0||sramp()||RampUp())//&& GET_SWITCH7() == SWITCH_ON//isBump()
    {   
       setTimeBeep_ms(200,HIGH);
    }
 //   setTimeBeep_ms(200,HIGH);
 /***控制****/   
   turn();
}
uint8 getDown(uint8 dir)
{
	if (dir == AR1 || dir == AR2)
	{
		uint8 min = getMapYMin_Col2(XX, 0,basemap);
		if (min>5)
		for (uint8 j = XX; j>XX - 10; --j)
		{
			if (basemap[min - 2][j]) return min - 2;
		}
	}
	else if (dir == AL1 || dir == AL2)
	{
		uint8 min = getMapYMin_Col2(0,0, basemap);
		if (min>5)
		for (uint8 j = 0; j<10; ++j)
		{
			if (basemap[min - 2][j]) return min - 2;
		}
	}
	return YM;
}
