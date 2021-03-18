/* 
 *  A车模，齿数77，编码器齿轮24，后轮半径26mm，
 *  ->则编码器转1圈对应行驶距离50.9mm，共有500个脉冲
 *  当控制周期为10ms时，每100个脉冲对应速度约1.018m/s
 *  龙邱编码器：齿轮25
 *  ->编码器转一圈对应行驶距离53mm，共有512个脉冲
 *  当控制周期为10ms时，每100个脉冲对应速度约1.035m/s
 */

#include "common.h"
#include "m_Structure.h"
#include "m_Switch.h"
#include "m_LED.h"
#include "m_DealImg.h"
#include "m_SpeedControl.h"


/*
 *  外部变量
 */
extern float Slope_avg; 		 //斜率设定速度
extern SPEEDInfo_st SpeedInfo;    		//速度信息结构体
/*
 *	全局变量
 */
RUNMode_e RunMode = CONSTANT_SPEED;	        //运行模式标志

SPEEDParam_st SpeedParam = {0};	        //速度参数结构体
SPEED_TYPE_e SpeedType = NORMAL_SHIFT;  //速度控制类型标志，0表示二次公式循迹，1表示直线加速，2表示入弯减速


// 用于速度控制的参数和标志位

uint16 absde[5] = {0};    //偏差	
int16  dede[3]  = {0};    //偏差的差值

uint8 top_save[2] = {0};  //最远处的赛道边界的y坐标
uint8 top1 = 0;		      //车身延长线与赛道边界的交点的y坐标

uint16 AimSpeed_brake;         //用于减速完成的判断
uint16 BrakeTiming_cnt = 0;    //减速延时计数
uint8  BrakeType = 0;          //用于判断刹车种类，1表示直道刹车，2表示短入弯刹车，3表示长入弯刹车


/******************************************************************************
 * @Function     SpeedParam_Init()
 * @Description  速度参数结构体初始化
 * @Callby       main
 *****************************************************************************/
void SpeedParam_Init()
{	
	SpeedParam.add_speed = 8;					//加速
	
	SpeedParam.extroSpeed = 400; 				//直道最大速度
	SpeedParam.corner_minSpeed = 230; 			//弯道最小速度
	SpeedParam.exitCorner_maxSpeed = 320;		//出弯加速的最大速度
	SpeedParam.rampSpeed = 280;
	
	SpeedParam.speed_k = 85;					//二次公式系数
	SpeedParam.curve_maxSpeed = 280;			//二次公式的最大速度
	
	SpeedParam.zrstop_speed = 20;				//直道减速多减的余量
	SpeedParam.cwrstop_speed = 0;				//长入弯减速多减的余量
	SpeedParam.dwrstop_speed = 0;				//短入弯减速多减的余量

	SpeedParam.zhidao_ade_th = 10; 				//入直道的判断
	SpeedParam.chuwan_ade_th = 30;			    //出弯加速的判断
	SpeedParam.ruwan_ade_th = 15;				//入弯减速的判断

	SpeedParam.top1_th = 64;
}

/*******************************************************************************
 * @Function     SpeedInfo_Init()
 * @Description  速度信息结构体初始化
 * @Callby       main
*******************************************************************************/
void SpeedInfo_Init()
{
	SpeedInfo.var[0] = 0;      //当前编码器测量的数值
        SpeedInfo.var[1] = 0;
        SpeedInfo.var[2] = 0;

	SpeedInfo.nowSpeed = 0;    //当前速度
	SpeedInfo.sumSpeed = 0;    //速度累加和(对应路程)
	SpeedInfo.avgSpeed = 0;    //平均速度
	
	SpeedInfo.aimSpeed = 250;  //运行的目标速度，默认250(2.5m/s)
    
        SpeedInfo.MotroPWM = 0;  //PWM输出
}

/******************************************************************************
 * @Function          Get_AvgDeviation 
 * @Description      统计平均偏差
 * @Input            src: 输入图像，待处理
                     width：标准线数组
                     d_line: 开始统计的行数，主要统计顶部
 * @Output           10000: 全白，
                     else： 平均偏差                  
 * @Call by          Car_Turn of control.c
 * @Create           zhejiang university of technology
 ******************************************************************************/
float Get_AvgDeviation(uint8 src[][COLUMN], uint8 width[], uint8 d_line)
{
    /*
     * 局部变量声明
     */
    float sum = 0;
    float x;
    int n = 0;
  
    /*
     * 主处理
     */
    for (uint8 i = d_line; i < ROW; ++i)
    {
        for (uint8 j = 0; j < COLUMN; ++j)
        {
            if (src[i][j] == 2)
            {
                ++n;
                
                x = (float)((j - (int)width[i]) * (0.05625 * i + 1.25));  //平均偏差
                sum += x;    //求和
            }
        }
    }
  
    if (n == 0)    //全白，无边界
    {
        return 10000;
    }
    else
    {
        return (sum / n);
    }
}

/******************************************************************************
 * @Function     Get_AimSpeed
 * @Description  计算目标速度
 * @Callby       Car_Go of CarControl.c
 *****************************************************************************/
uint16 Get_AimSpeed(void)
{ 
	uint8 rgb_light = 0;  //用于全彩LED亮度控制
    
	uint8 top = ROW - 1;  //最远处的赛道边界对应的y轴坐标
	float ave_de = 0;    //平均偏差(与标准线相比)

	uint16 aimSpeed_t = 0;                   //设定的目标速度
    uint16 nowSpeed_t = SpeedInfo.nowSpeed;  //当前速度
	
	/* 计算用来判赛道类型的参数 */
    float avgDeviation_L, avgDeviation_R;
    
    /* 统计平均偏差(与标准线相比)，预测赛道类型，丢线返回10000 */
    avgDeviation_L = Get_AvgDeviation(leftmap, StandardLine_L, 28);
    avgDeviation_R = Get_AvgDeviation(rightmap, StandardLine_R, 28);
    
    if (avgDeviation_L != 10000 && avgDeviation_R != 10000)        //左，右全有边界
    {
        ave_de = (avgDeviation_L + avgDeviation_R) / 2;
    }
    else if (avgDeviation_L == 10000 && avgDeviation_R != 10000)  //右有图像
    {
        ave_de = avgDeviation_R;
    }
    else if (avgDeviation_R == 10000 && avgDeviation_L != 10000)  //左有图像
    {
        ave_de = avgDeviation_L;
    }                                                               
    
    /* 匀速模式下直接返回 */
	if( RunMode == CONSTANT_SPEED )
	{
		return SpeedInfo.aimSpeed;
	}

    //计算top
    uint8 myflag = 0;      
    
    /*
	 *  从上往下找白色区域，找到最上面第一个白点就退出，记下此行数 
	 */
    for (uint8 i = 0; i < ROW_ADD; ++ i)
    {
        for (uint8 j = 0; j < COLUMN; ++ j)
        {
            if (basemap[ROW_ADD - 1 - i][j] == 2)
            {
                top = ROW_ADD - 1 - i;
                myflag = 1;
                break;
            }
        }
        if (myflag)
        {
            break;
        } 
    }
    
     /* 
      *  计算other_top 
      *  车身延长线从下往上与赛道边界相交的点的y轴坐标值
      */
    for (uint8 i = 20; i < ROW; ++i)
    {
        if (basemap[i][SpeedLine_L[i]] == 2 || basemap[i][SpeedLine_R[i]] == 2) 
        {
            top1 = i;
            break;
        }
        top1 = i;
    }
  
    /*以下用于目标速度计算*/
    top_save[1] = top_save[0];
    top_save[0] = top;
    
    /*保存abs(偏差)历史值*/
    absde[4] = absde[3];
    absde[3] = absde[2];
    absde[2] = absde[1];
    absde[1] = absde[0];    
    absde[0] = abs((int)ave_de);
    
    /*保存偏差的差值历史值*/
    dede[2] = dede[1];     //用于判断入弯还是出弯
    dede[1] = dede[0];     //用于判断入弯还是出弯
    dede[0] = absde[0] - absde[1];

    
    /************************** 进行速度类型判断 ******************************/
	// 加速条件满足
    if( top >= 79 && top1 >= SpeedParam.top1_th
		&& absde[0] <= SpeedParam.zhidao_ade_th && absde[1] <= SpeedParam.zhidao_ade_th && absde[2] <= SpeedParam.zhidao_ade_th && absde[3] <= SpeedParam.zhidao_ade_th && absde[4] <= SpeedParam.zhidao_ade_th
        && SpeedType != BRAKE)  
    {
        SpeedType = FULL_ACCELE;	//将速度类型标志置为加速档
    }
	// 直道入圆环刹车
	else if( circle_in_flag > 2 && SpeedType == FULL_ACCELE && nowSpeed_t > 300 )
	{
		SpeedType = BRAKE; 	  //将速度类型标志置为刹车档
        BrakeType = 1;

		if(DoubleCar_Info.car_ID == LEADER_CAR)
			DoubleCar_Info.flag_tracktype |= BRAKE_MASK;  //刹车通知后车
	}
	// 前车刹车
	else if( (DoubleCar_Info.flag_tracktype & BRAKE_MASK) && DoubleCar_Info.car_ID == FOLLOW_CAR )
	{
		SpeedType = BRAKE; 	  //将速度类型标志置为刹车档
        BrakeType = 1;
	}
	// 长直道刹车条件满足
    else if( (top < 79 || top1 < SpeedParam.top1_th || absde[0] > SpeedParam.zhidao_ade_th)  
		&& SpeedType == FULL_ACCELE && nowSpeed_t > 350 )    
    {
        SpeedType = BRAKE; 	  //将速度类型标志置为刹车档
        BrakeType = 1;

		if(DoubleCar_Info.car_ID == LEADER_CAR)
			DoubleCar_Info.flag_tracktype |= BRAKE_MASK;  //刹车通知后车
    }
	// 出弯加速条件满足
    else if( absde[0] <= SpeedParam.chuwan_ade_th && dede[0] < 0 && dede[1] < 0 && dede[2] < 0
		&& SpeedType == NORMAL_SHIFT && SpeedType != BRAKE 
        && top1 > 50 
        && GET_SWITCH6() == SWITCH_ON )     
    {   
        SpeedType = CURVE_ACCELE;    //将速度类型标志置为出弯加速
    }
    else if(absde[0] >= SpeedParam.ruwan_ade_th && dede[0] > 0 && dede[1] > 0 && dede[2] > 0 
            && SpeedType != BRAKE )
    {   
        SpeedType = NORMAL_SHIFT;
        
        //Flag.beepON = TRUE;
    }
	/************************** 结束速度类型判断 ******************************/


	
    /*************************** 计算刹车速度 *********************************/
    if (BrakeType == 1)    //计算直道刹车的速度
    {
        AimSpeed_brake = SpeedParam.corner_minSpeed - SpeedParam.zrstop_speed;
	}
    else if(BrakeType == 2)
    {
        AimSpeed_brake = (uint16)((SpeedParam.curve_maxSpeed - SpeedParam.zrstop_speed - ((float)SpeedParam.speed_k / 10000) * Slope_avg * Slope_avg));    //设定的速度
            
        /* 设定值限限幅 */
        if (AimSpeed_brake < SpeedParam.corner_minSpeed) 
        {
            AimSpeed_brake = SpeedParam.corner_minSpeed;
        }
    }
    /************************** 结束计算刹车速度 ******************************/

	
  
    /************************** 判断刹车是否完成 ******************************/
    if (nowSpeed_t <= AimSpeed_brake && SpeedType == BRAKE)    //减速完成的延时滤波
    {
        BrakeTiming_cnt++;
        
        //Flag.beepON = TRUE;
    }
	
	//刹车完成后延时一段时间再开始加速
    if (BrakeTiming_cnt >= 10)    
    {
        BrakeTiming_cnt = 0;

		// 加速条件满足
        if( top >= 79 && top1 >= SpeedParam.top1_th 
			&& absde[0] <= SpeedParam.zhidao_ade_th && absde[1] <= SpeedParam.zhidao_ade_th && absde[2] <= SpeedParam.zhidao_ade_th && absde[3] <= SpeedParam.zhidao_ade_th && absde[4] <= SpeedParam.zhidao_ade_th)
        {
            SpeedType = FULL_ACCELE;    //将速度类型标志置为加速档
        }
		// 出弯加速条件满足
        else if( absde[0] <= SpeedParam.chuwan_ade_th && dede[0] < 0 && dede[1] < 0 && dede[2] < 0 
			&& SpeedType != FULL_ACCELE
			&& GET_SWITCH6() == SWITCH_ON )    
        {   
            SpeedType = CURVE_ACCELE;    //将速度类型标志置为出弯加速
        }
		// 未满足加速条件就进入二次公式
        else 
        {
            SpeedType = NORMAL_SHIFT;    
        }    
    }
    /************************** 结束判断刹车是否完成 **************************/


	
    /************************** 根据速度类型实现控速 **************************/
	switch(SpeedType)
	{
		case FULL_ACCELE:
		{
			// 全速运行
       	    aimSpeed_t = (uint16)SpeedParam.extroSpeed;

		    WS_SetColorAll(0, 255, 0);

			break;
		}
		case NORMAL_SHIFT:
		{
			aimSpeed_t = (uint16)(SpeedParam.curve_maxSpeed - ((float)SpeedParam.speed_k / 10000) * Slope_avg * Slope_avg);    //二次公式
	            
	        /*速度限幅*/
	        if (aimSpeed_t < SpeedParam.corner_minSpeed)
	        {
	            aimSpeed_t = SpeedParam.corner_minSpeed;
	        }

			float a = (240 - 30)/(SpeedParam.curve_maxSpeed - SpeedParam.corner_minSpeed);
			float b = 30 - a*SpeedParam.corner_minSpeed;

			rgb_light = (uint8)(a * aimSpeed_t + b);

			WS_SetColorAll(255-rgb_light, rgb_light, 0);

			break;
		}
		case BRAKE:
		{
       	    aimSpeed_t = AimSpeed_brake;    //减速设定值

		    WS_SetColorAll(225, 0, 0);

			break;
		}
		case CURVE_ACCELE:
		{	
	        aimSpeed_t = SpeedInfo.aimSpeed + SpeedParam.add_speed;
	        
	        if (aimSpeed_t > SpeedParam.exitCorner_maxSpeed)
	        {
	            aimSpeed_t = (uint16)SpeedParam.exitCorner_maxSpeed;    //速度限幅
	        }

			WS_SetColorAll(255, 225, 255);

			break;
		}
	}
    /************************* 结束根据速度类型控速 ***************************/
	
    //最后进行限幅，以免意外
    if (aimSpeed_t > SpeedParam.extroSpeed)
    {
        aimSpeed_t = (uint16)SpeedParam.extroSpeed;
    }
	
	return aimSpeed_t;
}


