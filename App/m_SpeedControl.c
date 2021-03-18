/* 
 *  A��ģ������77������������24�����ְ뾶26mm��
 *  ->�������ת1Ȧ��Ӧ��ʻ����50.9mm������500������
 *  ����������Ϊ10msʱ��ÿ100�������Ӧ�ٶ�Լ1.018m/s
 *  ���������������25
 *  ->������תһȦ��Ӧ��ʻ����53mm������512������
 *  ����������Ϊ10msʱ��ÿ100�������Ӧ�ٶ�Լ1.035m/s
 */

#include "common.h"
#include "m_Structure.h"
#include "m_Switch.h"
#include "m_LED.h"
#include "m_DealImg.h"
#include "m_SpeedControl.h"


/*
 *  �ⲿ����
 */
extern float Slope_avg; 		 //б���趨�ٶ�
extern SPEEDInfo_st SpeedInfo;    		//�ٶ���Ϣ�ṹ��
/*
 *	ȫ�ֱ���
 */
RUNMode_e RunMode = CONSTANT_SPEED;	        //����ģʽ��־

SPEEDParam_st SpeedParam = {0};	        //�ٶȲ����ṹ��
SPEED_TYPE_e SpeedType = NORMAL_SHIFT;  //�ٶȿ������ͱ�־��0��ʾ���ι�ʽѭ����1��ʾֱ�߼��٣�2��ʾ�������


// �����ٶȿ��ƵĲ����ͱ�־λ

uint16 absde[5] = {0};    //ƫ��	
int16  dede[3]  = {0};    //ƫ��Ĳ�ֵ

uint8 top_save[2] = {0};  //��Զ���������߽��y����
uint8 top1 = 0;		      //�����ӳ����������߽�Ľ����y����

uint16 AimSpeed_brake;         //���ڼ�����ɵ��ж�
uint16 BrakeTiming_cnt = 0;    //������ʱ����
uint8  BrakeType = 0;          //�����ж�ɲ�����࣬1��ʾֱ��ɲ����2��ʾ������ɲ����3��ʾ������ɲ��


/******************************************************************************
 * @Function     SpeedParam_Init()
 * @Description  �ٶȲ����ṹ���ʼ��
 * @Callby       main
 *****************************************************************************/
void SpeedParam_Init()
{	
	SpeedParam.add_speed = 8;					//����
	
	SpeedParam.extroSpeed = 400; 				//ֱ������ٶ�
	SpeedParam.corner_minSpeed = 230; 			//�����С�ٶ�
	SpeedParam.exitCorner_maxSpeed = 320;		//������ٵ�����ٶ�
	SpeedParam.rampSpeed = 280;
	
	SpeedParam.speed_k = 85;					//���ι�ʽϵ��
	SpeedParam.curve_maxSpeed = 280;			//���ι�ʽ������ٶ�
	
	SpeedParam.zrstop_speed = 20;				//ֱ�����ٶ��������
	SpeedParam.cwrstop_speed = 0;				//��������ٶ��������
	SpeedParam.dwrstop_speed = 0;				//��������ٶ��������

	SpeedParam.zhidao_ade_th = 10; 				//��ֱ�����ж�
	SpeedParam.chuwan_ade_th = 30;			    //������ٵ��ж�
	SpeedParam.ruwan_ade_th = 15;				//������ٵ��ж�

	SpeedParam.top1_th = 64;
}

/*******************************************************************************
 * @Function     SpeedInfo_Init()
 * @Description  �ٶ���Ϣ�ṹ���ʼ��
 * @Callby       main
*******************************************************************************/
void SpeedInfo_Init()
{
	SpeedInfo.var[0] = 0;      //��ǰ��������������ֵ
        SpeedInfo.var[1] = 0;
        SpeedInfo.var[2] = 0;

	SpeedInfo.nowSpeed = 0;    //��ǰ�ٶ�
	SpeedInfo.sumSpeed = 0;    //�ٶ��ۼӺ�(��Ӧ·��)
	SpeedInfo.avgSpeed = 0;    //ƽ���ٶ�
	
	SpeedInfo.aimSpeed = 250;  //���е�Ŀ���ٶȣ�Ĭ��250(2.5m/s)
    
        SpeedInfo.MotroPWM = 0;  //PWM���
}

/******************************************************************************
 * @Function          Get_AvgDeviation 
 * @Description      ͳ��ƽ��ƫ��
 * @Input            src: ����ͼ�񣬴�����
                     width����׼������
                     d_line: ��ʼͳ�Ƶ���������Ҫͳ�ƶ���
 * @Output           10000: ȫ�ף�
                     else�� ƽ��ƫ��                  
 * @Call by          Car_Turn of control.c
 * @Create           zhejiang university of technology
 ******************************************************************************/
float Get_AvgDeviation(uint8 src[][COLUMN], uint8 width[], uint8 d_line)
{
    /*
     * �ֲ���������
     */
    float sum = 0;
    float x;
    int n = 0;
  
    /*
     * ������
     */
    for (uint8 i = d_line; i < ROW; ++i)
    {
        for (uint8 j = 0; j < COLUMN; ++j)
        {
            if (src[i][j] == 2)
            {
                ++n;
                
                x = (float)((j - (int)width[i]) * (0.05625 * i + 1.25));  //ƽ��ƫ��
                sum += x;    //���
            }
        }
    }
  
    if (n == 0)    //ȫ�ף��ޱ߽�
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
 * @Description  ����Ŀ���ٶ�
 * @Callby       Car_Go of CarControl.c
 *****************************************************************************/
uint16 Get_AimSpeed(void)
{ 
	uint8 rgb_light = 0;  //����ȫ��LED���ȿ���
    
	uint8 top = ROW - 1;  //��Զ���������߽��Ӧ��y������
	float ave_de = 0;    //ƽ��ƫ��(���׼�����)

	uint16 aimSpeed_t = 0;                   //�趨��Ŀ���ٶ�
    uint16 nowSpeed_t = SpeedInfo.nowSpeed;  //��ǰ�ٶ�
	
	/* �����������������͵Ĳ��� */
    float avgDeviation_L, avgDeviation_R;
    
    /* ͳ��ƽ��ƫ��(���׼�����)��Ԥ���������ͣ����߷���10000 */
    avgDeviation_L = Get_AvgDeviation(leftmap, StandardLine_L, 28);
    avgDeviation_R = Get_AvgDeviation(rightmap, StandardLine_R, 28);
    
    if (avgDeviation_L != 10000 && avgDeviation_R != 10000)        //����ȫ�б߽�
    {
        ave_de = (avgDeviation_L + avgDeviation_R) / 2;
    }
    else if (avgDeviation_L == 10000 && avgDeviation_R != 10000)  //����ͼ��
    {
        ave_de = avgDeviation_R;
    }
    else if (avgDeviation_R == 10000 && avgDeviation_L != 10000)  //����ͼ��
    {
        ave_de = avgDeviation_L;
    }                                                               
    
    /* ����ģʽ��ֱ�ӷ��� */
	if( RunMode == CONSTANT_SPEED )
	{
		return SpeedInfo.aimSpeed;
	}

    //����top
    uint8 myflag = 0;      
    
    /*
	 *  ���������Ұ�ɫ�����ҵ��������һ���׵���˳������´����� 
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
      *  ����other_top 
      *  �����ӳ��ߴ��������������߽��ཻ�ĵ��y������ֵ
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
  
    /*��������Ŀ���ٶȼ���*/
    top_save[1] = top_save[0];
    top_save[0] = top;
    
    /*����abs(ƫ��)��ʷֵ*/
    absde[4] = absde[3];
    absde[3] = absde[2];
    absde[2] = absde[1];
    absde[1] = absde[0];    
    absde[0] = abs((int)ave_de);
    
    /*����ƫ��Ĳ�ֵ��ʷֵ*/
    dede[2] = dede[1];     //�����ж����仹�ǳ���
    dede[1] = dede[0];     //�����ж����仹�ǳ���
    dede[0] = absde[0] - absde[1];

    
    /************************** �����ٶ������ж� ******************************/
	// ������������
    if( top >= 79 && top1 >= SpeedParam.top1_th
		&& absde[0] <= SpeedParam.zhidao_ade_th && absde[1] <= SpeedParam.zhidao_ade_th && absde[2] <= SpeedParam.zhidao_ade_th && absde[3] <= SpeedParam.zhidao_ade_th && absde[4] <= SpeedParam.zhidao_ade_th
        && SpeedType != BRAKE)  
    {
        SpeedType = FULL_ACCELE;	//���ٶ����ͱ�־��Ϊ���ٵ�
    }
	// ֱ����Բ��ɲ��
	else if( circle_in_flag > 2 && SpeedType == FULL_ACCELE && nowSpeed_t > 300 )
	{
		SpeedType = BRAKE; 	  //���ٶ����ͱ�־��Ϊɲ����
        BrakeType = 1;

		if(DoubleCar_Info.car_ID == LEADER_CAR)
			DoubleCar_Info.flag_tracktype |= BRAKE_MASK;  //ɲ��֪ͨ��
	}
	// ǰ��ɲ��
	else if( (DoubleCar_Info.flag_tracktype & BRAKE_MASK) && DoubleCar_Info.car_ID == FOLLOW_CAR )
	{
		SpeedType = BRAKE; 	  //���ٶ����ͱ�־��Ϊɲ����
        BrakeType = 1;
	}
	// ��ֱ��ɲ����������
    else if( (top < 79 || top1 < SpeedParam.top1_th || absde[0] > SpeedParam.zhidao_ade_th)  
		&& SpeedType == FULL_ACCELE && nowSpeed_t > 350 )    
    {
        SpeedType = BRAKE; 	  //���ٶ����ͱ�־��Ϊɲ����
        BrakeType = 1;

		if(DoubleCar_Info.car_ID == LEADER_CAR)
			DoubleCar_Info.flag_tracktype |= BRAKE_MASK;  //ɲ��֪ͨ��
    }
	// ���������������
    else if( absde[0] <= SpeedParam.chuwan_ade_th && dede[0] < 0 && dede[1] < 0 && dede[2] < 0
		&& SpeedType == NORMAL_SHIFT && SpeedType != BRAKE 
        && top1 > 50 
        && GET_SWITCH6() == SWITCH_ON )     
    {   
        SpeedType = CURVE_ACCELE;    //���ٶ����ͱ�־��Ϊ�������
    }
    else if(absde[0] >= SpeedParam.ruwan_ade_th && dede[0] > 0 && dede[1] > 0 && dede[2] > 0 
            && SpeedType != BRAKE )
    {   
        SpeedType = NORMAL_SHIFT;
        
        //Flag.beepON = TRUE;
    }
	/************************** �����ٶ������ж� ******************************/


	
    /*************************** ����ɲ���ٶ� *********************************/
    if (BrakeType == 1)    //����ֱ��ɲ�����ٶ�
    {
        AimSpeed_brake = SpeedParam.corner_minSpeed - SpeedParam.zrstop_speed;
	}
    else if(BrakeType == 2)
    {
        AimSpeed_brake = (uint16)((SpeedParam.curve_maxSpeed - SpeedParam.zrstop_speed - ((float)SpeedParam.speed_k / 10000) * Slope_avg * Slope_avg));    //�趨���ٶ�
            
        /* �趨ֵ���޷� */
        if (AimSpeed_brake < SpeedParam.corner_minSpeed) 
        {
            AimSpeed_brake = SpeedParam.corner_minSpeed;
        }
    }
    /************************** ��������ɲ���ٶ� ******************************/

	
  
    /************************** �ж�ɲ���Ƿ���� ******************************/
    if (nowSpeed_t <= AimSpeed_brake && SpeedType == BRAKE)    //������ɵ���ʱ�˲�
    {
        BrakeTiming_cnt++;
        
        //Flag.beepON = TRUE;
    }
	
	//ɲ����ɺ���ʱһ��ʱ���ٿ�ʼ����
    if (BrakeTiming_cnt >= 10)    
    {
        BrakeTiming_cnt = 0;

		// ������������
        if( top >= 79 && top1 >= SpeedParam.top1_th 
			&& absde[0] <= SpeedParam.zhidao_ade_th && absde[1] <= SpeedParam.zhidao_ade_th && absde[2] <= SpeedParam.zhidao_ade_th && absde[3] <= SpeedParam.zhidao_ade_th && absde[4] <= SpeedParam.zhidao_ade_th)
        {
            SpeedType = FULL_ACCELE;    //���ٶ����ͱ�־��Ϊ���ٵ�
        }
		// ���������������
        else if( absde[0] <= SpeedParam.chuwan_ade_th && dede[0] < 0 && dede[1] < 0 && dede[2] < 0 
			&& SpeedType != FULL_ACCELE
			&& GET_SWITCH6() == SWITCH_ON )    
        {   
            SpeedType = CURVE_ACCELE;    //���ٶ����ͱ�־��Ϊ�������
        }
		// δ������������ͽ�����ι�ʽ
        else 
        {
            SpeedType = NORMAL_SHIFT;    
        }    
    }
    /************************** �����ж�ɲ���Ƿ���� **************************/


	
    /************************** �����ٶ�����ʵ�ֿ��� **************************/
	switch(SpeedType)
	{
		case FULL_ACCELE:
		{
			// ȫ������
       	    aimSpeed_t = (uint16)SpeedParam.extroSpeed;

		    WS_SetColorAll(0, 255, 0);

			break;
		}
		case NORMAL_SHIFT:
		{
			aimSpeed_t = (uint16)(SpeedParam.curve_maxSpeed - ((float)SpeedParam.speed_k / 10000) * Slope_avg * Slope_avg);    //���ι�ʽ
	            
	        /*�ٶ��޷�*/
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
       	    aimSpeed_t = AimSpeed_brake;    //�����趨ֵ

		    WS_SetColorAll(225, 0, 0);

			break;
		}
		case CURVE_ACCELE:
		{	
	        aimSpeed_t = SpeedInfo.aimSpeed + SpeedParam.add_speed;
	        
	        if (aimSpeed_t > SpeedParam.exitCorner_maxSpeed)
	        {
	            aimSpeed_t = (uint16)SpeedParam.exitCorner_maxSpeed;    //�ٶ��޷�
	        }

			WS_SetColorAll(255, 225, 255);

			break;
		}
	}
    /************************* ���������ٶ����Ϳ��� ***************************/
	
    //�������޷�����������
    if (aimSpeed_t > SpeedParam.extroSpeed)
    {
        aimSpeed_t = (uint16)SpeedParam.extroSpeed;
    }
	
	return aimSpeed_t;
}


