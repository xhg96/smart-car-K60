#ifndef _M_SPEEDCONTROL_H_
#define _M_SPEEDCONTROL_H_


typedef enum {

	NORMAL_SHIFT,	//ʹ�ö��κ�������
	FULL_ACCELE,	//ȫ����
	
	BRAKE,			//ɲ��

	CURVE_ACCELE,	//�������
	
}SPEED_TYPE_e;
//
//typedef struct {
//
//	int16 var[3];         //��¼�����ұ߱�������ֵ
//
//	int16 nowSpeed;         //���ұ������ٶ�ƽ��ֵ(��Ч��ǰ�ٶ�)
//	int64 sumSpeed;         //��ǰ�ۼ����ٶ�
//	int16 avgSpeed;         //��ǰƽ���ٶ�
//	uint16 aimSpeed;	     //Ŀ���ٶ�
//
//	int16 MotroPWM;        //������PWMֵ 
//	
//}SPEEDInfo_st;
//
//
//
///* С������ģʽö������ */
//typedef enum{
//
//	CONSTANT_SPEED,
//	NORMAL_RUN,
//    TIMING_RUN,
//	
//}RUNMode_e;

typedef struct
{
	uint16 add_speed;               //����
    
	uint16 extroSpeed;              //ֱ������ٶ�
	uint16 corner_minSpeed;         //�����С�ٶ�
	uint16 exitCorner_maxSpeed;     //������ٵ�����ٶ�
	uint16 rampSpeed;				//���µ��ٶ�
	uint16 chase_minSpeed;			//׷����С�ٶ�
	
	uint16 speed_k;             //���ι�ʽϵ��
	uint16 curve_maxSpeed;     //���ι�ʽ������ٶ�

	uint16 zrstop_speed;       //ֱ�����ٶ��������
	uint16 cwrstop_speed;      //��������ٶ��������
	uint16 dwrstop_speed;      //��������ٶ��������

	uint16 zhidao_ade_th;         //��ֱ�����ж�
	uint16 ruwan_ade_th;          //������ٵ��ж�
	uint16 chuwan_ade_th;         //������ٵ��ж�

	uint16 top1_th;				//ֱ���ж�top1��ֵ
	
}SPEEDParam_st;


extern uint16 absde[];
extern uint8 top_save[];
extern uint8 top1;

/*
 *  ���ⲿ���õĺ����ӿ�����
 */
extern void SpeedParam_Init();
extern uint16 Get_AimSpeed(void);

//
//extern SPEEDParam_st SpeedParam;	          //�ٶȲ����ṹ��
//extern SPEED_TYPE_e SpeedType;
//extern RUNMode_e RunMode;



#endif
