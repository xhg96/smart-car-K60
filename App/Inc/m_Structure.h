#ifndef __M_STRUCTURE__
#define __M_STRUCTURE__


#define FREE_CAR   0
#define LEADER_CAR 1
#define FOLLOW_CAR 2

// ˫��״̬��־λ
#define LOST_ULTRASOUND_MASK 0x01    //�������źŶ�ʧ
#define READY_OVERTAKE_MASK  0X02    //׼������(˫�����ڳ���״̬��)
#define FINISH_OVERTAKE_MASK 0X04    //�������
#define FOLLOWER_START_MASK  0x08    //��������־(����ʱǰ�󳵴���һ��������Զ�����)
#define LEADER_BREAKDOWN_MASK 0X10   //ǰ����ê֪ͨ��
#define FOLLOW_BREAKDOWN_MASK 0X20   //����ê֪ͨǰ��


// ǰ��·����־λ
#define STRAIGHTWAY_MASK    0X01    //ǰ����ֱ��
#define BRAKE_MASK 	     0x02    //ǰ������ɲ��
#define RAMP_MASK		     0X04    //ǰ���µ�
#define CIRCLE_MASK	     0X08    //ǰ��Բ��
#define LEFTOBSTRACLE_MASK  0X10    //���ϰ�
#define RIGHTOBSTRACLE_MASK 0X20    //���ϰ�
#define STOPLINE_MASK       0X40    //ֹͣ��


typedef struct {

	vuint8 beepON;
	vuint8 stop;    //1����ͣ����2��ʱͣ��
	vuint8 lowVoltage;    //�͵�����־
	vuint8 upLoad_SpeedWave;
	
}Flag_st;  //��װ���ֱ�־�Ľṹ��


typedef struct
{
	uint16 car_ID;          //ǰ��ID
	vuint8 flag1;           //����Э��������һЩ��־λ
	vuint8 flag_tracktype;  //ǰ�������󳵵�����������Ϣ
	
	vuint16 distance_cm;    //˫��֮��ľ���
	uint16  aim_distance;   //˫����Ŀ�����(���ھ���PID����)
	vuint16 follower_IntegralDistance;  //��ʧ�������źź���������ֵľ���(��)
	vuint16 lastUpdata_time;  //��¼�ϴν��յ��������źŵ�ʱ�䣬�Դ����жϱ����Ƿ���յ��������ź�(��Ϊ���������շ����ⲿ�ж��У�ֻ��ͨ�����ַ����ж�)

}DOUBLECAR_INFO_st;  //��װ����˫��Э������Ϣ�Ľṹ��


extern Flag_st Flag;
extern DOUBLECAR_INFO_st DoubleCar_Info;   //˫��������Ϣ


#endif
