#ifndef __DEAL_IMG_H__
#define __DEAL_IMG_H__


/*
 * ͼ������궨��
 */
#define ROW     70      //CAMERA_H/VERTICAL
#define COLUMN  40      //CAMERA_W/8

#define ROW_ADD 80      //ΪѰ��top���ӵ�����

#define XX      69
#define YY      39

#define STEP1     65      //55
#define THRESHOLD 0xf0    //0xf0���Ժ��ӱ����������κα���ɫ�ܣ�0xff��ֻ���ڰ�ɫ������fa


/*
 *  ���ⲿ���õĺ����ӿ�����
 */
extern void  StandardMap(void);       //�õ���׼����
extern void  CleanMap(void);      //��������ĳ�ʼ��
extern uint8 OverSamping1(uint8 x, uint8 y);         //ԭͼ��Ķ�ֵ��
extern void OverSamping();
extern void DeleteMyself(); 

__RAMFUNC void  SearchMap_1st(uint8 x, uint8 y, uint8 src[][COLUMN]);        //����ԭͼ������õ���������
__RAMFUNC void  SearchMap_2nd(uint8 x, uint8 y, uint8 dst[][COLUMN], uint8 src[][COLUMN]);      //������������õ����Ҳ�������

extern void  DeleteLeftLine(void);    //ɾ����ػ�
extern void  DeleteRightLine(void);   //ɾ���һػ�

extern void DeleteBottomLeft();
extern void DeleteBottomRight();

/* ʮ�� */
extern uint8 CrossroadForRight(void);
extern uint8 CrossroadForLeft(void);
extern void Crossroad_Handle(void);           //ʮ�ֺͻ�������ܴ���

/*��������*/
extern uint8 Ramp_judge(void);
extern uint8 Starting_Line_judge(void);
extern uint8 LostEdge_judge(void);
extern uint8 Obstacle_judge(void);
extern uint8 Obstacle_judge_left();
extern uint8 Obstacle_judge_right();

extern uint8 StraightWay_judge(void);
extern uint8 CrossCar_Judge(void);
extern uint8 Influence_AftrCar(void);
extern void Influence_AftrCar_handle(int16 direction);



/*
 * �ⲿ���õ�ȫ�ֱ�������
 */
extern float left_k, right_k;
extern int8 circle_in_flag, circle_in_flag2;

extern uint8 CompressedImg[][COLUMN];
extern uint8 basemap[][COLUMN];	       //���ε����õ�������
extern uint8 leftmap[][COLUMN];	       //������ٴε����õ�������
extern uint8 rightmap[][COLUMN];       //���ұ��ٴε����õ�������

extern uint8 StandardLine_L[];
extern uint8 StandardLine_R[];

extern uint8 SpeedLine_L[];
extern uint8 SpeedLine_R[];

extern uint8 CrossAndCircle_type;

extern uint8 CircleChoose_flag;
extern uint8 StartingLine_flag,StartingLine_flag_last;
extern uint8 Obstacle_flag,Obstacle_flag_last;
extern uint8 StraightWay_flag;


extern int8 circle_in_flag, circle_in_flag2;
extern int8 circle_in_flag_last,circle_in_flag2_last;

#endif  /*end of deal_img.h*/

