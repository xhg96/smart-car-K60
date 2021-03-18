#ifndef _DEAL_IMG_H
#define _DEAL_IMG_H
#include "include.h"

#define XM 40            //CAMERA_W/8
#define YM 80            //CAMERA_H/VERTICAL  ԭ����X Y�궨����windows api�е�ö�ٱ����ض���
#define XX 39
#define YY 79
#define VERTICAL 3
#define STEP1   48//53    //55
#define THRESHOLD 0xff//0xf0���Ժ��ӱ����������κα���ɫ�ܣ�0xff��ֻ���ڰ�ɫ������fa
#define STOP_LINE 70
#define goLeft 1
#define goRight 2
#define __Left 1
#define __Right 2
#define __Both 3
#define NO 0
#define AL1 1
#define AL2 2
#define AL3 3
#define AL4 4
#define AL5 5

#define AR1 7
#define AR2 8
#define AR3 9
#define AR4 10
#define AR5 11
#define Fre 1
typedef struct{
	uint8 num_lm;//��ͼ�������
	uint8 num_rm;//num of rightmap
	uint8 down_lm;
	uint8 down_rm;
	uint8 top_lm[2];//��һ������͵ڶ���������ߴ�
	uint8 top_rm[2];//end of rightmap
	uint8 end_lm[2];//��һ������͵ڶ����������������Զ��
	uint8 end_rm[2];
	uint8 left_x;
	uint8 left_y;//ʮ�ּ�⵽�Ľǵ�
	uint8 right_x;
	uint8 right_y;//ʮ�ּ�⵽�Ľǵ�
	uint8 Ramp_line;//�µ��ߣ����ڴ�������ͼȡ������Ѱ
	uint8 stop_line;//��ƫ���ֹ��
	uint8 line_forbid;
	uint16 bnum;//basemap�׵�����
	uint16 rnum;//����ͼ�ڵ�����
	uint16 lnum;
	uint16 dnum; //deletemapmap�ڵ�����
	uint8 lmin;
	uint8 rmin;
}IMG_STATUS;

typedef struct{
	uint8 sramp;
	uint8 ramp;
	uint8 annulus;
	uint8 obstacle;
	uint8 yhds;
	uint8 bump;
}IMG_FLAGS;
typedef struct{
	uint8 x;
	uint8 y;
}BOOM_POINT;
typedef struct PointNode{
	char num;
	BOOM_POINT point[40];
}PtStack;
typedef struct{
	uint16 numCnt;
	uint8 left;
	uint8 right;
	uint8 top;
	uint8 down;
}RegionInfo;
#define USE 2
#define ADD 1
typedef struct{
	uint16 cnt;
	float sumDev;
	uint8 flag;
}AnnulusDEV;
#define RF_INIT  {0,XX,0,0,YY}
/*�����趨*/
void standard();
void statusReset();
/*ͼ���ȡ*/
uint8 judge(uint8 x, uint8 y);
void searchimg(uint8 x, uint8 y);
void searchmap(uint8 x, uint8 y, uint8 src[][XM]);
void searchleftmap(uint8 x, uint8 y);
void searchrightmap(uint8 x, uint8 y);
void Get_insideMap();
/*Ԫ�ش���*/
uint8 Obstacle();
uint8 crossroadforleft();
uint8 crossroadforright();
uint8 AnnulusDeal(uint8 ADir, uint8 status);
uint8 goAnnulus();
uint8 leftAnnulusDetect();
uint8 rightAnnulusDetect();
uint8 leave(uint8 dir);
uint8 isEnter(uint8 dir);
uint8 czAnnulus();
uint8 go_ramp();
uint8 RampUp();
void deleteline();
/*������*/
uint8 isBumpNow();
uint8 isBump();
void getRegionInfo(uint8 x, uint8 y, uint8 src[][XM], RegionInfo *rf);
uint16 cntMap(uint8 x, uint8 y);
uint8 getMapYMin_Col(uint8 x, uint8 map[][XM]);
uint8 getMapYMin_Col2(uint8 x, uint8 y, uint8 map[][XM]);
void drawline(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM]);//y1>y2
void drawline2(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM]);
uint8 strJudge(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM], uint8 sy1, uint8 sy2);
uint8 X_WBW_Detect(char x1, char x2, uint8 y, uint8 map[][XM], uint8 flag);
uint8  horLineDect(uint8 y, uint8 dir, uint8 map[][XM]);
uint8 goBump();
#endif  // _DEAL_IMG_H
