#ifndef __DEAL_IMG_H__
#define __DEAL_IMG_H__


/*
 * 图像参数宏定义
 */
#define ROW     70      //CAMERA_H/VERTICAL
#define COLUMN  40      //CAMERA_W/8

#define ROW_ADD 80      //为寻找top增加的行数

#define XX      69
#define YY      39

#define STEP1     65      //55
#define THRESHOLD 0xf0    //0xf0可以忽视背景，可在任何背景色跑，0xff则只能在暗色背景地fa


/*
 *  供外部调用的函数接口声明
 */
extern void  StandardMap(void);       //得到标准赛道
extern void  CleanMap(void);      //迭代数组的初始化
extern uint8 OverSamping1(uint8 x, uint8 y);         //原图像的二值化
extern void OverSamping();
extern void DeleteMyself(); 

__RAMFUNC void  SearchMap_1st(uint8 x, uint8 y, uint8 src[][COLUMN]);        //迭代原图像数组得到初次数组
__RAMFUNC void  SearchMap_2nd(uint8 x, uint8 y, uint8 dst[][COLUMN], uint8 src[][COLUMN]);      //迭代初次数组得到左，右部分数组

extern void  DeleteLeftLine(void);    //删除左回环
extern void  DeleteRightLine(void);   //删除右回环

extern void DeleteBottomLeft();
extern void DeleteBottomRight();

/* 十字 */
extern uint8 CrossroadForRight(void);
extern uint8 CrossroadForLeft(void);
extern void Crossroad_Handle(void);           //十字和环形弯的总处理

/*其他类型*/
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
 * 外部调用的全局变量声明
 */
extern float left_k, right_k;
extern int8 circle_in_flag, circle_in_flag2;

extern uint8 CompressedImg[][COLUMN];
extern uint8 basemap[][COLUMN];	       //初次迭代得到的数组
extern uint8 leftmap[][COLUMN];	       //对左边再次迭代得到的数组
extern uint8 rightmap[][COLUMN];       //对右边再次迭代得到的数组

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

