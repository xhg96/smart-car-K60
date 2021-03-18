#include "control.h"
#include "deal_img.h"
#ifndef BOOM3_QT_DEBUG
//#include "lcd_key.h"
#include "motor.h"
#include "m_LCD.h"
#else
#include   <iostream>
using namespace std;
float LError;
float RError;
#endif
float k1[80] = {
	1.212121, 1.224806, 1.237759, 1.250990, 1.264506, 1.278317, 1.292434, 1.306865, 1.321623, 1.336717,
	1.352161, 1.367965, 1.384144, 1.400709, 1.417676, 1.435059, 1.452874, 1.471136, 1.489863, 1.509074,
	1.528786, 1.549020, 1.569796, 1.591138, 1.613068, 1.635611, 1.658793, 1.682641, 1.707185, 1.732456,
	1.758486, 1.785311, 1.812966, 1.841492, 1.870930, 1.901324, 1.932722, 1.965174, 1.998735, 2.033462,
	2.069417, 2.106667, 2.145282, 2.185339, 2.226920, 2.270115, 2.315018, 2.361734, 2.410374, 2.461059,
	2.513922, 2.569106, 2.626766, 2.687075, 2.750218, 2.816399, 2.885845, 2.958801, 3.035543, 3.116371,
	3.201621, 3.291667, 3.386924, 3.487859, 3.594994, 3.708920, 3.830303, 3.959900, 4.098573, 4.247312,
	4.407252, 4.579710, 4.766214, 4.968553, 5.188834, 5.429553, 5.693694, 5.984848, 6.307385, 6.666667 };

float k2[80] = {
	0.009506, 1.039279, 1.964688, 2.800334, 3.559774, 4.255556, 4.899258, 5.501522, 6.072093, 6.619856,
	7.152875, 7.678425, 8.203035, 8.732519, 9.272020, 9.826041, 10.398485, 10.992692, 11.611477, 12.257162,
	12.931622, 13.636312, 14.372313, 15.140363, 15.940897, 16.774083, 17.639859, 18.537973, 19.468015, 20.429457,
	21.421691, 22.444064, 23.495917, 24.576619, 25.685608, 26.822426, 27.986756, 29.178460, 30.397614, 31.644548,
	32.919883, 34.224563, 35.559899, 36.927602, 38.329821, 39.769181, 41.248817, 42.772417, 44.344252, 45.969218,
	47.652872, 49.401467, 51.221994, 53.122211, 55.110690, 57.196846, 59.390979, 61.704307, 64.149007, 66.738251,
	69.486240, 72.408246, 75.520645, 78.840959, 82.387885, 86.181340, 90.242495, 94.593812, 99.259081, 104.263456,
	109.633495, 115.397196, 121.584033, 128.224992, 135.352613, 143.001020, 151.205966, 160.004863, 169.436824, 179.542696 };

extern uint8 basemap[YM][XM];
extern uint8 leftmap[YM][XM];
extern uint8 rightmap[YM][XM];
extern uint8 insidemap[YM][XM];
extern uint8 leftline[YM], rightline[YM];
extern uint8 speedlline[YM], speedrline[YM];
extern uint8 stopFlag;
extern IMG_STATUS IS;
extern IMG_FLAGS IF;
extern uint16 param0;
extern uint16 obOffset;
extern uint16 obB;
extern AnnulusDEV AD;
int s0 = 0, delta[2] = { 0 };

float kb_send[2] = { 0 };
float Din_part = 0;
float Dout_part = 0;
uint16 taw = 5;
float deviation = 0;
float lastde = 0;
//float k=150;//155;
float K = 0;      //100
float BB = 0;     //100
//用于速度控制
float de_k = 0;
float ave_de = 0;
uint16 absde[3] = { 0 };
int dede[3] = { 0 };
int top = 0;//,top_save[2]={0};
int top_temp = 0;
int other_top = 0;       //入弯另外
int other_top_temp = 0;       //入弯另外
uint8 n1 = 0, n2 = 0;
uint8 LEFT, RIGHT, LEFT_F, RIGHT_F;
int leftnum = 0;
int rightnum = 0;
int lmin = 20;
int rmin = 20;
char control_line = 0;//0 双线 1 左线 2 右线 3 两条线都不要
float last_k = 0;
float last_b = 0;
uint16 Ann_k = 120;



CONTROL_PARAM CP = { 1180, 304, 60, 60 }; //

#if CAR_NUM==OLD
uint16 MID_PWM = 4850;
uint16 SERVOPWM_MAX = 1200;
uint16 SERVOPWM_MIN = 1350;
#else
uint16 MID_PWM = 4465;
uint16 SERVOPWM_MAX = 1180;
uint16 SERVOPWM_MIN = 1180;
#endif

PositionalPID_st Servo_PID;
extern uint16 obOffset;
void turn()
{
	float kbleft[2] = { 0 };
	float kbright[2] = { 0 };
	leftnum = 0;
	rightnum = 0;
	/******判断用于控制的图像*********/
	//  pit_time_start(PIT2);
	for (uint8 i = 0; i < YM; ++i)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (leftmap[i][j] == 2 || leftmap[i][j] == 3)   //数边界点左
				++leftnum;
			if (rightmap[i][j] == 2 || rightmap[i][j] == 3)  //数边界点右
				++rightnum;
		}
	}
	//   int dealTime=pit_time_get_us(PIT2);
	//   printf("%d\n",dealTime);
	if (IS.lnum < 600 && IS.lnum > 20)  //20<IS.lnum<600,左边的黑点数
		lmin = (int)(13 + (600 - IS.lnum) / 6.4);   //4
	else if (IS.lnum >= 600)//实际上只是比600大的情况
		lmin = 13;
	else
		lmin = 600;

	if (IS.rnum < 600 && IS.rnum > 20)
		rmin = (int)(13 + (600 - IS.rnum) / 6.4);
	else if (IS.rnum >= 600)
		rmin = 13;
	else
		rmin = 600;
	/******环岛***/
	if ((IF.annulus == AL1 || IF.annulus == AL2) && leftnum<lmin) rmin = 15;
	if ((IF.annulus == AR1 || IF.annulus == AR2) && rightnum<rmin) lmin = 15;
	if (IF.annulus == AL4&&rmin>35) rmin = 35;
	if (IF.annulus == AR4&&lmin>35) lmin = 35;
	/******坡道***/
	if (IF.ramp)
	{
		lmin = 20;
		rmin = 20;
		if (IS.top_rm[0]<20 && IS.right_y == YM || IS.top_rm[0]<10)
			rmin = 600;
		if (IS.top_lm[0]<20 && IS.left_y == YM || IS.top_lm[0]<10)
			lmin = 600;
	}
	/*********/
	if (IS.line_forbid == __Left) lmin = 1000;
	else if (IS.line_forbid == __Right) rmin = 1000;
	else if (IS.line_forbid == __Both)
	{
		rmin = 1000;
		lmin = 1000;
	}
	LEFT_F = 0;
	RIGHT_F = 0;
	for (int i = 0; i < XM; ++i)
	{
		if (leftmap[0][i] == 2)
		{
			LEFT = i;
			break; //最底部的线有边界线2，则跳出
		}
		if (i == XX)//39，最底部的线没有找出边界线
		{
			LEFT_F = 1;
			leftmap[0][leftline[0]] = 2;    //底部左无边线，则用标准线，人为赋值
		}
	}  //这个情况应该是十字路口图像已经到了图像底部的情况了
	for (int i = XX; i >= 0; --i)
	{
		if (rightmap[0][i] == 2)
		{
			RIGHT = i;
			break;
		}
		if (i == 0)
		{
			RIGHT_F = 1;
			rightmap[0][rightline[0]] = 2;//底部右无边线，则用标准线，人为赋值
		}
	}

	/********出界停车*********/
	if ((LEFT_F == 1 && RIGHT_F == 0))
	{
		if (RIGHT<10)
		{

			stopFlag = 1;
#ifndef BOOM3_QT_DEBUG
			stopReason = RunOutLine;
#endif
		}
	}
	else if (LEFT_F == 0 && RIGHT_F == 1)
	{
		if (LEFT>29)
		{

			stopFlag = 1;
#ifndef BOOM3_QT_DEBUG
			stopReason = RunOutLine;
#endif
		}
	}

	getline(leftmap, leftline, kbleft, 1);  //偏差算法，算偏差
	getline(rightmap, rightline, kbright, 2);
	float kk = CP.k / 10.0, bb = CP.b / 100.0;
	if (IF.annulus == AL1 || IF.annulus == AR1 )
	{
		kk = Ann_k;

	}
	if (IF.obstacle == __Right)
	{
		kbleft[1] -= obOffset / 10.0;
		kbright[1] -= obOffset / 10.0;
		bb = obB / 10.0;
	}
	else if (IF.obstacle == __Left)
	{
		kbleft[1] += obOffset / 10.0;
		kbright[1] += obOffset / 10.0;
		bb = obB / 10.0;
	}
#ifdef BOOM3_QT_DEBUG
	LError = kbleft[0] * kk + kbleft[1] * bb;
	RError = kbright[0] * kk + kbright[1] * bb;
#endif
	if (leftnum > lmin  && rightnum > rmin)
	{
		K = (kbleft[0] + kbright[0]) / 2;
		BB = (kbleft[1] + kbright[1]) / 2;
		kb_send[0] = K* kk;
		kb_send[1] = BB * bb;
		deviation = K* kk + BB * bb;

		de_k = (kbleft[0] + kbright[0]) / 2 * 100;
		control_line = 0;
	}
	else if (rightnum > rmin && leftnum <= lmin)//只有1边的情况
	{
		K = kbright[0];
		BB = kbright[1];
		kb_send[0] = K* kk;
		kb_send[1] = BB * bb;
		deviation = K* kk + BB * bb;
		de_k = kbright[0] * 100;
		control_line = 2;
	}
	else if (rightnum <= rmin && leftnum > lmin)
	{
		K = kbleft[0];
		BB = kbleft[1];
		kb_send[0] = K* kk;
		kb_send[1] = BB * bb;
		de_k = kbleft[0] * 100;
		deviation = K * kk + BB * bb;
		control_line = 1;
	}
	else
	{
#ifdef BOOM3_QT_DEBUG
		if (IF.annulus&&AD.flag == 2)
			deviation = AD.sumDev / AD.cnt;
#else
		if (IF.annulus&&AD.flag == 2&&GET_SWITCH8() == SWITCH_ON)
			deviation = AD.sumDev / AD.cnt;
#endif
		control_line = 3;
	}

	if (!(deviation == deviation&&K == K&&BB == BB))//在某个情况下会算出一个NaN
	{
		deviation = lastde;
		K = last_k;
		BB = last_b;
		kb_send[0] = K* kk;
		kb_send[1] = BB * bb;
		// NAN_count++;
	}

	lastde = deviation;
	last_k = K;
	last_b = BB;
	if (AD.flag == 1)
	{
		AD.sumDev += deviation;
		++AD.cnt;
	}
	if (deviation>0) Servo_PID.error = (int)(deviation* (CP.right_k / 10.0));
	if (deviation <= 0)Servo_PID.error = (int)(deviation* (CP.left_k / 10.0));
	/*******************************************************/
	Calc_ServoPID(&Servo_PID);//位置式PD

	/*****************************************************/
#ifndef BOOM3_QT_DEBUG
	ftm_pwm_duty(SD5_FTM, SD5_CH, MID_PWM + Servo_PID.PID_Out);  //舵机转向
#endif
	//*********************************************************//

	float ave_de_l, ave_de_r;
	ave_de_l = average_de(leftmap, leftline, 1);//左边的偏差值  **对比下和偏差算法的区别
	ave_de_r = average_de(rightmap, rightline, 2);  //右边的偏差值

	if (ave_de_l != 10000 && ave_de_r != 10000)  //两边都有边界
	{
		ave_de = (ave_de_l + ave_de_r) / 2;        //两边边界则取平均值
	}
	else if (ave_de_l == 10000 && ave_de_r != 10000)   //一边边界的情况
	{
		ave_de = ave_de_r;
	}
	else if (ave_de_r == 10000 && ave_de_l != 10000)
	{
		ave_de = ave_de_l;
	}
	absde[2] = absde[1];
	absde[1] = absde[0];
	absde[0] = abs((int)ave_de);

	dede[2] = dede[1];
	dede[1] = dede[0];
	dede[0] = absde[0] - absde[1];
}

void getTop()
{
	uint8 myflag = 0;
	for (uint8 i = 0; i < YM; ++i)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (basemap[YY - i][j] == 0)  //从顶部开始向下，从左向右若有白点
			{
				top_temp = YY - i;//在这里只对top_temp进行赋值
				myflag = 1;
				break;
			}
		}
		if (myflag)
			break;   //找到白点则跳出
	}


	for (uint8 i = 0; i<YM; ++i) //从底部向上寻other_top
	{
		if (basemap[i][speedlline[i]] && !insidemap[i][speedlline[i]])//||rightmap[i][speedlline[i]]==1)
		{
			other_top_temp = i;
			break;
		}
		if (basemap[i][speedrline[i]] && !insidemap[i][speedrline[i]])//|| leftmap[i][speedrline[i]]==1)
		{
			other_top_temp = i;
			break;
		}
		if (i >= YY)
			other_top_temp = YY;
	}
}

float average_de(uint8 src[][XM], uint8 width[YM], char flag) //leftmap, leftline
{
	float sum = 0;
	float x;
	int n = 0;
	uint8 stop = 75;
	uint8 d_line = 40;
	if (flag == 1)
	{
		for (uint8 i = d_line; i < stop; ++i)  //d_line = 40， YM=80（40-80）
		{
			for (uint8 j = 0; j < XM; ++j)  //XM=40
			{
				if (j + 1>XX) break;
				if (src[i][j] == 2 || src[i][j] == 3)
				{
					++n;  //边界黑点计数
					x = (float)(j - (int)width[i]) * (0.05625 * i + 1.25); //这里偏差只是做参考的，随便搞搞啦
					sum += x; //累加
				}
			}
		}
	}
	if (flag == 2)
	{
		for (uint8 i = d_line; i < stop; ++i)  //d_line = 40， YM=80（40-80）
		{
			for (uint8 j = XX; j >0; --j)  //XM=40
			{
				if (j - 1<0) break;;
				if (src[i][j] == 2 || src[i][j] == 3)
				{
					++n;  //边界黑点计数
					x = (float)(j - (int)width[i])* (0.05625 * i + 1.25);
					sum += x; //累加
				}
			}
		}
	}
	if (n == 0)
		return 10000;
	else
		return (sum / n);   //40-80行偏差的平均值
}
void getline(uint8 src[][XM], uint8 width[YM], float kb[2], uint8 flag)//最小二乘拟合
{
	float sumx = 0;
	float sumy = 0;
	float sumxy = 0;
	float sumx2 = 0;
	int n = 0;
	float x;
	float y;
	int lastx = 0, llastx = 0;
	int lasty = 0;
	int i = 0;
	int j = 0;
	// uint8 stop_line=69;
	if (flag == 1)
	{
		lastx = 0;
		lasty = 0;

		for (i = 0; i < IS.stop_line; i++)
		{
			for (j = 0; j < XX; j++)
			{
				if ((src[i][j] == 2 && (src[i][j + 1] == 0 || src[i][j + 1] == 2)) || src[i][j] == 3)  //去回环
				{
					if (i == lasty && (abs(j - lastx)>2 || abs(llastx - j)>3) && i>0 && src[i][j] != 3) //去水平线
						break;
					leftmap[i][j] = 3;
					n++;
					y = (float)((j - (int)width[i]))*k1[i];

					x = k2[i];
					sumx += x;
					sumy += y;
					sumxy += x * y;
					sumx2 += x * x;

					lastx = j;
					lasty = i;
				}
			}
			llastx = lastx;
		}
	}

	else if (flag == 2)
	{
		lastx = XX;
		llastx = 0;
		lasty = 0;
		for (i = 0; i < IS.stop_line; i++)
		{
			for (j = XX; j >0; j--)
			{
				if ((src[i][j] == 2 && (src[i][j - 1] == 0 || src[i][j - 1] == 2)) || src[i][j] == 3)
				{

					if (i == lasty && (abs(j - lastx)>2 || abs(llastx - j)>3) && i>0 && src[i][j] != 3)
						break;
					rightmap[i][j] = 3;
					n++;
					y = (float)((j - (int)width[i]))*k1[i];
					x = k2[i];
					sumx += x;
					sumy += y;
					sumxy += x * y;
					sumx2 += x * x;

					lastx = j;
					lasty = i;
				}
			}
			llastx = lastx;
		}
	}

	kb[0] = (n * sumxy - sumx * sumy) / (n * sumx2 - sumx * sumx);        //斜率
	kb[1] = sumy / n - kb[0] * sumx / n;          //截距
}
int16 Calc_ServoPID(PositionalPID_st *pp)
{
	float kp, kd;
	int16 error;
	static int16 d_error = 0;
	error = pp->error;      			   //e(k)
	d_error = (int16)((pp->error - pp->preError)*(1 - taw / 10.0) + d_error*(taw / 10.0));    //e(k)-e(k-1)
	//  d_error = (int16)(pp->error - pp->prepreError);
	pp->prepreError = pp->preError;
	pp->preError = error;

	kp = 1;
	if ((d_error>0 && error<0) || (d_error<0 && error>0)){
		kd = (float)(pp->Kdout / 100.0);
		Din_part = 0;
		Dout_part = kd * d_error;
	}
	else{
		kd = (float)(pp->Kdin / 100.0);
		Din_part = kd * d_error;
		Dout_part = 0;
	}
#ifdef BOOM3_QT_DEBUG
	//	kd = 0;
#endif
	int16 temp = (int16)(kp * error + kd * d_error);

	if (temp > SERVOPWM_MAX)
		pp->PID_Out = SERVOPWM_MAX;
	else if (temp< -SERVOPWM_MIN)
		pp->PID_Out = -SERVOPWM_MIN;
	else
		pp->PID_Out = temp;

	return (pp->PID_Out);
}
void Servo_Init()
{
	Servo_PID.error = 0;
	Servo_PID.preError = 0;
	Servo_PID.prepreError = 0;
	Servo_PID.Kp = 100;
	Servo_PID.Kdin = 252;
	Servo_PID.Kdout = 306;
	Servo_PID.PID_Out = 0;
#ifndef BOOM3_QT_DEBUG
	ftm_pwm_init(SD5_FTM, SD5_CH, SD5_HZ, MID_PWM);
#endif
}


