#include "deal_img.h"
/*ͼ�����õ�char�����з������͵ģ�����������charҪ���ó�signed*/
#ifdef BOOM3_QT_DEBUG
#include   <iostream>
#include "time.h"
#include <malloc.h>
using   namespace   std;
uint8 *flagAdress[4] = { NULL };
extern int dataTemp[3];
uint8 stopLineDetected;
#else
#include "boom3_debug.h"
extern uint8 stopLineDetected;

extern void setTimeBeep_ms(uint8 timeSet, uint8 vt);
extern uint32 runTimeCnt;
#endif
/*****ͼ������*****/
uint8 imgbuff[CAMERA_SIZE];
uint8 basemap[YM][XM];
uint8 leftmap[YM][XM];    // 2 3
uint8 rightmap[YM][XM];
uint8 insidemap[YM][XM];
uint8 deletemap[YM][XM];// = {0};
uint8 CCD_BUFF_INIT[TSL1401_SIZE];
/*****��������*****/
uint8 leftline[YM], rightline[YM];
uint8 speedlline[YM], speedrline[YM];//���ڼ���other_top
/*****״̬��Ϣ*****/
IMG_STATUS IS;
IMG_STATUS IS_INTI;
IMG_FLAGS IF = { 0 };
IMG_FLAGS LAST_IF = { 0 };
uint8 ob_direction;//�ϰ�
PtStack noDown;
/*����*/
uint16 param0 = 15;
uint16 obOffset = 66;
uint16 obB = 34;
uint16 numCnt;
AnnulusDEV AD;
AnnulusDEV AD_INIT = { 0 };
uint8 enhanceFlag = 0;
/**********�ⲿ����*********/
extern int top, top_temp, other_top, other_top_temp;
extern float k2[80], k1[80];
extern int Hgyro1[5];
extern float HRoll[5];
extern float Pitch, Roll, Yaw;
//extern uint8 CCD_BUFF[TSL1401_SIZE];
void standard()                            //��׼�ߣ���������ͷ�߶ȽǶȵ���
{
	float x, x11, x22;

	for (uint8 i = 0; i < YM; ++i)
	{
		x = (float)(-27 * i / 158 + 16.5);
		leftline[i] = (uint8)ceil(19 - x);
		rightline[i] = (uint8)floor(20 + x);
		x22 = -0.1701*i / 2.7 + 38.1 + 0.5;
		x11 = 0.1701*i / 2.7 + 0.9 + 0.5;
		speedlline[i] = (uint8)(x11 + 14.5);
		speedrline[i] = (uint8)(x22 - 14.5);
	}
	IS_INTI.num_lm = 0;
	IS_INTI.num_rm = 0;
	memset((uint8*)IS_INTI.end_lm, 0, sizeof(uint8)* 2);
	memset((uint8*)IS_INTI.end_rm, XX, sizeof(uint8)* 2);
	memset((uint8*)IS_INTI.top_lm, 0, sizeof(uint8)* 2);
	memset((uint8*)IS_INTI.top_rm, 0, sizeof(uint8)* 2);
	IS_INTI.left_x = 0;
	IS_INTI.right_x = XM;
	IS_INTI.left_y = YM;
	IS_INTI.right_y = YM;
	IS_INTI.Ramp_line = 70;
	IS_INTI.stop_line = STOP_LINE;
	IS_INTI.line_forbid = NO;
	IS_INTI.down_lm = YM;
	IS_INTI.down_rm = YM;
	IS_INTI.bnum = 0;
	IS_INTI.lnum = 0;
	IS_INTI.rnum = 0;
	IS_INTI.dnum = 0;
	IS_INTI.lmin = YM;
	IS_INTI.rmin = YM;
	// IS_INTI.top = 0;
	//memcpy(CCD_BUFF_INIT, CCD_BUFF, TSL1401_SIZE);
}

void statusReset()
{
#if BOOM_SD_SAVE
	memset((uint8*)SAB.wifi_buff.map, 0xff, sizeof(uint8)*YM*XM / 8);
#endif
	memset((uint8*)basemap, 1, sizeof(uint8)*YM*XM);
	memset((uint8*)leftmap, 0, sizeof(uint8)*YM*XM);
	memset((uint8*)rightmap, 0, sizeof(uint8)*YM*XM);
	memset((uint8*)insidemap, 1, sizeof(uint8)*YM*XM);
	memset((uint8*)deletemap, 0, sizeof(uint8)*YM*XM);
	memset((uint8*)noDown.point, 0, sizeof(uint8)* 20);
	noDown.num = 0;
	IS = IS_INTI;
}
/*****���ͼ������*******/
uint8 judge(uint8 x, uint8 y)    //�к�ɫ
{
	for (uint8 i = 1; i <= VERTICAL; ++i)               //3�в�һ�У�240��80
	{
		if (imgbuff[(CAMERA_H - y * VERTICAL - i) * XM + x] >= THRESHOLD)
			return 1;
	}
	return 0;
}
void searchimg(uint8 x, uint8 y) //baseMap�����︳��ֵ
{
	if (!judge(x, y))//��ɫ����if�������
	{
		++IS.bnum;
		insidemap[y][x] = 0;
		basemap[y][x] = 0; //��ɫ��Ϊ0
#if BOOM_SD_SAVE
		SAB.wifi_buff.map[y][x / 8] &= ~(0x80 >> (x % 8));
#endif
		if (x > 0 && basemap[y][x - 1] == 1)//����baseMap��ֵΪ1
			searchimg(x - 1, y);
		if (x < XX && basemap[y][x + 1] == 1)//����  xx=39
			searchimg(x + 1, y);
		if (y > 0 && basemap[y - 1][x] == 1)//����
			searchimg(x, y - 1);
		if (y < YY && basemap[y + 1][x] == 1)//YY=79
			searchimg(x, y + 1);//����
	}
	else
		basemap[y][x] = 2; //baseMap,��ɫ��Ϊ2

}
void searchmap(uint8 x, uint8 y, uint8 src[][XM])
{
	if (basemap[y][x])
	{
		src[y][x] = 1;
		if (x > 0 && src[y][x - 1] == 0)
			searchmap(x - 1, y, src);
		if (x < XX && src[y][x + 1] == 0)
			searchmap(x + 1, y, src);
		if (y > 0 && src[y - 1][x] == 0)
			searchmap(x, y - 1, src);
		if (y < YY && src[y + 1][x] == 0)
			searchmap(x, y + 1, src);
	}
	else
		src[y][x] = 2;
}
void searchCountmap(uint8 x, uint8 y, uint8 src[][XM])
{
	if (basemap[y][x])
	{
		++numCnt;
		src[y][x] = 1;
		if (x > 0 && src[y][x - 1] == 0)
			searchCountmap(x - 1, y, src);
		if (x < XX && src[y][x + 1] == 0)
			searchCountmap(x + 1, y, src);
		if (y > 0 && src[y - 1][x] == 0)
			searchCountmap(x, y - 1, src);
		if (y < YY && src[y + 1][x] == 0)
			searchCountmap(x, y + 1, src);
	}
	else
		src[y][x] = 2;
}
void getRegionInfo(uint8 x, uint8 y, uint8 src[][XM], RegionInfo *rf)
{
	if (basemap[y][x])
	{
		++rf->numCnt;
		if (y >= rf->top) rf->top = y;
		if (y <= rf->down) rf->down = y;
		if (x <= rf->left) rf->left = x;
		if (x >= rf->right) rf->right = x;
		src[y][x] = 1;
		if (x > 0 && src[y][x - 1] == 0)
			getRegionInfo(x - 1, y, src, rf);
		if (x < XX && src[y][x + 1] == 0)
			getRegionInfo(x + 1, y, src, rf);
		if (y > 0 && src[y - 1][x] == 0)
			getRegionInfo(x, y - 1, src, rf);
		if (y < YY && src[y + 1][x] == 0)
			getRegionInfo(x, y + 1, src, rf);
	}
	else
		src[y][x] = 2;
}
uint16 cntMap(uint8 x, uint8 y)
{
	numCnt = 0;
	uint8 map[YM][XM] = { 0 };
	searchCountmap(x, y, map);
	return numCnt;
}
void searchleftmap(uint8 x, uint8 y) //leftMap baseMap
{

	if (basemap[y][x])
	{
		if (IS.top_lm[IS.num_lm]<y)
			IS.top_lm[IS.num_lm] = y;
		if (IS.end_lm[IS.num_lm]<x)
			IS.end_lm[IS.num_lm] = x;
		leftmap[y][x] = 1;
		insidemap[y][x] = 0;
		++IS.lnum;
		if (x > 0 && leftmap[y][x - 1] == 0)
			searchleftmap(x - 1, y);
		if (x < XX && leftmap[y][x + 1] == 0)
			searchleftmap(x + 1, y);
		if (y < YY && leftmap[y + 1][x] == 0)
			searchleftmap(x, y + 1);
		if (y > 0 && leftmap[y - 1][x] == 0)//y <= IS.Ramp_line&&
		{
			if (y <= IS.Ramp_line)
				searchleftmap(x, y - 1);
			else if (basemap[y - 1][x] && noDown.num<40)
			{
				noDown.point[noDown.num].x = x;
				noDown.point[noDown.num].y = y - 1;
				++noDown.num;
			}
		}
	}
	else
		leftmap[y][x] = 2;   //��ɫ�ڱ߽������м�Ϊ2�����˳�����
}
void searchrightmap(uint8 x, uint8 y) //leftMap baseMap
{
	if (basemap[y][x])
	{
		if (IS.top_rm[IS.num_rm]<y)
			IS.top_rm[IS.num_rm] = y;
		if (IS.end_rm[IS.num_rm]>x)
			IS.end_rm[IS.num_rm] = x;
		rightmap[y][x] = 1;
		insidemap[y][x] = 0;
		++IS.rnum;
		if (x > 0 && rightmap[y][x - 1] == 0)  //src==0����û�и���ֵ
			searchrightmap(x - 1, y);
		if (x < XX && rightmap[y][x + 1] == 0)
			searchrightmap(x + 1, y);
		if (y < YY && rightmap[y + 1][x] == 0)
			searchrightmap(x, y + 1);
		if (y > 0 && rightmap[y - 1][x] == 0) //y <= IS.Ramp_line&&
		{
			if (y <= IS.Ramp_line)
				searchrightmap(x, y - 1);
			else if (basemap[y - 1][x] && noDown.num<40)
			{
				noDown.point[noDown.num].x = x;
				noDown.point[noDown.num].y = y - 1;
				++noDown.num;
			}
		}
	}
	else
		rightmap[y][x] = 2;   //��ɫ�ڱ߽������м�Ϊ2�����˳�����
}
void searchdeletemap(uint8 x, uint8 y, uint8 src[][XM], uint8 src2[][XM])
{
	if (src2[y][x])
	{
		++IS.dnum;
		src[y][x] = 1;
		insidemap[y][x] = 0;
		if (x > 0 && src[y][x - 1] == 0)
			searchdeletemap(x - 1, y, src, src2);
		if (x < XX && src[y][x + 1] == 0)
			searchdeletemap(x + 1, y, src, src2);
		if (y > 0 && src[y - 1][x] == 0)
			searchdeletemap(x, y - 1, src, src2);
		if (y < YY && src[y + 1][x] == 0)
			searchdeletemap(x, y + 1, src, src2);
	}
	else
		src[y][x] = 2;
}
//void searchdeletemap(uint8 x, uint8 y)
//{
//    if (basemap[y][x])
//    {
//        deletemap[y][x] = 1;
//        insidemap[y][x] = 0;
//        if (x > 0 && deletemap[y][x - 1] == 0)
//            searchdeletemap(x - 1, y);
//        if (x < XX && deletemap[y][x + 1] == 0)
//            searchdeletemap(x + 1, y);
//        if (y > 0 && deletemap[y - 1][x] == 0)
//            searchdeletemap(x, y - 1);
//        if (y < YY && deletemap[y + 1][x] == 0)
//            searchdeletemap(x, y + 1);
//    }
//    else
//        deletemap[y][x] = 2;
//}
void searchdeletemap2(uint8 x, uint8 y)
{
	if (basemap[y][x])
	{
		++IS.dnum;
		deletemap[y][x] = ~0x01;
		insidemap[y][x] = 0;
		if (x > 0 && deletemap[y][x - 1] == 0 && leftmap[y][x - 1] == 0 && rightmap[y][x - 1] == 0)
			searchdeletemap2(x - 1, y);
		if (x < XX && deletemap[y][x + 1] == 0 && leftmap[y][x + 1] == 0 && rightmap[y][x + 1] == 0)
			searchdeletemap2(x + 1, y);
		if (y > 0 && deletemap[y - 1][x] == 0 && leftmap[y - 1][x] == 0 && rightmap[y - 1][x] == 0)
			searchdeletemap2(x, y - 1);
		if (y < YY && deletemap[y + 1][x] == 0 && leftmap[y + 1][x] == 0 && rightmap[y + 1][x] == 0)
			searchdeletemap2(x, y + 1);
	}
	else
		deletemap[y][x] = ~0x02;
}
/*****���ͼ������*******/
/************************/
void deleteline()
{
#define DELETE_NUM 2
	uint8 down = YM;
	for (char i = IS.top_lm[0]; i>1; --i)
	{
		if (X_WBW_Detect(0, XX, i, leftmap, goRight))
			down = i;
		else
			break;
	}
	if (down != YM && IS.top_lm[0] >= DELETE_NUM + down)        //�Д����^deletenum �ŕ��h�Q
	{
		for (uint8 i = down; i + DELETE_NUM <= IS.top_lm[0]; ++i)//�حh������ȫ�h��//�޷����������������
		{
			for (uint8 j = 0; j < XM; ++j)
			{
				if (leftmap[i][j] == 2)
					leftmap[i][j] = 0;
				if (leftmap[i][j] == 1)
					break;
			}
		}
	}
	/****�ҾQ****/
	down = YM;
	for (char i = IS.top_rm[0]; i>1; --i)
	{
		if (X_WBW_Detect(XX, 0, i, rightmap, goLeft))
			down = i;
		else
			break;
	}
	if (down != YM && IS.top_rm[0] >= DELETE_NUM + down)
	{
		for (uint8 i = down; i + DELETE_NUM <= IS.top_rm[0]; ++i)
		{
			for (uint8 j = 0; j < XM; ++j)
			{
				if (rightmap[i][XX - j] == 2)
					rightmap[i][XX - j] = 0;
				if (rightmap[i][XX - j] == 1)
					break;
			}
		}
	}
}
uint8 crossroadforleft()
{
	uint8 high = 3;
	int upd;//��¼�ǵ��±��ص�YMλ��
	int upu;//��¼�ǵ��ϱ��ص�YMλ��
	int up;//��¼�ǵ��XMλ��
	char undelete_flag = 0;
	uint8 flag = 0;
	for (uint8 i = XX - IS.end_lm[0]; i < XM; ++i)//������ת����ʵ���Ǵ�ͼ���������
	{
		for (uint8 j = 0; j < YM - 6; ++j)//��������
		{
			if (leftmap[j][XX - i] == 1)//�����ڵ�
			{
				upd = j;                  //22222222
				up = XX - i;             //111112
				upu = YY;                //1112
				for (uint8 k = j + 1; k < YM; ++k)//���ҵ��ڵ��λ��������һ�񣬼�������Ѱ
				{
					if (i + high>XX)break;           //�����겻���������˳�
					if (leftmap[k][XX - i] != 1)//�ҵ�leftmap�ϱ߽磬���ǵ��ϱ߽�// && leftmap[k][XX - i - high] != 1 �Ӹ��������Ժ�upu��ֵ�����ߣ�
					{                              //���ᵼ�³�Բ���Ľǵ㲻��ɾ����ѹ��
						upu = k - 1;
						flag = 1;
						break;
					}
				}
				break;
			}
		}
		if (flag)
			break;
	}
	//���Ͻǵ�������������ֵѰ�����
	int downu;//����high����ϱ߽�
	int downd;//����high����±߽�
	uint8 down;//extra��down����Խ�絫Ӱ�첻��
	if (flag)
	{
		down = up - high;//extra��down����Խ�絫Ӱ�첻��֮��ĵڶ������з���
		uint8 upmax = 20;   //�ǵ����±߽��y֮������ֵ
		if (upu >= 70 || up <= high || upu - upd >upmax)// || upd <= 5  ȥ�����������Ժ�ʮ��ɾ�߻�ɾ�һЩ
			return 0;
		int cnt = 0;
		for (uint8 i = upu; i < YM; ++i)     //�ӽǵ��ϱ߽�����
		{
			if (rightmap[i][up] == 1) //��basemap��ʮ�ֱض����У������ܼ����������Ϊʮ��
			{
				undelete_flag = 1;
				break;
			}
			if (i - upu == (uint8)((YM - upu) / 1.2) || undelete_flag)
				break;
		}
		if (undelete_flag == 1)              //��Ŀ������ɾ����Բ��ʱ�Ľǵ㣬��ֹѹ��
		{
			cnt = 0;                           //ʮ�ֶ�ֱ�ǵ�ʱ������ų�����
			for (uint8 i = upu + 5; i <= upu + 9; i++)
			{
				for (int j = up + 5; j <= XX; j++)
				{
					if (rightmap[i][j] == 2)
					{
						cnt++;
						break;
					}
				}
			}
			if (cnt <= 2)
				undelete_flag = 0;
		}
		flag = 0;
		for (uint8 i = 0; i < YM; ++i)//��������
		{
			if (leftmap[i][down] == 1)//������ɫ�ĵ�
			{
				downd = i;
				downu = YY;
				flag = 1;
				for (uint8 j = i; j < YM; ++j)//��������ɨ//1
				{
					if (leftmap[j][down] != 1)//�����Ǻ�ɫ����
					{
						downu = j - 1;//ˢ��ʮ��·���±��ص�YMλ��
						break;//1
					}
				}
				break;//2
			}
		}
	}
	else
		return 0;

	uint8 downmax = 40;//�ױߵ���󳤶ȣ����Ҫ����úõ���
	int pointnum = 0;
	int mynum = 0;
	if (flag && downu - downd < downmax)//�ױ��ҵ��ҳ��ȷ��ϣ�3�����ض�Ӧ�������ص�����Ӧ��Ҳ��һ����Χ�ڣ�
	{
		for (uint8 i = down; i <= up; ++i)
		{
			for (uint8 j = downd; j <= downu; ++j)//�������⼸���������ɵ��������ڵĺڵ���
			{
				if (leftmap[j][i] == 1)
					++pointnum;//����
			}
		}
		mynum = (int)((downu - downd) * (high + 1) / 2);//�����������
		mynum += (int)((downu - downd)*1.5) + 3;//������������ߵĳ���*1.8
		if (mynum >= pointnum)//������������,���˸��ϸ���ж�  //����оݾ������ʲôԭ��û��
		{  //����������ĵ���
			IS.left_x = up;
			IS.left_y = upu;
			if (IF.annulus == AL1)//Բ��״̬1���Ҳ�������ʶ��������ɾ��
			{
				do
				{
					if (leftmap[IS.top_lm[0]][0] != 1)//��������Ȼ�޷��ų� ����ͼ��1.bmp�����������������������߻ᱻ���ã������Ȳ���
						break;
					uint8 min = getMapYMin_Col2(0, IS.top_lm[0] + 1, basemap);
					if (min != YM&& k2[min] - k2[IS.top_lm[0]] > 100)
						break;
					return 0;
				} while (0);
			}
			if (upu < 40 && IF.annulus == 0 && IS.num_lm == 1)//���Ƕ����ϲ�����Ļ��һ������ʱ��
			{
				for (uint8 i = 5; i < (YM - upu) * 2 / 3 + 2; ++i)
				{               //79
					if (i + upu > YY)//��ֹԽ��
						break;
					if (basemap[i + upu][up])//�ǰ�
					{                             //0                       //0 2        //2
						if (leftmap[i + upu][up] != 1 && rightmap[i + upu][up] != 1 && basemap[i + upu][up] && !((IS.down_rm<5||enhanceFlag&&IS.top_lm[0]>40) && IS.top_rm[0]>i + upu && k2[IS.top_rm[0]] - k2[IS.top_lm[0]] >= 30))//Բ��������Ѱ
						{
							searchleftmap(up, i + upu);//����Ѱ�߽���  ��Բ��ʱ��ֹѰ�߹��ߣ�����һ���ߵĽǵ�
							++IS.num_lm;
						}
						break;                                         //���˽�����Ӧ�ý���
					}
				}
			}
			if (undelete_flag == 0)
			{
				for (uint8 i = 0; i <= up + 5; ++i)//�������ң�ֹ��up+5
				{
					if (i>XX) break;
					for (uint8 j = 0; j < YM; ++j)//������ԭ�㿪ʼѰ
					{
						if (leftmap[j][i] == 1)//������ɫ����
						{
							for (uint8 k = j + 1; k < YM; ++k)//����������
							{
								if (leftmap[k][i] == 2)//�����±߽�  //1
								{
									if (upd> k + 10)//��֤���ǻػ�   upd>k+10  ���˷�����������˻�Ӱ��ʮ��ɾ��
										break;
									for (uint8 m = k; m < YM; ++m)//����������
									{
										if (leftmap[m][i] == 0)//������ɫ����û�и�ֵ�������� //2&&leftmap[m+1][i] == 0
										{
											for (uint8 n = m + 1; n < YM; ++n)//����������
											{
												if (leftmap[n][i] == 2)//����ͼ�������ϣ��߽�  //3
												{
													for (uint8 a = n; a < YM; ++a)//����������
													{
														//                              if(leftmap[a][i] != 2)//���������ֻ��Ϊ�˰��ϱ߽���ɾ�ɾ�
														//                                break;
														if (leftmap[a][i] == 2)
															leftmap[a][i] = 0;//ɾ����ͼ�ϱ߽� //4
													}
													break;
												}
											}
											break;
										}
										if (leftmap[m][i] == 2)
											leftmap[m][i] = 0;//ɾ���±߽� extra����ʵ����д��else
									}
									break;
								}
							}
							break;
						}
					}
				} //

				for (int i = up; i <= up + 8; i++)   /* �о��������������ɾ�߹��࣬ɾ��̫���ֱ�����ע��*/
				{
					if (i>XX) break;
					for (int j = upu + 3; j <= YY; j++)
					{
						if (leftmap[j][i] == 2){
							leftmap[j][i] = 0;
							break;
						}
					}
				}
				/*if (upd>30)
				for (uint8 i = upd; i <= upu; ++i)
				{
				if (leftmap[i][up + 1] == 2) leftmap[i][up + 1] = 0;
				}*/
			}
		}
	}
	else
		return 0;

	return 0;
}

uint8 crossroadforright()
{
	uint8 high = 3;
	int upd;
	int upu;
	int up;
	char undelete_flag = 0;
	uint8 flag = 0;
	for (uint8 i = IS.end_rm[0]; i < XM; ++i)
	{
		for (uint8 j = 0; j < YM - 6; ++j)
		{
			if (rightmap[j][i] == 1)
			{
				upd = j;
				up = i;
				upu = YY;
				for (uint8 k = j + 1; k < YM; ++k)
				{
					if (i + high>XX)break;
					if (rightmap[k][i] != 1)//&& rightmap[k][i+ high] != 1
					{
						upu = k - 1;
						flag = 1;

						break;
					}
				}
				break;
			}
		}
		if (flag)
			break;
	}
	int downu;
	int downd;
	uint8 down;
	if (flag)
	{
		down = up + high;
		uint8 upmax = 20;
		if (upu >= 70 || up >= XM - high || upu - upd >upmax)// || upd <= 5
			return 0;

		int cnt = 0;
		for (uint8 i = upu; i < YM; ++i)     //��ֱ��
		{
			if (leftmap[i][up] == 1)
			{
				undelete_flag = 1;
				break;
			}
			if (i - upu == (uint8)((YM - upu) / 1.2) || undelete_flag)
				break;
		}
		if (undelete_flag == 1)
		{
			cnt = 0;
			for (uint8 i = upu + 5; i <= upu + 9; i++)
			{
				for (int j = up - 5; j >= 0; j--)
				{
					if (leftmap[i][j] == 2)
					{
						cnt++;
						break;
					}
				}
			}
			if (cnt <= 2)
				undelete_flag = 0;
		}

		flag = 0;
		for (uint8 i = 0; i < YM; ++i)
		{
			if (rightmap[i][down] == 1)
			{
				downd = i;
				downu = YY;//������
				flag = 1;
				for (uint8 j = i; j < YM; ++j)
				{
					if (rightmap[j][down] != 1)
					{
						downu = j - 1;
						break;
					}
				}
				break;
			}
		}
	}
	else
		return 0;

	uint8 downmax = 40;//Ҫ�ص��
	int pointnum = 0;
	int mynum = 0;    //ʮ�ֺ�ɫ���������µ�
	if (flag && downu - downd < downmax)
	{
		for (uint8 i = up; i <= down; ++i)
		{
			for (uint8 j = downd; j <= downu; ++j)
			{
				if (rightmap[j][i] == 1)
					++pointnum;
			}
		}
		mynum = (int)((downu - downd) * (high + 1) / 2);
		mynum += (int)((downu - downd) *1.5) + 3;//1.8
		if (mynum >= pointnum)//������������,���˸��ϸ���ж�
		{
			IS.right_x = up;
			IS.right_y = upu;
			if (IF.annulus == AR1) //�Ȳ�ɾ����
			{
				do
				{
					if (rightmap[IS.top_lm[0]][XX] != 1)
						break;
					uint8 min = getMapYMin_Col2(XX, IS.top_rm[0] + 1, basemap);
					if (min != YM&& k2[min] - k2[IS.top_lm[0]] > 100)
						break;
					return 0;
				} while (0);
			}
			if (upu <  40 && IF.annulus == 0 && IS.num_rm == 1)  //�����޸�һ��  ȷ��Ϊ�ǻػ������԰����ֵ�������ų��ǻػ�
			{
				for (uint8 i = 5; i < (YM - upu) * 2 / 3 + 2; ++i)
				{
					if (i + upu > YY)
						break;
					if (basemap[i + upu][up])
					{
						if (leftmap[i + upu][up] != 1 && rightmap[i + upu][up] != 1 && basemap[i + upu][up] && !((IS.down_lm<5 ||enhanceFlag&&IS.top_rm[0]>40) && IS.top_lm[0]>i + upu && k2[IS.top_lm[0]] - k2[IS.top_rm[0]] >= 30))
						{
							searchrightmap(up, i + upu);
							++IS.num_rm;
						}
						break;
					}
				}
			}
			if (undelete_flag == 0)
			{
				for (int i = XX; i >= up - 5; --i)
				{
					if (i<0)break;
					for (uint8 j = 0; j < YM; ++j)
					{
						if (rightmap[j][i] == 1)
						{
							for (uint8 k = j + 1; k < YM; ++k)
							{
								if (rightmap[k][i] == 2)
								{
									if (upd - 10 > k)   //���ڱ���ע�͵ģ�����֮����˵��Ŀǰ�о�����
										break;
									for (uint8 m = k; m < YM; ++m)
									{
										if (rightmap[m][i] == 0)//&&rightmap[m+1][i] == 0
										{
											for (uint8 n = m + 1; n < YM; ++n)
											{
												if (rightmap[n][i] == 2)
												{
													for (uint8 a = n; a < YM; ++a)
													{
														//                            if(rightmap[a][i] != 2)
														//                              break;
														if (rightmap[a][i] == 2)
															rightmap[a][i] = 0;
													}
													break;
												}

											}
											break;
										}
										if (rightmap[m][i] == 2)
											rightmap[m][i] = 0;
									}
									break;
								}
							}
							break;
						}
					}
				}
				/*���ɾ���ֱ�̫�󣬿��ܻ�����ʮ�ֲ��ȶ�����ע�͵�����һ��*/
				for (int i = up - 8; i <= up; i++)
				{
					if (i<0) continue;
					for (int j = upu + 3; j <= YY; j++)
					{
						if (rightmap[j][i] == 2){
							rightmap[j][i] = 0;
							break;
						}
					}
				}
				/*if (upd>30)
				for (uint8 i = upd; i <= upu; ++i)
				{
				if (rightmap[i][up - 1] == 2) rightmap[i][up - 1] = 0;
				}*/
			}




		}
	}
	else
		return 0;

	return 0;
}
uint8 sramp()
{
	if (IS.left_y != YM || IS.right_y != YM) return 0;
	float max = 20;
	float topWidth = 0;
	uint8 leftMax_i = 0, rightMax_i = 0;
	int leftMax = -100, rightMax = -100;
	int leftEnd = -100, rightEnd = -100;
	for (uint8 i = 0; i < max; ++i)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (leftmap[YY - i][XX - j] == 2)
			{
				for (uint8 k = XX - j + 1; k < XM; ++k)
				{
					if (rightmap[YY - i][k] == 2)
					{
						if (i >= 13 && k - (XX - j) < rightline[YY - i] - leftline[YY - i] + (max - i) * 4 / max + 1)  //������+0
							return 0;
						if (i <= 3 && k - (XX - j) > rightline[YY - i] - leftline[YY - i] + 2)  //������+0
							return 0;
						if (i>8 && i<13 && k - (XX - j) < rightline[YY - i] - leftline[YY - i])  //������+0
							return 0;
						if (k <= 19 || XX - j >= 20)
							return 0;
						if (topWidth < (k - (XX - j))*k1[YY - i])
							topWidth = (k - (XX - j))*k1[YY - i];

						if ((i == 0 || i == max) && leftEnd < XX - j - (int)leftline[YY - i])
							leftEnd = XX - j - (int)leftline[YY - i];
						else if (leftMax <= XX - j - (int)leftline[YY - i])
						{
							leftMax = XX - j - (int)leftline[YY - i];
							leftMax_i = i;
						}
						if ((i == 0 || i == max) && rightEnd < (int)rightline[YY - i] - k)
							rightEnd = (int)rightline[YY - i] - k;
						else if (rightMax <= (int)rightline[YY - i] - k)
						{
							rightMax = (int)rightline[YY - i] - k;
							rightMax_i = i;
						}

						break;
					}
					if (k == XX)
						return 0;
				}
				break;
			}
			if (j == XX)
				return 0;
		}
	}
	if (topWidth < 60 || leftMax>leftEnd && leftMax_i>3 || rightMax>rightEnd && rightMax_i>3) return 0;
	return 1;
}
uint8 ramp()
{
	float max = 20;
	for (uint8 i = 0; i < max; ++i)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (leftmap[YY - i][XX - j] == 2)
			{
				for (uint8 k = XX - j + 1; k < XM; ++k)
				{
					if (rightmap[YY - i][k] == 2)
					{
						if (k - (XX - j) < rightline[YY - i] - leftline[YY - i] + (max - i) * 4 / max + 1 || k<19 || XX - j>20)  //������
							return 0;
						break;
					}
					if (k == XX)
						return 0;
				}
				break;
			}
			if (j == XX)
				return 0;
		}
	}
	return 1;
}
uint8 rampUp()
{
	for (uint8 i = YY; i > YY - 20; --i)
	{
		uint8 flag = 0;
		for (uint8 j = 0; j < XM; ++j)
		{
			if (leftmap[i][j] == 2)
			{
				for (uint8 k = j + 1; k < XM; ++k)
				{
					if (rightmap[i][k] == 2)
						flag = 1;
				}
			}
		}
		if (flag == 0) return 1;
	}
	return 0;
}
uint8 isBumpNow()
{
	if (Hgyro1[0]<-2900)
		return 1;
	else return 0;
}
uint8 isBump()
{
	for (uint8 i = 0; i<10; ++i)
	{
		if (Hgyro1[i]<-2500/*&&HRoll[i]<6*/)
			return 1;
	}
	return 0;
}
uint8 go_ramp()
{
	static uint8 pitchFlag = 0;
	static uint8 my_ramp_flag = 0;
	static uint8 delay_time = 0;
	static uint8 xiaodou = 0;
	static uint8(*fp)() = ramp;
	uint8  all_delay = (uint8)(50 * Fre);
#ifdef BOOM3_QT_DEBUG
	//	my_ramp_flag = 0;
	//	delay_time = 0;
#endif
	//�������ѹ·��ʶ��Ϊ����·�棬���������µ�����ֻ�ܿ����£�С��ʶ���Pitchʶ��
	if (ramp() && my_ramp_flag == 0 && isBump() == 0)      //���¼�⵽
	{
		my_ramp_flag = 1;
		xiaodou = 0;
		fp = ramp;
		IF.sramp = 0;
		pitchFlag = 0;
	}
	//	if (sramp() && my_ramp_flag == 0)      //���¼�⵽
	//	{
	//		my_ramp_flag = 1;
	//		xiaodou = 0;
	//		fp = sramp;
	//		IF.sramp = 1;
	//		all_delay = 30;
	//		pitchFlag = 0;
	//	}
	if (RampUp() && my_ramp_flag == 0)      //���¼�⵽
	{
		my_ramp_flag = 2;
		all_delay = 30;
		pitchFlag = 0;
		IF.sramp = 0;
	}
	if (Pitch>24 && my_ramp_flag == 0 && top_temp<70)      //���¼�⵽
	{
		pitchFlag = 1;
		IF.sramp = 1;
		my_ramp_flag = 2;
		all_delay = 30;
	}
	if (my_ramp_flag == 1)
	{
		xiaodou++;
		if (xiaodou>10 * Fre)
		{
			if (!(*fp)()) //if (rampUp())//             //�����ϼ�ⲻ��
			{
				my_ramp_flag = 2;
			}
		}
		/*	else if (xiaodou > all_delay)
		{
		my_ramp_flag = 0;
		delay_time = 0;
		}*/
	}

	if (my_ramp_flag == 2)
	{
		delay_time++;
		if (ramp())          //�����ٴμ�⵽
		{
			my_ramp_flag = 3;
			delay_time = 0;
		}
		//                else if(Pitch<-5)
		//                {
		//                    my_ramp_flag = 3;
		//                    delay_time = 0;
		//                    setTimeBeep_ms(100, VHIGH);
		//                }
		else if (delay_time>all_delay)      //�����У����������ʱ��Ϊ50
		{
			my_ramp_flag = 0;
			delay_time = 0;
		}
	}

	if (my_ramp_flag == 3 || my_ramp_flag == 4)
	{
		delay_time++;
		if (delay_time>10 * Fre && my_ramp_flag == 3 && !ramp())         //��ʱ���ָ���ʼ��
			my_ramp_flag = 4;
		if (delay_time>50 * Fre)         //��ʱ���ָ���ʼ��
		{
			my_ramp_flag = 0;
			delay_time = 0;
		}
	}

	if ((my_ramp_flag == 1 || my_ramp_flag == 2))//&&(pitchFlag==0)
	{
		if (IF.sramp)
			IS.stop_line = 40;
		else
			IS.stop_line = 30;
	}

	if (my_ramp_flag == 1 || my_ramp_flag == 2 || my_ramp_flag == 3)
		return my_ramp_flag;
	else return 0;
	//Ч��������ֵΪ1��2��3���µ�������0ʱ����
}
void Get_insideMap()
{
	/*����rampline���ϵ�����ͼѰ�������ƣ���Ӧ������ͼ������������Ѱ��deletemap��ֵΪ~0x01��~0x02*/
	for (char i = 0; i < noDown.num; ++i)
	{
		if (leftmap[noDown.point[i].y][noDown.point[i].x] == 0 && rightmap[noDown.point[i].y][noDown.point[i].x] == 0 && deletemap[noDown.point[i].y][noDown.point[i].x] == 0)
			searchdeletemap2(noDown.point[i].x, noDown.point[i].y);
	}
	if (IS.bnum + IS.lnum + IS.rnum == XM*YM) return;
	for (uint8 i = 0; i < YM; ++i)
	{    //��ûѰ�����ߣ���ֹ����ȡ�ϰ���
		if (basemap[i][0] && leftmap[i][0] != 1 && rightmap[i][0] != 1 && deletemap[i][0] == 0)
		{
			searchdeletemap(0, i, deletemap, basemap);
			if (IS.lmin == YM) IS.lmin = i;
		}
		if (basemap[i][XX] && leftmap[i][XX] != 1 && rightmap[i][XX] != 1 && deletemap[i][XX] == 0)
		{
			searchdeletemap(XX, i, deletemap, basemap);
			if (IS.rmin == YM) IS.rmin = i;
		}
	}
	for (uint8 i = 0; i <= XX; i++)
	{
		if (basemap[YY][i] && leftmap[YY][i] != 1 && rightmap[YY][i] != 1 && deletemap[YY][i] == 0)
			searchdeletemap(i, YY, deletemap, basemap);
		//    if(basemap[0][i] && leftmap[0][i] != 1 && rightmap[0][i] != 1 && deletemap[0][i] == 0)
		//      searchmap(i, 0, deletemap, basemap); //x,y
	}
}
uint8 goAnnulus()
{
	//�뻷һ��ʱ��û����ĳ�龰����Ϊ������
	/*
	*/
	static uint8 status = 0;
	static uint8 dir = 1;
	static int time;
	static uint16 in = 0;
#ifdef BOOM3_QT_DEBUG
	flagAdress[0] = &status;
	flagAdress[1] = &IF.annulus;
	//status = 0;
	//#else
	//	if (status != 0 && IS.left_y > YM / 2 && IS.right_y > YM / 2)
	//	{
	//		status = 0;
	//
	//	}
#endif
	if (status == 0 && leftAnnulusDetect())
	{
		status = 1;
		dir = goLeft;
		AnnulusDeal(dir, status);
		time = 0;
		in = 0;
		AD = AD_INIT;
	}
	else if (status == 0 && rightAnnulusDetect())
	{
		status = 1;
		dir = goRight;
		AnnulusDeal(dir, status);
		time = 0;
		in = 0;
		AD = AD_INIT;
	}
	else if (status == 1)
	{
		if (isEnter(dir)) status = 2;
		uint8 ret = AnnulusDeal(dir, status);
		if (ret) ++in;
		else if (in >= 2 && status == 1)
		{
			IS.line_forbid = (dir == goLeft ? __Right : __Left);
		}
	}
	else if (status == 2)
	{
		if (dir == goLeft&&getMapYMin_Col(0, deletemap) == YM&&IS.end_rm[0] == 0 && top_temp < 70 || dir == goRight&&getMapYMin_Col(XX, deletemap) == YM&&IS.end_lm[0] == XX&&top_temp < 70)
		{
			status = 3;//��Ϊ�õ�top_temp������Ҫ���� AnnulusDealǰ��
			AD.flag = 1;
		}
		AnnulusDeal(dir, status);
	}
	else if (status == 3)
	{
		if (dir == goRight&&IS.left_y<70 || dir == goLeft&&IS.right_y<70)
		{
			AD.flag = 2;
			status = 4;
		}
	}
	else if (status == 4)
	{
		uint8 lmin = getMapYMin_Col(0, deletemap);
		uint8 rmin = getMapYMin_Col(XX, deletemap);
		if (dir == goRight && IS.num_lm == 0 && lmin>30 && rmin>30)//&& (IS.dnum >= (YM - lmin)*XM || IS.dnum >= (YM - rmin)*XM))// && rmin>lmin��СԲ�����ܲ�����
			status = 5;//�ײ�ȫ��
		else if (dir == goLeft &&  IS.num_rm == 0 && rmin > 30 && lmin > 30)// && (IS.dnum >= (YM - lmin)*XM || IS.dnum >= (YM - rmin)*XM))//&& lmin>rmin
			status = 5;
		leave(dir);
	}
	else if (status == 5)
	{
#define MIN_DOWN 30
		if (dir == goRight)
		{
			//IS.line_forbid = __Right;
			if (IS.down_lm>MIN_DOWN)
				IS.line_forbid = __Left;//__Both;
			else if (IS.down_lm <= MIN_DOWN)
				status = 6;
		}
		else if (dir == goLeft)
		{
			//IS.line_forbid = __Left;
			if (IS.down_rm>MIN_DOWN)
				IS.line_forbid = __Right;//__Both;
			else if (IS.down_rm <= MIN_DOWN)
				status = 6;
		}

	}
	else if (status == 6)
	{
		++time;
		if (time>40)
			status = 0;//��ʱ����ֹ����ʱ�����뻷ʶ��
	}
	if (status&&status != 6) return (dir - 1) * 6 + status;
	else return NO;
	// return status;
}
uint8 isEnter(uint8 dir)
{
	if (dir == goLeft&&IS.num_rm > 0) return 0;
	if (dir == goRight&&IS.num_lm > 0) return 0;
	for (uint8 i = 0; i < YM / 2; ++i)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (deletemap[i][j] == 0)
			{
				for (uint8 k = j + 1; k < XM; ++k)
				{
					if (deletemap[i][k] == 1)
					{
						for (uint8 m = k + 1; m < XM; ++m)
						{
							if (deletemap[i][m] == 0) return 1;
						}
						break;
					}
				}
				break;
			}
		}
	}
	return 0;
}
uint8 leave(uint8 dir)
{
	if (dir == goLeft&&IS.right_x >= 5 && IS.right_y<60)//50��Ϊ40��󻷳������ã�С����û����
	{
		if (IS.right_x <= 34)
		{
			uint8 y1 = getMapYMin_Col2(IS.right_x, 0, rightmap);
			uint8 y2 = getMapYMin_Col2(IS.right_x + 5, 0, rightmap);
			if (y1 != YM&&y2 != YM && 0)
			{
				float k = (float)(y1 - y2) / 5;
				uint8 last = 0;
				float stp = 0.04f;
				uint8 min = getMapYMin_Col(0, deletemap);
				if (min<(int)(y1 + (k - stp*IS.right_x)*IS.right_x))//min==YM����Ҳ�����
				{
					stp = (k - (float)(min - y1) / (IS.right_x)) / IS.right_x;
				}
				for (uint8 j = 1; j <= IS.right_x; ++j)
				{
					k = k - stp;
					uint8 temp = (uint8)(y1 + k*j);
					if (temp >= YM || temp<last) break;//�������YM�´�ѭ��k-0.2ȴС��YM�˾��ֲ����ˣ�����Ӧ�ò������
					rightmap[temp][IS.right_x - j] = 2;
					last = temp;
				}
			}
			else
			{
				uint8 min = getMapYMin_Col(0, deletemap);
				if (min<YM&&min>IS.right_y)
				{
					uint8 upd = getMapYMin_Col(IS.right_x, rightmap);
					drawline2(IS.right_x, upd, 0, min, rightmap);
				}
			}
			//IS.line_forbid = __Right;
		}
		else if (IS.down_rm>20)
		{
			IS.line_forbid = __Right;
		}
		else
		{
			uint8 min = getMapYMin_Col(0, deletemap);
			if (min<YM&&min>IS.right_y)
			{
				uint8 upd = getMapYMin_Col(IS.right_x, rightmap);
				drawline2(IS.right_x, upd, 0, min, rightmap);
			}
		}
	}
	else if (dir == goRight&&IS.left_x <= XX - 5 && IS.left_y<60)
	{
		if (IS.left_x >= 5 && 0)
		{
			uint8 y1 = getMapYMin_Col2(IS.left_x, 0, leftmap);
			uint8 y2 = getMapYMin_Col2(IS.left_x - 5, 0, leftmap);
			if (y1 != YM&&y2 != YM)
			{
				float k = (float)(y1 - y2) / 5;
				uint8 last = 0;
				float stp = 0.04f;
				uint8 min = getMapYMin_Col(XX, deletemap);
				if (min<(int)(y1 + (k - stp*(XX - IS.left_x))*(XX - IS.left_x)))//min==YM����Ҳ�����
				{
					stp = (k - (float)(min - y1) / (XX - IS.left_x)) / (XX - IS.left_x);
				}
				for (uint8 j = 1; j + IS.left_x<XM; ++j)
				{
					k = k - stp;
					uint8 temp = (uint8)(y1 + k*j);
					if (temp >= YM || temp<last) break;//�������YM�´�ѭ��k-0.2ȴС��YM�˾��ֲ����ˣ�����Ӧ�ò������
					leftmap[temp][IS.left_x + j] = 2;
					last = temp;
				}
			}
			else
			{
				uint8 min = getMapYMin_Col(XX, deletemap);
				if (min<YM&&min>IS.left_y)
				{
					uint8 upd = getMapYMin_Col(IS.left_x, leftmap);
					drawline2(XX, min, IS.left_x, upd, leftmap);
				}

			}
		}
		else
		{
			uint8 min = getMapYMin_Col(XX, deletemap);
			if (min<YM&&min>IS.left_y&&IS.left_y>0)
			{
				uint8 upd = getMapYMin_Col(IS.left_x, leftmap);
				drawline2(XX, min, IS.left_x, upd, leftmap);
			}

			//  IS.line_forbid = __Left;

		}

	}
	//   if (dir == goLeft)//&&IS.right_x>3
	//    {
	//        int cnt=0;
	//        for(uint8 j=0;XX-j>=0;++j)//IS.right_x
	//        {
	//            uint8 y1=getMapYMin_Col2(XX-j,leftmap);
	//            if(y1!=YM)
	//            {
	//                if(XX-j-5>=0)//IS.right_x
	//                {
	//                    uint8 y2=getMapYMin_Col2(XX-j-5,leftmap);
	//                    if(y2!=YM)
	//                    {
	//                        float k=-((float)(y2-y1))/5;
	//                        cnt++;
	//                        cout<<k<<" ";
	//                        if(cnt%5==0) cout<<"|";
	//                    }
	//                }
	//            }
	//        }
	//    }
	//    cout<<endl;
	return 0;
}
int getLeftSimilarity()
{
	//  int similarity=0;
	// if(IS.left_x)
	return 0;
}
#define DROW 20
uint8 leftAnnulusDetect()
{
	//�����top_temp���ƽ��������ܿ���Բ��ʱ��ʶ�����Ҫ����Բ��ʱ����ʱʶ��top_temp���С
	if (IS.num_lm != 1 || IS.num_rm != 1 || IS.top_rm[0] <= 56 || IS.top_rm[0] - IS.down_rm<30 || top_temp<65 || IS.end_lm[0]>20 || k2[IS.top_rm[0]] - k2[IS.top_lm[0]]<20)//|| IS.end_rm[0]<15)//k2[IS.top_rm[0]] - k2[IS.top_lm[0]]��ͼ���29������ͼ�񼯺�4.bmp�����
		return 0;
	int min = getMapYMin_Col(0, deletemap);
	if (min != YM&&cntMap(0, min) <= 50) return 0;//����λ�ò��ܽ�������Ȼ��Խ��
	if (IS.top_rm[0]<min || IS.right_y + 6 <= min) return 0; //������ͼ�񼯺�3.bmp�������IS.right_y <= min���ܻ���֣����Ը�min-10-10
	//if (IS.num_lm == 1)
	{
		if (IS.top_lm[0]<5 || min <= 54) return 0;
		if (IS.down_lm<3 && k2[IS.top_lm[0]]>77)
			return 0;//Ϊ�˷�ֹ�µ�������ʮ��ʱ��ֻ����һ��ʮ�ֵ�����
		if (IS.top_lm[0]<20 && IS.down_rm>10
			|| IS.down_lm >= 30) return 0;//leftmap[IS.top_lm[0]][0] ==2Ҳ������Ҫ���|| leftmap[IS.top_lm[0]][0] == 0
		if (IS.down_rm>0)
		for (uint8 j = XX; j >20; --j)
		{
			if (rightmap[IS.down_rm - 1][j] == 1) return 0;//���м���1.bmp���
		}
		if (IS.left_y<YM)
		for (int j = IS.left_x; j>0; --j)
		{
			if (leftmap[IS.left_y][j] == 0) return 0;
		}
		uint8 down = YY;
		for (int j = IS.end_lm[0]; j >= 0; --j)
		{
			for (uint8 i = 0; i<YM; ++i)
			{
				if (leftmap[i][j] == 1)
				{
					if (down<i) return 0;
					down = i;
					break;
				}
			}
		}
		if (min >= 75) return 0;
		for (int i = 0; i<YM; ++i)
		{
			if (deletemap[i][3] == 1)
			{
				if (i - min >= 2)
					return 0;
				break;
			}
		}
		for (int i = 0; i<YM; ++i)
		{
			if (deletemap[i][7] == 1)
			{
				if (i - min >= 4)
					return 0;
				break;
			}
		}
	}
	/*��ֱ���ж�*/
	if (IS.top_lm[0] - IS.down_lm> 40 && IS.end_lm[0]>12)
	{
		uint8 x1, y1, x2, y2;
		for (uint8 j = 1; j < XM; ++j)
		{
			if (leftmap[IS.down_lm][XX - j] == 1)
			{
				x1 = XX - j + 1;
				y1 = IS.down_lm;
				break;
			}
			if (j == XX) return 0;
		}
		for (uint8 j = 1; j < XM; ++j)
		{
			if (leftmap[IS.top_lm[0]][XX - j] == 1)
			{
				x2 = XX - j + 1;
				y2 = IS.top_lm[0];
				break;
			}
			if (j == XX) return 0;
		}
		if (strJudge(x1, y1, x2, y2, leftmap, y1, y2 - 5)) return 0;

	}
	/*ֱ���ж�*/
	if (IS.down_rm > 0)
	{
		for (uint8 j = XX; j >1; --j)
		{
			if (rightmap[IS.down_rm + 4][j] != 1) break;
			if (j <= XX - 3) return 0;
		}
	}
	uint8 x1, y1, x2, y2;
	for (uint8 j = 1; j < XM; ++j)
	{
		if (rightmap[IS.down_rm][j] == 2)
		{
			//	if (rightmap[IS.down_rm][j]==1)
			//		x1 = j-1;
			//	else
			x1 = j;
			y1 = IS.down_rm;
			break;
		}
		if (j == XX) return 0;
	}
	for (uint8 j = 1; j < XM; ++j)
	{
		if (rightmap[min][j] == 2)
		{
			//	if (rightmap[min][j] == 1)
			//		x2 = j-1;
			//else
			x2 = j;
			y2 = min;
			break;
		}
		if (j == XX) return 0;
	}
	return strJudge(x1, y1, x2, y2, rightmap, y1, y2);
}
uint8 rightAnnulusDetect()
{
	// cout<<(int)IS.top_lm[0]<<endl;
	//�����top���ƽ��������ܿ���Բ��ʱ��ʶ�����Ҫ����Բ��ʱ����ʱʶ��top���С
	if (IS.num_rm != 1 || IS.num_lm != 1 || IS.top_lm[0] <= 56 || IS.top_lm[0] - IS.down_lm<30 || top_temp<65 || IS.end_rm[0]<19 || k2[IS.top_lm[0]] - k2[IS.top_rm[0]]<20)//IS.top_lm[0] - IS.down_lm<30�ȴ�40��30����
		return 0;

	int min = getMapYMin_Col(XX, deletemap);
	if (IS.top_lm[0]<min || IS.left_y + 6 <= min) return 0;//-10
	if (min != YM&&cntMap(XX, min) <= 50) return 0;
	//if (IS.num_rm == 1)
	{/*rightmap[IS.top_lm[0]][XX]==0���ʮ�ֺܸߣ�������ʱ�����У�����ͼ�����м���2.bmp
	 ��һ�ַ����а취���ұ߽���y����ɨ�裬ɨ���ܳ�����ֱ�߽���2������Ϊ��ֱ��������Բ������������
	 ����Ϊ��һ���з�Բ��Ҳ���ܱ��ų������ڿ��������������ֹ��������*/
		if (IS.top_rm[0]<5 || min <= 54) return 0;
		if (IS.down_rm<3 && k2[IS.top_rm[0]]>77)
			return 0;//Ϊ�˷�ֹ�µ�������ʮ��ʱ��ֻ����һ��ʮ�ֵ�����
		if (IS.top_rm[0]<20 && IS.down_lm>10//......rm
			|| IS.down_rm >= 30) return 0;//|| rightmap[IS.top_rm[0]][XX] == 0
		if (IS.down_lm>0)
		for (uint8 j = 0; j < 19; ++j)
		{
			if (leftmap[IS.down_lm - 1][j] == 1) return 0;
		}
		uint8 down = YY;
		if (IS.right_y<YM)
		for (uint8 j = IS.right_x; j<XM; ++j)
		{
			if (rightmap[IS.right_y][j] == 0) return 0;
		}
		for (uint8 j = IS.end_rm[0]; j<XM; ++j)
		{
			for (uint8 i = 0; i<YM; ++i)
			{
				if (rightmap[i][j] == 1)
				{
					if (down<i) return 0;
					down = i;
					break;
				}
			}
		}
		if (min >= 75) return 0;
		for (int i = 0; i<YM; ++i)
		{
			if (deletemap[i][XX - 3] == 1)
			{
				if (i - min >= 2)
					return 0;
				break;
			}
		}
		for (int i = 0; i<YM; ++i)
		{
			if (deletemap[i][XX - 7] == 1)
			{
				if (i - min >= 4)
					return 0;
				break;
			}
		}
	}
	/*��ֱ���ж�*/
	if (IS.top_rm[0] - IS.down_rm> 40 && IS.end_rm[0]<XX - 12)
	{
		uint8 x1, y1, x2, y2;
		for (uint8 j = 1; j < XM; ++j)
		{
			if (rightmap[IS.down_rm + 5][j] == 1)
			{
				x1 = j - 1;
				y1 = IS.down_rm + 5;
				break;
			}
			if (j == XX) return 0;
		}
		for (uint8 j = 1; j < XM; ++j)
		{
			if (rightmap[IS.top_rm[0]][j] == 1)
			{
				x2 = j - 1;
				y2 = IS.top_rm[0];
				break;
			}
			if (j == XX) return 0;
		}
		if (strJudge(x1, y1, x2, y2, rightmap, y1, y2 - 5)) return 0;

	}
	/*ֱ���ж�*/
	if (IS.down_lm > 0)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (leftmap[IS.down_lm + 4][j] != 1) break;
			if (j >= 3) return 0;
		}
	}
	uint8 x1, y1, x2, y2;
	for (uint8 j = 1; j < XM; ++j)
	{
		if (leftmap[IS.down_lm][XX - j] == 2)
		{
			x1 = XX - j;
			y1 = IS.down_lm;
			break;
		}
		if (j == XX) return 0;
	}
	for (uint8 j = 1; j < XM; ++j)
	{
		if (leftmap[min][XX - j] == 2)
		{
			x2 = XX - j;
			y2 = min;
			break;
		}
		if (j == XX) return 0;
	}
	return strJudge(x1, y1, x2, y2, leftmap, y1, y2);
}
uint8 AnnulusDeal(uint8 ADir, uint8 status)
{
#define yhmap deletemap
	if (ADir == NO) return 0;
	uint8 down = YM, up = YM, downX = XM;
	uint8 ret = 0;
	if (ADir == goLeft)
	{
		int min = getMapYMin_Col(0, deletemap);
		if (min == YM&&status == 1) IS.line_forbid = __Right;//�����ƫ����������ֹ�����ܱ����ϴδ��
		if (top_temp>min - 1) top_temp = min - 1;
		if (min != YM && (IS.num_lm == 1 && k2[min] - k2[IS.top_lm[0]] > 90 || k2[min]> 90 && IS.num_lm == 0)) return 0;//СԲ������ʶ�������Ȳ�����
		if (min == YM) return 0;//IS.top_lm[0]<5 && X_WBW_Detect(0, XX, IS.lmin - 2, deletemap, goRight) == 0 ||   //&& horLineDect(IS.lmin - 1,goRight,deletemap)<7
		//uint8 yhmap[YM][XM] = { 0 };
		uint8 myflag = 0;
		//uint8 lastdown = YM;
		//	uint8 lastdownX = XM;
		if (min != YM)
		{
			memset((uint8*)deletemap, 0, sizeof(uint8)*YM*XM);
			searchmap(0, min, yhmap);
		}
		//	searchmap(0, min, yhmap);
		for (uint8 j = 0; j<XM; ++j)
		{
			for (uint8 i = 0; i<YM - 1; ++i)
			{
				if (yhmap[i][j] == 2 && yhmap[i + 1][j] != 2 || j<3)
				{
					rightmap[i][j] = 3;
					//		if (i<down || down == YM)
					{
						//		lastdown = down;
						down = i;
						//		lastdownX = downX;
						downX = j;
					}
					break;
				}
				else if (yhmap[i][j] == 2 && yhmap[i + 1][j] == 2)
				{
					if (j<XX&&getMapYMin_Col(j + 1, yhmap)>i&&myflag == 0)//��֤�����м�����
					{
						myflag = 1;
						break;
					}
					else
					{
						rightmap[i][j] = 3;
						rightmap[i + 1][j] = 3;
						//	if (i< down || down == YM)
						{
							//			lastdown = down;
							down = i;
							//			lastdownX = downX;
							downX = j;
						}
						break;
					}
				}
			}
			if (myflag) break;
		}
		if (down - min > 0 || down > 75 || downX<5 && down<40 || X_WBW_Detect(0, downX + 5, down - 2, yhmap, goRight) && down != YM)
		{
			for (uint8 i = min - 10; i < down + 5 && i < YM; ++i)
			{
				for (uint8 j = 0; j <= downX; ++j)
				{
					if (rightmap[i][j] == 3) rightmap[i][j] = 0;
				}
			}
			return 0;
		}
		if (down<other_top_temp) other_top_temp = down;
		uint8 flag2 = 0;
		int x, y;
		if (down != YM)
		for (uint8 i = 1; i<XM; ++i)
		{
			float kk = 1.5;//
			x = downX + i;
			y = (int)(down - i*kk);
			if (x>XX || y<0) break;
			if (rightmap[y][x] == 2 || x == XX || y == 0)
			{
				uint8 bo = y + 1;
				if (rightmap[y][x] == 2)
					flag2 = 1;;
				for (uint8 k = bo; k<YM; ++k)
				{
					for (uint8 m = 0; m<XM; ++m) //���油��һ��Ҫ��3��Ȼ���ڴ˴���ɾ��
					{
						if (rightmap[k][m] == 2)
							rightmap[k][m] = 0;
					}
				}
				break;
			}
		}
		/*****���ֱ���-��ɫ-���ߵ�ʱ��ɾ����߱���*********/
		for (uint8 i = down; i<YM; ++i)//i<STOP_LINE
		{
			for (int j = downX; j>0; --j)
			{
				if (rightmap[i][j] == 3)
				{
					for (int k = j - 1; k>0; --k)
					{
						if (basemap[i][k]) break;
						if (leftmap[i][k] == 0)
						{
							for (int m = k - 1; m>0; --m)
							{
								if (basemap[i][k]) break;
								if (leftmap[i][m] == 3)
									leftmap[i][m] = 0;
							}
							break;
						}
					}
					break;
				}
			}
		}
		/*******�ײ�����***********/
		if (flag2&&down != YM)
		{
			drawline2(x, y, downX, down, rightmap);
			ret = 1;
		}
		if (flag2 == 0 && down != YM)//IS.down_rm>10 &&
		{
			/*float k = (float)(downX - rightline[0]) / down;
			if (k>-3 && k<0)
			for (uint8 i = 0; i<(10>down ? down : 10); ++i)
			{
			rightmap[i][(int)(rightline[0] + k*i)] = 2;
			}*/
			drawline2(rightline[0], 0, downX, down, rightmap);
			ret = 1;
		}

	}
	else if (ADir == goRight)
	{
		int min = getMapYMin_Col(XX, deletemap);
		if (min == YM&&status == 1) IS.line_forbid = __Left;
		if (top_temp > min - 1) top_temp = min - 1;
		if (min != YM && (IS.num_lm == 1 && k2[min] - k2[IS.top_lm[0]] > 90 || k2[min] > 90 && IS.num_rm == 0)) return 0;//СԲ������ʶ�������Ȳ�����
		if (min == YM) return 0;//IS.top_rm[0]<5 && X_WBW_Detect(XX, 0, IS.rmin - 2, deletemap, goLeft) == 0 ||   && horLineDect(IS.rmin - 1, goLeft, deletemap)<7
		//|| IS.top_rm[0]<5 && X_WBW_Detect(XX, 0, IS.rmin - 1, deletemap, goLeft) == 0 && horLineDect(IS.rmin - 1, goLeft, deletemap)<6
		//uint8 yhmap[YM][XM] = { 0 };
		uint8 myflag = 0;
		//	uint8 lastdown = YM;
		//		uint8 lastdownX = XM;
		if (min != YM)
		{
			memset((uint8*)deletemap, 0, sizeof(uint8)*YM*XM);
			searchmap(XX, min, yhmap);
		}
		//searchmap(XX, min, yhmap);
		for (uint8 j = 0; j < XM; ++j)
		{
			for (uint8 i = 0; i<YM - 1; ++i)
			{
				if (yhmap[i][XX - j] == 2 && yhmap[i + 1][XX - j] != 2 || j>XX - 3)
				{
					leftmap[i][XX - j] = 3;
					//	if (i<down || down == YM)
					{
						//			lastdown = down;
						down = i;
						//			lastdownX = downX;
						downX = XX - j;
					}
					break;
				}
				else if (yhmap[i][XX - j] == 2 && yhmap[i + 1][XX - j] == 2 && myflag == 0)
				{
					if (XX - j > 0 && getMapYMin_Col(XX - j - 1, yhmap) > i)//��֤�����м�����
					{
						myflag = 1;
						break;
					}
					else
					{
						leftmap[i][XX - j] = 3;
						leftmap[i + 1][XX - j] = 3;
						//if (i< down || down == YM)
						{
							//			lastdown = down;
							down = i;
							//			lastdownX = downX;
							downX = XX - j;
						}
						break;
					}
				}
			}
			if (myflag) break;
		}
		/*if (lastdown<down)
		{
		leftmap[down][downX] = 0;
		down = lastdown;
		downX = lastdownX;
		}*/
		//cout<<(int)down<<"  "<<(int)downX<<endl;
		if (down - min > 0 || down > 75 || downX > XX - 5 && down < 40 || X_WBW_Detect(XX, downX - 5, down - 2, yhmap, goLeft) && down != YM)
		{
			for (uint8 i = min - 10; i < down + 5 && i < YM; ++i)
			{
				for (char j = XX; j >= downX; --j)
				{
					if (leftmap[i][j] == 3) leftmap[i][j] = 0;
				}
			}
			return 0;
		}
		if (down < other_top_temp) other_top_temp = down;
		uint8 flag2 = 0;
		int y, x;
		if (down != YM)
		for (uint8 i = 1; i < XM; ++i)
		{
			float kk = 1.5;
			y = (int)(down - i*kk);
			x = downX - i;
			if (x < 0 || y < 0) break;
			if (leftmap[y][x] == 2 || x == 0 || y == 0)
			{
				uint8 bo = y + 1;
				if (leftmap[y][x] == 2)
					flag2 = 1;
				for (uint8 k = bo; k < YM; ++k)
				{
					for (uint8 m = 0; m < XM; ++m)
					{
						if (leftmap[k][m] == 2)
							leftmap[k][m] = 0;
					}
				}
				break;
			}
		}
		/*****���ֱ���-��ɫ-���ߵ�ʱ��ɾ���ұ߱���*********/
		for (uint8 i = down; i < YM; ++i)
		{
			for (uint8 j = downX; j < XM; ++j)
			{
				if (leftmap[i][j] == 3)
				{
					for (uint8 k = j + 1; k < XM; ++k)
					{
						if (basemap[i][k]) break;
						if (leftmap[i][k] == 0)
						{
							for (uint8 m = k + 1; m < XM; ++m)
							{
								if (basemap[i][k]) break;
								if (leftmap[i][m] == 3)
									leftmap[i][m] = 0;
							}
							break;
						}
					}
					break;
				}
			}
		}
		if (flag2&&down != YM)
		{
			drawline2(downX, down, x, y, leftmap);
			ret = 1;
		}
		/*******�ײ�����***********/
		if (flag2 == 0 && down != YM)//IS.down_lm>10 &&
		{
			/*	float k = (float)(downX - leftline[0]) / down;
			if (k<3 && k>0)
			for (uint8 i = 0; i<(10>down ? down : 10); ++i)
			{
			leftmap[i][(int)(leftline[0] + k*i)] = 2;
			}*/
			drawline2(downX, down, leftline[0], 0, leftmap);
			ret = 1;
		}
	}
	if (up<IS.stop_line) IS.stop_line = up + 1;
	return ret;
}

uint8 getMapYMin_Col(uint8 x, uint8 map[][XM])
{
	for (uint8 i = 0; i<YM; ++i)
	{
		if (map[i][x] == 1)
			return i;
	}
	return YM;
}
uint8 getMapYMin_Col2(uint8 x, uint8 y, uint8 map[][XM])
{
	for (uint8 i = y; i<YM; ++i)
	{
		if (map[i][x] == 1) break;
		if (map[i][x] == 2)
			return i;
	}
	return YM;
}
void drawline(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM])//y1>y2
{
	if (y1 == y2) return;
	float k = (float)(x1 - x2) / (y1 - y2);
	for (uint8 i = y2; i <= y1; ++i)
	{
		map[i][(uint8)(x2 + k*(i - y2))] = 2;
	}
}
void drawline2(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM])//x1>x2
{
	char lasty = YM;
	if (x1 == x2) return;
	float k = (float)(y1 - y2) / (x1 - x2);
	float mid = (float)(x1 + x2) / 2;
	uint8 climax = (uint8)9.5 * (x1 - x2) / XM;//12.0
	for (uint8 j = x2; j < x1; ++j)
	{
		uint8 temp = (uint8)((j - x2)*k + y2 + climax - climax*(2 * fabs(j - mid) / (x1 - x2))*(2 * fabs(j - mid) / (x1 - x2)));
		if (temp>YY) continue;
		map[temp][j] = 3;
		char gap = (char)temp - lasty;
		if (gap > 1 && lasty != YM)
		{
			while (--gap)
				map[temp - gap][j] = 3;
		}
		if (gap < -1 && lasty != YM)
		{
			while (++gap)
				map[temp - gap][j - 1] = 3;
		}
		lasty = temp;
	}
}
//uint8 getYhtTop()
//{
//	IF.yhds = YM;
//	if (top < 79) return YM;
//	if ((IS.num_lm == 2 && IS.end_lm[1]>IS.end_lm[0] + 5 || IS.num_lm == 1 && IS.down_lm>2)//&& IS.down_lm >= IS.down_rm
//		&& IS.right_y >= 65
//		&& IS.end_rm[0]>14
//		&& IS.end_rm[1]>14
//		&& IS.top_rm[0] >= 65
//		&& IS.top_rm[0]>(IS.top_lm[0]>IS.top_lm[1] ? IS.top_lm[0] : IS.top_lm[1])
//		)
//	{
//		uint8 end = IS.end_lm[1] > IS.end_lm[0] ? IS.end_lm[1] : IS.end_lm[0];
//		if (end < 3) return YM;
//		uint8 upu = YM, upd = YM, downd = YM, downu = YM;
//		for (uint8 i = 0; i<YM - 4; ++i)
//		{
//			if (leftmap[i][end] == 1)
//			{
//				for (uint8 k = i + 1; k < YM - 6; ++k)
//				{
//					if (basemap[k][end] == 0)
//					{
//						upu = k - 1;
//						upd = i;
//						break;
//					}
//				}
//				break;
//			}
//		}
//		if (upu >70 || upd<20 || (IS.right_y != YM&&upu + 20>IS.right_y)) return YM;
//		for (uint8 i = 0; i<YM - 4; ++i)
//		{
//			if (leftmap[i][end - 3] == 1)
//			{
//				for (uint8 k = i + 1; k < YM - 6; ++k)
//				{
//					if (basemap[k][end - 3] == 0)
//					{
//						downu = k - 1;
//						downd = i;
//						break;
//					}
//				}
//				break;
//			}
//		}
//		if (downu<upu || downd>upd || downu - downd <= upu - upd + 2) return YM;
//		IF.yhds = getMapYMin_Col2(IS.end_lm[0],0, basemap);
//		return getMapYMin_Col(0, deletemap);
//	}
//	else if ((IS.num_rm == 2 && IS.end_rm[0]>IS.end_rm[1] + 5 || IS.num_rm == 1 && IS.down_rm>2)// && IS.down_rm >= IS.down_lm
//		&& IS.left_y >= 65
//		&& IS.end_lm[0]<25
//		&& IS.end_lm[1]<25
//		&& IS.top_lm[0] >= 65
//		&& IS.top_lm[0]>(IS.top_rm[0]>IS.top_rm[1] ? IS.top_rm[0] : IS.top_rm[1])
//		)
//	{
//		uint8 end = IS.end_rm[1] < IS.end_rm[0] ? IS.end_rm[1] : IS.end_rm[0];
//		if (end >XX - 3) return YM;
//		uint8 upu = YM, upd = YM, downd = YM, downu = YM;
//		for (uint8 i = 0; i<YM - 4; ++i)
//		{
//			if (rightmap[i][end] == 1)
//			{
//				for (uint8 k = i + 1; k < YM - 6; ++k)
//				{
//					if (rightmap[k][end] == 0)
//					{
//						upu = k - 1;
//						upd = i;
//						break;
//					}
//				}
//				break;
//			}
//		}
//		if (upu >70 || upd<20 || (IS.left_y != YM&&upu + 20>IS.left_y)) return YM;
//		for (uint8 i = 0; i<YM - 4; ++i)
//		{
//			if (rightmap[i][end + 3] == 1)
//			{
//				for (uint8 k = i + 1; k < YM - 4; ++k)
//				{
//					if (rightmap[k][end + 3] == 0)
//					{
//						downu = k - 1;
//						downd = i;
//						break;
//					}
//				}
//				break;
//			}
//		}
//		if (downu < upu || downd > upd || downu - downd <= upu - upd + 2) return YM;
//		IF.yhds = getMapYMin_Col2(IS.end_rm[0], 0,basemap);
//		return getMapYMin_Col(XX, deletemap);
//	}
//	return YM;
//}


uint8 czAnnulus()
{
	static uint8 hold_flag = 0;
	static uint8 time = 0;
	if (hold_flag) goto GETDS;
	if (IS.right_y < 60//40
		&& IS.left_y == YM
		&&IS.num_lm == 1
		&& IS.top_lm[0]>75//�����70��Ϊ�˷�ֹԶ�����⣬����Ӧ�ø�79
		&& basemap[IS.top_rm[0]][XX] == 0
		)
	{
		if (IS.num_rm == 1 && IS.top_lm[0] == YY || IS.num_rm == 2)
		{
			uint8 upu = YM, upd = YM, downd = YM, downu = YM;
			RegionInfo info = { 0, XX, 0, 0, YY };
			uint8 yhmap[YM][XM] = { 0 };
			uint8 end; uint8(*map)[XM];
			if (IS.num_rm == 1 && IS.top_lm[0] == YY)
			{
				for (uint8 i = STEP1 + 1; i < YM; ++i)
				{
					if (basemap[i][IS.end_rm[0]] && leftmap[i][IS.end_rm[0]] != 1 && rightmap[i][IS.end_rm[0]] != 1)
					{
						getRegionInfo(IS.end_rm[0], i, yhmap, &info);
						break;
					}
					if (i>77) return YM;
				}
				end = info.left;
				map = yhmap;
			}
			else
			{
				end = IS.end_rm[1];
				map = rightmap;
			}
			for (uint8 i = IS.top_rm[0] + 2; i<YM - 4; ++i)
			{
				if (map[i][end] == 1)
				{
					for (uint8 k = i + 1; k < YM; ++k)
					{
						if (map[k][end] != 1)
						{
							upu = k - 1;
							upd = i;
							break;
						}
					}
					break;
				}
			}
			if (upu >78 || upd<YM / 2) return YM;
			for (uint8 i = IS.top_rm[0]; i<YM - 4; ++i)
			{
				if (map[i][end + 3] == 1)
				{
					for (uint8 k = i + 1; k < YM; ++k)
					{
						if (map[k][end + 3] != 1)
						{
							downu = k - 1;
							downd = i;
							break;
						}
						else if (k == YY)
						{
							downu = k;
							downd = i;
							break;
						}
					}
					break;
				}
			}
			if (downu < upu || downd > upd || downu - downd <= upu - upd + 2) return YM;
			hold_flag = __Right;
			time = 0;
			return upd;
		}
		else
			return YM;
	}
	if (IS.left_y < 60
		&& IS.right_y == YM
		&& IS.num_rm == 1
		&& IS.top_rm[0] > 75//�����70��Ϊ�˷�ֹԶ�����⣬����Ӧ�ø�79
		&& basemap[IS.top_lm[0]][0] == 0
		)
	{
		if (IS.num_lm == 1 && IS.top_rm[0] == YY || IS.num_lm == 2)
		{
			uint8 upu = YM, upd = YM, downd = YM, downu = YM;
			RegionInfo info = { 0, XX, 0, 0, YY };
			uint8 yhmap[YM][XM] = { 0 };
			uint8 end; uint8(*map)[XM];
			if (IS.num_lm == 1 && IS.top_rm[0] == YY)
			{
				for (uint8 i = STEP1 + 1; i < YM; ++i)
				{
					if (basemap[i][IS.end_lm[0]] && leftmap[i][IS.end_lm[0]] != 1 && rightmap[i][IS.end_lm[0]] != 1)
					{
						getRegionInfo(IS.end_lm[0], i, yhmap, &info);
						break;
					}
					if (i>77) return YM;
				}
				end = info.right;
				map = yhmap;
			}
			else
			{
				end = IS.end_lm[1];
				map = leftmap;
			}
			for (uint8 i = IS.top_lm[0] + 2; i < YM - 4; ++i)
			{
				if (map[i][end] == 1)
				{
					for (uint8 k = i + 1; k < YM; ++k)
					{
						if (map[k][end] != 1)
						{
							upu = k - 1;
							upd = i;
							break;
						}
					}
					break;
				}
			}
			if (upu > 78 || upd < YM / 2) return YM;
			for (uint8 i = IS.top_lm[0] + 2; i < YM - 4; ++i)
			{
				if (map[i][end - 3] == 1)
				{
					for (uint8 k = i + 1; k < YM; ++k)
					{
						if (map[k][end - 3] != 1)
						{
							downu = k - 1;
							downd = i;
							break;
						}
						else if (k == YY)
						{
							downu = k;
							downd = i;
							break;
						}
					}
					break;
				}
			}
			if (downu < upu || downd > upd || downu - downd <= upu - upd + 2) return YM;
			hold_flag = __Left;
			time = 0;
			return upd;
		}
		else
			return YM;
	}
	return YM;
GETDS:
	++time;
	if (time > 45 || IF.annulus)
	{
		time = 0;
		hold_flag = 0;
	}
	if (hold_flag == __Left)
	{
		if (IS.num_lm == 2)
			return getMapYMin_Col(IS.end_lm[1], leftmap);
		else if (IS.num_lm == 1)
			return getMapYMin_Col(IS.end_lm[0], leftmap);
	}
	else if (hold_flag == __Right)
	{
		if (IS.num_rm == 2)
			return getMapYMin_Col(IS.end_rm[1], rightmap);
		else if (IS.num_rm == 1)
			return getMapYMin_Col(IS.end_rm[0], rightmap);
	}

	return YM;
}
uint8 strJudge(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 map[][XM], uint8 sy1, uint8 sy2)
{
#define LIMT 2
	char x;
	if (y1 >= y2 || sy1<y1 || sy2>y2)return 0;
	float k = (float)(x2 - x1) / (y2 - y1);

	for (uint8 i = sy1; i < sy2; ++i)
	{
		x = (char)(x1 + k*(i - y1) + 0.5);
		if (map[i][x] == 2) continue;
		else
		{
			uint8 flag = 0;
			for (char j = -LIMT; j <= LIMT; ++j)
			{
				if (x + j<0 || x + j>XX) continue;
				if (map[i][x + j] == 2)
				{
					flag = 1;
					break;
				}
			}
			if (flag == 0)
			{
#ifdef BOOM3_QT_DEBUG
				for (uint8 i = y1; i < y2; ++i)
				{
					map[i][(int)(x1 + k*(i - y1))] = 4;
				}
#endif
				return 0;
			}
		}
	}
#ifdef BOOM3_QT_DEBUG
	for (uint8 i = y1; i < y2; ++i)
	{
		map[i][(int)(x1 + k*(i - y1))] = 4;
	}
#endif
	return 1;
}
uint8 X_WBW_Detect(char x1, char x2, uint8 y, uint8 map[][XM], uint8 flag)
{
	if (flag == goRight)
	for (char j = x1; j <= x2; ++j)
	{
		if (map[y][j] != 1)
		{
			for (char k = j + 1; k <= x2&&k<XM; ++k)
			{
				if (map[y][k] == 1)
				{
					for (char m = k + 1; m <= x2; ++m)
					{
						if (map[y][m] != 1)
							return 1;
					}
					break;
				}
			}
			break;
		}
	}
	if (flag == goLeft)
	for (char j = x1; j >= x2&&j >= 0; --j)
	{
		if (map[y][j] != 1)
		{
			for (char k = j - 1; k >= x2; --k)
			{
				if (map[y][k] == 1)
				{
					for (char m = k - 1; m >= x2; --m)
					{
						if (map[y][m] != 1)
							return 1;
					}
					break;
				}
			}
			break;
		}
	}
	return 0;
}
//uint8 X_BWB_Detect(uint8 x1,uint8 x2,uint8 y, uint8 map[][XM])
//{
//	for (char j = x1; j <= x2; ++j)
//	{
//		if (map[y][j] == 0)
//		{
//			for (char k = j + 1; k <= x2; ++k)
//			{
//				if (map[y][k] == 1)
//				{
//					for (char m = k + 1; m < x2; ++m)
//					{
//						if (map[y][m] == 0)  return 1;
//					}
//					break;
//				}
//			}
//			break;
//		}
//	}
//	return 0;
//}
uint8  horLineDect(uint8 y, uint8 dir, uint8 map[][XM])
{
	if (dir == goRight)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (map[y][j] != 2)
				return j;
		}
		return XM;
	}
	else if (dir == goLeft)
	{
		for (uint8 j = 0; j < XM; ++j)
		{
			if (map[y][XX - j] != 2)
				return j;
		}
	}
	return 0;
}
uint8 Obstacle()//ƽ�Ʊ��ߵķ����ܺã���������ԭͼ��ƽ�ƣ���Ϊԭͼ�ϵ�ƽ�Ʋ����ڱ任���ͼ�ϵ�ƽ�ƣ�ֱ�Ӹ��ؾ����ƫ������Դﵽ��Ч��
{
	static int16 obNum[67] = { 240, 237, 234, 230, 227, 223, 218, 214, 209, 204, 199, 194, 189, 184, 179, 173, 168, 163, 158, 153,
		148, 144, 139, 134, 130, 126, 121, 117, 113, 109, 105, 102, 98, 94, 91, 88, 84, 81, 78, 75,
		72, 69, 66, 63, 60, 57, 54, 52, 49, 46, 44, 41, 39, 36, 34, 32, 29, 27, 26, 24,
		22, 21, 20, 19, 18, 18, 18 };
	int left = 0;
	int right = XX;
	int downy = 0;
	char start_only_ones = 0;
	uint8 obstaclemap[YM][XM] = { 0 };
	uint16 insideNum = XM*YM - IS.bnum - IS.rnum - IS.lnum - IS.dnum;
	stopLineDetected = 0;
	for (uint8 i = 0; i <= YY; i++)//�ӵײ�������һ��ʶ�𣬻����ᴩ��������
	{
		uint8 myflag = 0;
		if (insideNum < 8) break;
		for (char m = XX - 1; m >= 0; --m)//����ʹ������ͼ��
		{
			if (leftmap[i][m] == 1)
			{
				left = m + 1;
				myflag = 1;
				break;
			}
		}
		if (myflag == 0)
		for (uint8 m = 0; m <= XX; ++m)
		{
			if (basemap[i][m] == 0)//�׵�
			{
				left = m;
				break;
			}
		}
		myflag = 0;
		for (char n = 1; n<XM; ++n)
		{
			if (rightmap[i][n] == 1)
			{
				right = n - 1;
				myflag = 1;
				break;
			}
		}
		if (myflag == 0)
		for (int n = XX; n>0; --n)
		{
			if (basemap[i][n] == 0)
			{
				right = n;
				break;
			}
		}


		for (uint8 j = left; j <= right; j++)
		{
			if (insidemap[i][j])//����insidemap�ĺڵ�
			{
				/* ****************������ʶ�����*******************/
				if (start_only_ones<1 && i<30)//����ͨ����ĸ�����һ��ͼ����1��
				{
					uint8 startmap[YM][XM] = { 0 };
					uint8 s_cnt = 0;
					for (uint8 m = i; m<i + 20; m++)
					{
						for (uint8 n = 0; n<XM; n++)
						{
							if (!startmap[m][n] && insidemap[m][n])
							{
								searchmap(n, m, startmap);
								s_cnt++;
							}
						}
					}
					uint8 snum = 0;
					if (s_cnt>4)
					{
						for (uint8 m = i; m < i + 20; ++m)
						{
							snum = 0;
							for (uint8 n = left; n < right; ++n)
							{
								if (startmap[m][n] != 1 && startmap[m][n + 1] == 1)
									++snum;
							}
							if (snum >4) break;
						}
					}
					if (snum >4)
					{
						stopLineDetected = 1;
						++start_only_ones;
						break;
					}
					else
						stopLineDetected = 0;
				}
				else if (start_only_ones == 0)
					stopLineDetected = 0;
				downy = i;
				int ob_num;
				numCnt = 0;
				if (obstaclemap[i][j] == 0)
					searchCountmap(j, i, obstaclemap);
				else continue;
				ob_num = numCnt;
				insideNum -= ob_num;
				//��ʼ���ϰ���
				//if (IF.obstacle != 0) ob_num = (int)(ob_num*1.1);
				if (downy <= 65 && ob_num > obNum[downy] * 0.8 - 10 && ob_num < obNum[downy] * 1.3 + 25)
				{  //������ϰ���֮ǰ�Ѿ�ʶ����ϰ���
					int max = 4;
					//  int direction=0;
					ob_direction = 0;
					/*		if (IF.obstacle == 1) ob_direction = 1;
					else if (IF.obstacle == 2) ob_direction = 2;*/
					if (ob_direction == 0)
					{
						for (uint8 k = downy + 5; k <= YY; k++)
						{
							if (left == 0 && ob_direction == 0)//������û����߽磬�Ҳ�û���й��ϰ�����ǰ��֮ǰ��
							{
								max = 10;
								uint8 obs_right = right;
								for (char m = XX; m>0; m--)
								{
									if (basemap[k][m] == 0)
									{
										obs_right = m;
										break;
									}
								}
								for (uint8 m = 1; m<max; m++)
								{
									if (obs_right  < m)
										break;
									if (obstaclemap[k][obs_right - m])
									{
										ob_direction = 2;
										if (k1[k] * m>23)
											ob_direction = 1;
										break;
									}
								}
							}
							else if (right == XX&&ob_direction == 0)//����ұ�û���ұ߽磬�Ҳ�û���й��ϰ�����ǰ��֮ǰ��
							{
								max = 10;
								uint8 obs_left = left;
								for (char m = 0; m<XX; m++)
								{
									if (basemap[k][m] == 0)
									{
										obs_left = m;
										break;
									}
								}
								for (uint8 m = 1; m < max; ++m)
								{
									if (obs_left + m > XX)
										break;
									if (obstaclemap[k][obs_left + m])
									{
										ob_direction = 1; //�ϰ������
										if (k1[k] * m>23)
											ob_direction = 2;
										break;
									}
								}
							}
							else if (ob_direction == 0 && left != 0 && right != XX)//���ұ߶��б߽磬�Ҳ�û���й��ϰ�����ǰ��֮ǰ��
							{
								max = 8;
								for (uint8 m = 1; m < max; ++m)
								{
									if (right - m  < 0 || left + m > XX)
										break;
									if (obstaclemap[k][right - m])
									{
										ob_direction = 2; //�ϰ����ұ�
										break;
									}
									if (obstaclemap[k][left + m])
									{
										ob_direction = 1; //�ϰ������
										break;
									}
								}
							}
						}
					}
					if (ob_direction == 0)
					{
						if (left == 0) ob_direction = 1;
						else if (right == XX) ob_direction = 2;
					}
					if (ob_direction == 1)   //֮ǰ�Ѿ��Ѿ��жϳ�ǰ�����ϰ������ϰ������
					{
						return 1;  //��ʾ���ϰ���
					}
					//�ϰ������ұ�
					else if (ob_direction == 2)
					{
						return 2;  //��ʾ���ϰ���
					}
				}
				break;//-----------------------------------�������
			}

		}

		/////////////////////////////////////////////

	}
	return 0;
}
uint8 RampUp()
{
	if (IS.num_lm + IS.num_rm != 1) return 0;
	uint8 top = YM;

	for (uint8 i = YY; i>35; i--)
	{
		uint8 myflag = 0;
		for (uint8 j = 0; j < XM; ++j)
		{
			if (basemap[i][j] == 0)
			{
				top = i;
				myflag = 1;
				break;
			}
		}
		if (myflag) break;
	}
	if (top == YM) return 0;
	uint8 max = 9;
	top = top - 4;
	if (IS.num_lm == 1)
	{
		uint8 min = getMapYMin_Col(XX, leftmap);
		if (top - min<7) return 0;
		if (IS.down_lm != 0 || IS.top_lm[0] != YY) return 0;
		for (uint8 i = 0; i < max; ++i)
		{
			if (i<min) break;
			for (uint8 j = 1; j < XX; ++j)
			{
				if (leftmap[top - i][j] == 2 && leftmap[top - i][j + 1] == 0)
				{
					for (uint8 k = j + 1; k < XX; ++k)
					{
						if (leftmap[top - i][k] == 2 && leftmap[top - i][k - 1] == 0)
						{
							if (k - j > rightline[top - i] - leftline[top - i] - 2)  //������
								return 0;
							if (k <= leftline[top - i] || j >= rightline[top - i])
								return 0;
							break;
						}
						if (k == XX - 1)
							return 0;
					}
					break;
				}
				if (j == XX - 1)
					return 0;
			}
		}
	}
	else if (IS.num_rm == 1)
	{
		if (IS.down_rm != 0 || IS.top_rm[0] != YY) return 0;
		uint8 min = getMapYMin_Col(0, rightmap);
		if (top - min<7) return 0;
		for (uint8 i = 0; i < max; ++i)
		{
			if (i<min) break;
			for (uint8 j = 1; j < XX; ++j)
			{
				if (rightmap[top - i][j] == 2 && rightmap[top - i][j + 1] == 0)
				{
					for (uint8 k = j + 1; k < XX; ++k)
					{
						if (rightmap[top - i][k] == 2 && rightmap[top - i][k - 1] == 0)
						{
							if (k - j > rightline[top - i] - leftline[top - i] - 2)
								return 0;
							if (k <= leftline[top - i] || j >= rightline[top - i])
								return 0;
							break;
						}
						if (k == XX - 1)
							return 0;
					}
					break;
				}
				if (j == XX - 1)
					return 0;
			}
		}
	}
	else return 0;
	return 1;
}
uint8 RampUp2()
{
	if (IS.num_lm == 0 || IS.num_rm == 0) return 0;
	// for(uint8 i=0;i<top;)
}
uint8 bump()
{
#define NN 5
	//   int m=0,s2=0;
	//   uint8 myflag=0;
	//    for(uint8 i=0;i<NN;++i)
	//    {
	//        if(Hgyro1[i]<-1800)
	//        {
	//            myflag=1;
	//            break;
	//        }
	//    }
	//    if(myflag==0) return 0;
	//    for(uint8 i=0;i<NN;++i)
	//    {
	//         m+=Hgyro1[i];
	//    }
	//    m=m/NN;
	//    for(uint8 i=0;i<NN;++i)
	//    {
	//         s2+=(Hgyro1[i]-m)*(Hgyro1[i]-m);
	//    }
	//    s2=s2/NN/1000;
	if (Hgyro1[0]<-3061 || Hgyro1[0]<-2300 && Hgyro1[0] + Hgyro1[1]<-4000)
		return 1;
	else
		return 0;
}
uint8 goBump()//����·�洦��
{
	static uint8 flag = 0;
	static uint8 time = 0;
	static uint8 rampTime = 0;
#ifdef BOOM3_QT_DEBUG
	flag = 0;
#endif
	if (IF.ramp) rampTime = 30;
	if (bump() && flag == 0)
		flag = 1;
	if (flag == 1)
	{
		++time;
	}
	if (time>30 || IF.ramp)
	{
		flag = 0;
		time = 0;
	}
	if (rampTime)
	{
		--rampTime;
		flag = 0;
		time = 0;
	}
	return flag;
}