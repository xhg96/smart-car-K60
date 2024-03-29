#include "deal_img.h"
#include "include.h"

float xx[80]={1.2913,1.2898,1.2906,1.2937,1.3289,1.3361,1.3450,1.3556,1.3677,1.3813,1.3961,1.4121,1.4292,1.4473,1.4663,1.4861,1.5066,1.5277,1.5495,1.5418,1.5646,1.5879,1.6115,1.6356,1.6600,1.6847,1.7098,1.7352,1.7610,1.7871,1.8136,1.8404,1.8677,1.8955,1.9237,1.9525,1.9819,2.0120,2.0428,2.0745,2.1070,2.1406,2.1753,2.2111,2.2483,2.2869,2.3271,2.3690,2.4127,2.4584,2.5062,2.5562,2.6088,2.6639,2.7219,2.7829,2.8470,2.9145,2.9856,3.0605,3.1394,3.2225,3.3101,3.4025,3.4998,3.6023,3.7102,3.8239,3.9437,4.0697,4.2023,4.3417,4.4883,4.6424,4.8043,4.9743,5.1528,5.3400,5.5363,5.7421};
float yy[80]={0,0.56058,1.01680,1.52182,2.07163,2.66242,3.29057,3.95268,4.64552,5.36609,6.11156,6.87932,7.66693,8.47218,9.29305,10.12769,10.97449,11.83202,12.69903,13.57450,14.45759,15.34766,16.24428,17.14720,18.05637,18.97196,19.89431,20.82398,21.76171,22.70846,23.66538,24.63379,25.61526,26.61152,27.62450,28.65635,29.70940,30.78618,31.88943,33.02208,34.18726,35.38829,36.62870,37.91220,39.24273,40.62441,42.06154,43.55865,45.12045,46.75186,48.45798,50.24413,52.11580,54.07872,56.13878,58.30207,60.57492,62.96381,65.47543,68.11669,70.89468,73.81669,76.89021,80.12292,83.52272,87.09769,90.85611,94.80646,98.95743,103.31788,107.89690,112.70376,117.74792,123.03907,128.58708,134.40199,140.49410,146.87384,153.55190,160.53912};
int obnum[70]={402,399,393,386,379,367,363,356,346,340,331,325,314,307,299,299,290,281,276,267,263,255,253,244,240,231,227,223,218,212,205,195,187,178,170,163,157,153,152,146,141,138,131,124,120,119,116,110,105,99,96,90,88,86,82,79,77,72,71,66,62,60,56,51,51,51,48,44,44,39};
uint8 basemap[Y][X];	//白点置0，黑点置2
uint8 leftmap[Y][X];		// 1黑 2 边线 0白
uint8 bob_flag=0;
uint8 rightmap[Y][X];
uint8 obstaclemap[Y][X];
uint8 leftline[Y],rightline[Y];
uint8 speedlline[Y],speedrline[Y];//用于计算other_top
uint8 baseline[Y][X];
DIRECTION ring_direction=left;
float direction_save=0;
uint8 midline[Y];
float ramp_delay=10;
float sl_k=12;
float sl_b=6;
float lpp=0.54;
float rpp=0.54;
float flpp=1.21;
float frpp=1.21;
extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 ring_flag;
extern uint8 ramp_flag;
extern float runTime;
//赋值为100表示未检测到
uint8 left_x=100;
uint8 left_y=100;
uint8 right_x=100;
uint8 right_y=100;

//Site_t site_numSign={92, 55};        
//Site_t site_numValue={100, 55};
//Site_t site_obdySign={85-40, 55};        
//Site_t site_obdyValue={93-40, 55};
void standard()                            //标准线，根据摄像头高度角度调整
{
   float x;
//    for(uint8 i=0;i<Y;++i)
//   {
//    for(uint8 j=0;j<X;++j)
//    {
//      baseline[i][j]=0;
//    }
//   }
    for(uint8 i = 0; i < Y; ++ i)
    {
     // midline[i] = X / 2;
      x=-25* i / 158+ 16;
      leftline[i] = (uint8)(19 - x);
      rightline[i] = (uint8)(20 + x);
//      lline[i] = (uint8)(19 - x);
//      rline[i] = (uint8)(20 + x);
      baseline[i][leftline[i]] = 1;
      baseline[i][rightline[i]] = 1;
      x=-sl_k* i / 160+ sl_b;
      speedlline[i] = (uint8)(19 - x);
      speedrline[i] = (uint8)(20 + x);
    }
}


void map_clean()
{
  for(uint8 i = 0; i < Y; ++ i)
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      basemap[i][j] = 1;
      leftmap[i][j]=0;
      rightmap[i][j]=0;
      obstaclemap[i][j]=0;
    }
  }
}

/*
用递归压缩图像，白点置0，黑点置2
最终src[]里只能有0和2
*/
void searchimg(uint8 x, uint8 y, uint8 src[][X], int threshold)//给basemap[]赋值
{
  if(!judge(x, y, threshold))
  {
    src[y][x] = 0;	
    if(x > 0 && src[y][x - 1] == 1)		//四周有为1的点则继续搜索
      searchimg(x - 1, y, src, threshold);
    if(x < XX && src[y][x + 1] == 1)
      searchimg(x + 1, y, src, threshold);
    if(y > 0 && src[y - 1][x] == 1)
      searchimg(x, y - 1, src, threshold);
    if(y < YY && src[y + 1][x] == 1)
      searchimg(x, y + 1, src, threshold);
  }else
    src[y][x] = 2;
}

/*
将scr2映射到scr
scr[]中最终仅有1和2
2->1 黑, 0->2	白
*/
void searchmap(uint8 x, uint8 y, uint8 src[][X], uint8 src2[][X])//用basemap[]给左右图赋值
{
  if(src2[y][x])
  {
    src[y][x] = 1;
    if(x > 0 && src[y][x - 1] == 0)
      searchmap(x - 1, y, src, src2);
    if(x < XX && src[y][x + 1] == 0)
      searchmap(x + 1, y, src, src2);
    if(y > 0 && src[y - 1][x] == 0)
      searchmap(x, y - 1, src, src2);
    if(y < YY && src[y + 1][x] == 0)
      searchmap(x, y + 1, src, src2);
  }else
    src[y][x] = 2;
}
//void searchmap_nor(uint8 x, uint8 y, uint8 src[][X], uint8 src2[][X])
//{
//  if(src2[y][x])
//  {
//    src[y][x] = 1;
//    if(x > 0 && src[y][x - 1] == 0)
//      searchmap(x - 1, y, src, src2);
////    if(x < XX && src[y][x + 1] == 0)
////      searchmap(x + 1, y, src, src2);
//    if(y > 0 && src[y - 1][x] == 0)
//      searchmap(x, y - 1, src, src2);
//    if(y < YY && src[y + 1][x] == 0)
//      searchmap(x, y + 1, src, src2);
//  }else
//    src[y][x] = 2;
//}
uint8 judge(uint8 x, uint8 y, int threshold)    //判黑色
{
  for(uint8 i = 1; i <= VERTICAL; ++ i)               //3行并一行，240变80
  {
    if(imgbuff[(CAMERA_H - y * VERTICAL - i) * X + x] >= threshold)
      return 1;
  }
  return 0;
}


void deleteline(int deletenum)
{
 // uint8 deletenum = 0;  //这个值看情况调节
  uint8 down = 0;
  uint8 up = 0;
  for(uint8 i = 0; i < Y; ++ i)		//从底部向上，从左往右扫，遇白 黑 白，对应Y轴的值给up   
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[i][j] != 1)	//x遇白色
      {
        for(uint8 k = j + 1; k < X; ++ k)	//右边
        {
          if(leftmap[i][k] == 1)	//为黑色
          {
            for(uint8 m = k + 1; m < X; ++ m)//再右
            {
              if(leftmap[i][m] != 1)		//白色x
              {
                if(i != up + 1)  //若有間隔則更新down
                  down = i;
                up = i;
                break;
              }
            }
            break;
          }
        }
        break;
      }
    }
  }
  
  if(up != 0 && up - down > deletenum)        //行數超過deletenum 才會刪綫
  {
    for(uint8 i = down; i < up - deletenum&&i<Y; ++ i)//回環并不完全刪完
    {
      for(uint8 j = 0; j < X; ++ j)
      {
        if(leftmap[i][j] == 2)
          leftmap[i][j] = 0;
        if(leftmap[i][j] == 1)
          break;
      }
    }
  }
  /*****右綫再來一次****/
  down = 0;
  up = 0;
  for(uint8 i = 0; i < Y; ++ i)
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(rightmap[i][j] != 1)
      {
        for(uint8 k = j + 1; k < X; ++ k)
        {
          if(rightmap[i][k] == 1)
          {
            for(uint8 m = k + 1; m < X; ++ m)
            {
              if(rightmap[i][m] != 1)
              {
                if(i != up + 1)
                  down = i;
                up = i;
                break;
              }
            }
            break;
          }
        }
        break;
      }
    }
  }
  if(up != 0 && up - down > deletenum)        
  {
    for(uint8 i = down; i < up - deletenum&&i<Y; ++ i)
    {
      for(uint8 j = 0; j < X; ++ j)
      {
        if(rightmap[i][XX - j] == 2)
          rightmap[i][XX - j] = 0;
        if(rightmap[i][XX - j] == 1)
          break;
      }
    }
  }
}
uint8 ramp()
{
  float max = 20;
  for(uint8 i = 1; i < 12; ++ i)		//从上到下，从右到左
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[YY - i][XX - j] == 2)	//左图遇边线
      {
        for(uint8 k = XX - j + 1; k < X; ++ k)  //k记录边线x坐标，从x开始向右扫
        {
          if(rightmap[YY - i][k] == 2)		//右图遇到边线
          {
            if(XX-j>rightline[YY-i] || k<leftline[YY-i])  //左边线越过右边线或右边线越过左边线
              return 0;
            break;
          }
          if(k == XX)
            return 0;
        }
        break;
      }
      if(j == XX)
        return 0;
    }
  }
  
  for(uint8 i = 13; i < max; ++ i)//13行到20行
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[YY - i][XX - j] == 2)
      {
        for(uint8 k = XX - j + 1; k < X; ++ k)
        {
          if(rightmap[YY - i][k] == 2)
          {
            if(k - (XX - j) < rightline[YY - i] - leftline[YY - i] + (max - i) * 4 / max + 2)  //调整我
              return 0;
            break;
          }
          if(k == XX)
            return 0;
        }
        break;
      }
      if(j == XX)
        return 0;
    }
  }
  return 1;
}

void rampline()
{
  for(uint8 i = 30; i < Y; ++ i)		//30行以上的图像，清除边线
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[i][j] == 2)	
        leftmap[i][j] = 0;
      if(rightmap[i][j] == 2)
        rightmap[i][j] = 0;
    }
  }
  for(uint8 i = 0; i < 30; ++ i)
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[i][j]==2)
      {
        for(uint8 k=j+1;k<X;++k)	//30行以下， 左边线以右清0  为什么?
        {
          leftmap[i][k]=0;
        }
      }
      if(rightmap[i][XX-j]==2)
      {
         for(uint8 k=j+1;k<X;++k)
        {
          rightmap[i][XX-k]=0;
        }
      }
    }
  }
}
uint8 go_ramp()
{
 static uint8 my_ramp_flag=0;
 static uint8 delay_time=0;
 static uint8 xiaodou=0;
  if(ramp()&&my_ramp_flag==0)      //上坡检测到
  {
    my_ramp_flag=1;
    xiaodou=0;
  }
  
  
  if(my_ramp_flag==1)
  {
    xiaodou++;
    if(xiaodou>5)
    {
      if(!ramp())              //在坡上检测不到
      {
        my_ramp_flag=2;
      }
    }
  }
  
  if(my_ramp_flag==2)
  {
    delay_time++;
    if(ramp())          //下坡再次检测到
    {
      my_ramp_flag=3;
      delay_time=0;
    }
    else if(delay_time>50)      //若误判，则最多误判时间为50
    {
      my_ramp_flag=0;
      delay_time=0;
    }
  }
  
  if(my_ramp_flag==3||my_ramp_flag==4)
  {
    delay_time++;
    if(delay_time>ramp_delay)         //延时，恢复初始化
      my_ramp_flag=4;
    if(delay_time>50)         //延时，恢复初始化
    {
      my_ramp_flag=0;
      delay_time=0;
    }
  }
  
  if(my_ramp_flag==1||my_ramp_flag==2) rampline();  //坡上只使用底部图像控制方向。
  
  return my_ramp_flag;
  //效果，返回值为1、2、3在坡道，返回0时不在
}
uint8 crossroadforleft()
{
  left_x=100;
  left_y=100;
  uint8 high = 3;
  int upd;//记录角点下边沿的Y位置
  int upu;//记录角点上边沿的Y位置
  int up;//记录角点的X位置
  uint8 flag = 0;
  for(uint8 i = 0; i < X; ++ i)         
  {
    for(uint8 j = 0; j < Y; ++ j)//从下到上，从右到左
    {
      if(leftmap[j][XX - i] == 1)//遇到黑色区域就记下底部，接着扫上面边沿
      {
        upd = j;//记录角点下边沿的Y位置
        up = XX - i;//记录角点的X位置
        for(uint8 k = j + 1; k < Y; ++ k)//接着继续向上扫角点的上边沿
        {
          if(leftmap[k][up] != 1)//遇到非黑色的区域,且在75行以下时
          {
            upu = k - 1;//记录下边沿的位置
            if(upu<75)
            {
              flag = 1;//表示可以进入下一步了
            }
            break;
          } 
        }
        break;
      }
    }
    if(flag)//如果没扫到合适的就继续扫
      break;
  }
  
  int downu;//记录角底上边沿的Y位置
  int downd;//记录角底下边沿的Y位置
  uint8 down;
  if(flag)
  {
    down = up - high;//找到角底 
    uint8 upmax =15;//角点的最大宽度，Y轴上的     
    if(upu >= 75 || up <= high  || upu - upd >= upmax)//如果角点的特征不符合就退出程序|| upd <= 5
      return 0;
    
    
    
     for(uint8 i = upu; i < Y; ++ i)     //对直角 ，从角点顶部向上
    {
      if(rightmap[i][up] == 1)
      {
        uint8 j;
        for(j=0;j<X;j++)
        {
          if(rightmap[YY][j]==0)	//右图顶部扫
            break;
        }
        if(j>=X) return 0;	//右图顶部全黑色则退出
        else break;
      }
      if(i - upu == (uint8)((Y - upu) / 1.2))	//i扫到一定值放弃扫描
        break;
    }
    
    
    
    flag = 0;//标志位清0
    for(uint8 i = 0; i < Y; ++ i)//在角底从下往上扫
    {
      if(leftmap[i][down] == 1)//遇到黑色区域
      {
        downd = i;//记录角底下边沿的Y位置
        downu = YY;//记录角底上边沿的Y位置
        flag = 1;
        for(uint8 j = i; j < Y; ++ j)//继续向上扫
        {
          if(leftmap[j][down] != 1)//遇到非黑色区域
          {
            downu = j - 1;//刷新角底上边沿的Y位置
            break;
          }
        }
        break;
      }
    }
  }else
    return 0;
  
  uint8 downmax = 40;//底边的最大长度
  int pointnum = 0;
  int mynum = 0;
  if(flag && downu - downd < downmax)//底边找到且长度符合
  {
    left_x=up;
    left_y=upu;
    for(uint8 i = down; i <= up; ++ i)
    {
      for(uint8 j = downd; j <= downu; ++ j)//记录宽为角顶到角底的距离，长为角底的矩形内的黑点数
      {
        if(leftmap[j][i] == 1)
          ++ pointnum;
      }
    }
    mynum = (downu - downd) * (high + 1) / 2;//算三角形面积
    mynum += (downu - downd)*1.8;//加上底边的长度的1.5
    if(mynum >= pointnum)//初步条件成立,进人更严格的判定
    {
      for(uint8 i = 0; i < up; ++ i)
      {
        for(uint8 j = 0; j < Y; ++ j)//从下到上，从左到角顶
        {
          if(leftmap[j][i] == 1)//遇到黑色区域
          {
            for(uint8 k = j + 1; k < Y; ++ k)//继续向上找
            {
              if(leftmap[k][i] == 2)//遇到边界
              {
               // if(upd - 10 > k)//&& upd > 10 
                 // break;
                for(uint8 m = k; m < Y; ++ m)//继续向上找
                {
                  if(leftmap[m][i] == 0)//遇到白色区域
                  {
                    for(uint8 n = m + 1; n < Y; ++ n)//继续向上找
                    {
                      if(leftmap[n][i] == 2)//在左图遇到边界
                      {
                        for(uint8 a = n; a < Y; ++ a)//继续向上找
                        {
                          if(leftmap[a][i] != 2)
                            break;
                          leftmap[a][i] = 0;//删除左图上边界
                        }
                        break;
                      }
                      if(rightmap[n][i] == 2)//在右图遇到边界
                      {
                        for(uint8 a = n; a < Y; ++ a)//继续向上找
                        {
                          if(rightmap[a][i] != 2)
                            break;
                          rightmap[a][i] = 0;//删除右图上边界
                        }
                        break;
                      }
                    }
                    break;
                  }
                  leftmap[m][i] = 0;//删除下边界
                }
                break;
              }
            }
            break;
          }
        }
      }
      
      if(upu < Y / 2&&!ring_flag)//当角顶的上部在屏幕的一半以下时的
      {
        for(uint8 i = 1; i < (Y - upu) * 2 / 3; ++ i)
        {
          if(i + upu > YY)//防止越界
            break;
          if(basemap[i + upu][up])	//黑
          {
            if(leftmap[i + upu][up] != 1 && rightmap[i + upu][up] != 1 && basemap[i + upu][up])//右，左图为边线或白色，原图黑
              searchmap(up, i + upu, leftmap, basemap);
            break;
          }
        }
      }
      //标记从右到左的扫到的边界
      for(uint8 i = 0; i < Y; ++ i)//贴着右边沿从下到上扫   //这个删线感觉复杂而且效果不好，先注释掉试试
      {
        if(leftmap[i][XX] != 1)//遇到非黑色区域
        {
          for(uint8 j = 1; j < X; ++ j)//在非黑色那一行从右向左扫
          {
            if(leftmap[i][XX - j] == 2)//遇到边界就标记
            {
              leftmap[i][XX - j] = 3;
              break;
            }
          }
        }
      }
      //标记从下到上扫到的右半边界
      for(uint8 i = X / 2; i < X; ++ i)//在最底部行，从中间往右扫
      {
        if(leftmap[0][i] != 1)//遇到非黑色区域
        {
          for(uint8 j = 1; j < Y; ++ j)//从非黑色所在列从下向上扫
          {
            if(leftmap[j][i] == 2 || leftmap[j][i] == 3)//遇到边界就标记
            {
              leftmap[j][i] = 3;
              break;
            }
          }
        }
      }
      
      for(uint8 i = 0; i < Y; ++ i)//删除未标记的
      {
        for(uint8 j = 0; j < X; ++ j)
        {
          if(leftmap[i][j] == 2)
            leftmap[i][j] = 0;
          if(leftmap[i][j] == 3)
            leftmap[i][j] = 2;
        }
      }
      return 1;
    }
  }else
    return 0;
  
  return 0;
}


uint8 crossroadforright()
{
  right_x=100;
  right_y=100;
  uint8 high = 3;
  int upd;
  int upu;
  int up;
  uint8 flag = 0;
  for(uint8 i = 0; i < X; ++ i)
  {
    for(uint8 j = 0; j < Y; ++ j)
    {
      if(rightmap[j][i] == 1)
      {
        upd = j;
        up = i;
        for(uint8 k = j + 1; k < Y; ++ k)
        {
          if(rightmap[k][i] != 1)
          {
            upu = k - 1;
            if(upu<75)
            {
              flag = 1;
            }
            break;
          }
        }
        break;
      }
    }
    if(flag)
      break;
  }
  
  int downu;
  int downd;
  uint8 down;
  if(flag)
  {
    down = up + high;
    uint8 upmax = 15;
    if(upu >= 75|| up >= X - high  || upu - upd >= upmax)//|| upd <= 5
      return 0;
    
     for(uint8 i = upu; i < Y; ++ i)     //对直角
    {
      if(leftmap[i][up] == 1)
      {
        uint8 j;
        for(j=0;j<X;j++)
        {
          if(leftmap[YY][j]==0)
            break;
        } 
        if(j>=X) return 0;
        else break;
      }
      if(i - upu == (uint8)((Y - upu) / 1.2))
        break;
    }
   
    
    flag = 0;
    for(uint8 i = 0; i < Y; ++ i)
    {
      if(rightmap[i][down] == 1)
      {
        downd = i;
        downu = YY;
        flag = 1;
        for(uint8 j = i; j < Y; ++ j)
        {
          if(rightmap[j][down] != 1)
          {
            downu = j - 1;
            break;
          }
        }
        break;
      }
    }
  }else
    return 0;
  
  uint8 downmax = 40;
  int pointnum = 0;
  int mynum = 0;
  if(flag && downu - downd < downmax)
  {
    for(uint8 i = up; i <= down; ++ i)
    {
      for(uint8 j = downd; j <= downu; ++ j)
      {
        if(rightmap[j][i] == 1)
          ++ pointnum;
      }
    }
    mynum = (downu - downd) * (high + 1) / 2;
    mynum += (downu - downd) *1.8;
    if(mynum >= pointnum)//初步条件成立,进人更严格的判定
    {    
      right_x=up;
      right_y=upu;
      for(int i = XX; i >= up; -- i)
      {
        for(uint8 j = 0; j < Y; ++ j)
        {
          if(rightmap[j][i] == 1)
          {
            for(uint8 k = j + 1; k < Y; ++ k)
            {
              if(rightmap[k][i] == 2)
              {
             //   if( upd - 10 > k)//upd > 10 &&           就是这东西导致新赛道十字出问题，搞掉了妈的
               //   break;
                for(uint8 m = k; m < Y; ++ m)
                {
                  if(rightmap[m][i] == 0)
                  {
                    for(uint8 n = m + 1; n < Y; ++ n)
                    {
                      if(rightmap[n][i] == 2)
                      {
                        for(uint8 a = n; a < Y; ++ a)
                        {
                          if(rightmap[a][i] != 2)
                            break;
                          rightmap[a][i] = 0;
                        }
                        break;
                      }
                      if(leftmap[n][i] == 2)
                      {
                        for(uint8 a = n; a < Y; ++ a)
                        {
                          if(leftmap[a][i] != 2)
                            break;
                          leftmap[a][i] = 0;
                        }
                        break;
                      }
                    }
                    break;
                  }
                  rightmap[m][i] = 0;//删除的是下边界
                }
                break;
              }
            }
            break;
          }
        }
      } 
      
      if(upu <  Y / 2&&!ring_flag)
      {
        for(uint8 i = 1; i < Y / 2; ++ i)
        {
          if(i + upu > YY)
            break;
          if(basemap[i + upu][up])
          {
            if(leftmap[i + upu][up] != 1 && rightmap[i + upu][up] != 1 && basemap[i + upu][up])
              searchmap(up, i + upu, rightmap, basemap);
            break;
          }
        }
      }
      
      for(uint8 i = 0; i < Y; ++ i)//这个删线感觉复杂而且效果不好，先注释掉试试
      {
        if(rightmap[i][0] != 1)
        {
          for(uint8 j = 1; j < X; ++ j)
          {
            if(rightmap[i][j] == 2)
            {
              rightmap[i][j] = 3;
              break;
            }
          }
        }
      }
      for(uint8 i = 0; i < X / 2; ++ i) //修改？
      {
        if(rightmap[0][i] != 1)
        {
          for(uint8 j = 1; j < Y; ++ j)
          {
            if(rightmap[j][i] == 2 || rightmap[j][i] == 3)
            {
              rightmap[j][i] = 3;
              break;
            }
          }
        }
      }
      for(uint8 i = 0; i < Y; ++ i)
      {
        for(uint8 j = 0; j < X; ++ j)
        {
          if(rightmap[i][j] == 2)
            rightmap[i][j] = 0;
          if(rightmap[i][j] == 3)
            rightmap[i][j] = 2;
        }
      }
      return 1;
    }
  }else
    return 0;
  
  return 0;
}

void go_ring()
{
  static uint8 count=0;
  static uint8 myflag=0;
  static int time=0;
  if(time>0) time--;
  else ring_flag=0;
//  ring_direction=right;
  if(ring_flag==0)
  {
    myflag=0;
    time=135;
    ring_flag=ring_judge();
    if(ring_flag)
    {
      ring_direction=(DIRECTION)( ( (int)(direction_save)>>count )&0x1 ) ;
      
      count++;
    }
  }
  if(ring_flag==1)
  { 
    if( (right_y<35||left_y<35||right_y==100) && ring_direction==left)
      addin_left();
    else if( (right_y<35||left_y<35||right_y==100) && ring_direction==right)
      addin_right();
    uint8 j;
    for(j=0;j<X;j++)
    {
      if(basemap[0][j]) break;	//底部图像全白
    }
    if(j>=X) ring_flag=2;
  }
  else if(ring_flag==2)
  {
    if(ring_direction==left)
      addin_left();
    else
      addin_right();
    uint8 nn=0;
    for(uint8 i=0;i<5;i++)		//底部5行都有黑色
    {
      uint8 j;
      for(j=0;j<X;j++)
      {
        if(basemap[i][j]) break;
      }
      if(j<X) nn++;
    }
    if(nn==5) ring_flag=3;
  }
  else if(ring_flag==3)			//在圆环内时
  {    
    uint8 down=0;
    uint8 up=0;
    uint8 whiteline=Y;  
    if(ring_direction==left)//走左边
    {
      addout_left();
      if(left_y<50&&left_x<30)		//角点处在这个位置就出圆环
      {
        ring_flag=4;
        return;
      }
      for(uint8 i=0;i<Y;i++) //从下往上，从左往右，记录右图离底部 最近的一条全白行
      {
        uint8 j;
        for(j=0;j<X;j++)
        {
          if(rightmap[i][j])
            break;
        }
        if(j>=X) 
        {
          whiteline=i;
          break;
        }
      }
      for(uint8 i=whiteline;i<Y;i++)	//从全白行往上扫
      {
        for(uint8 j=0;j<X;j++)
        {
          if(rightmap[i][j]==0)	//右图遇白点
          {
            for(uint8 k=j+1;k<X;k++)
            {
              if(rightmap[i][k]==1)	//再黑
              {
                for(uint8 m=k+1;m<X;m++)
                {
                  if(rightmap[i][m]==0)	//再白
                  {
                    if(i!=up+1)		//更新 down 和 up
                      down=i;
                    up=i;
                    break;
                  }
                }
                break;
              }
            }
            break;
          }
        }
      } 
    }
    else			//走右边
    {
      addout_right();
      if(left_y<50&&left_x>10)
      {
        ring_flag=4;
        return;
      }
      for(uint8 i=0;i<Y;i++)
      {
        uint8 j;
        for(j=0;j<X;j++)
        {
          if(leftmap[i][j]) 		
            break;
        }
        if(j>=X) 
        {
          whiteline=i;
          break;
        }
      }
      for(uint8 i=whiteline;i<Y;i++)
      {
        for(uint8 j=0;j<X;j++)
        {
          if(leftmap[i][j]==0)
          {
            for(uint8 k=j+1;k<X;k++)
            {
              if(leftmap[i][k]==1)
              {
                for(uint8 m=k+1;m<X;m++)
                {
                  if(leftmap[i][m]==0)
                  {
                    if(i!=up+1)
                      down=i;
                    up=i;
                    break;
                  }
                }
                break;
              }
            }
            break;
          }
        }
      } 
    }//走右边
    if(up-down>2&&down<75&&up-down<40)    //遇条件满足出环
      ring_flag=4;
  }
  else if(ring_flag==4)
  {
    if(ring_direction==left)
      addout_left();
    else
      addout_right();
    if(myflag==0)
    {
      time=40;
      myflag=1;
    }
  }
}
uint8 ring_judge()//目前尝试m-k>8.n>2
{
 if( (right_x==100||right_y==100) && (left_y==100||left_x==100) )  return 0; 	//左右角点都无角点
 int lline[Y],rline[Y];//用于圆环判别
 uint8 mid_y;
 if(right_x==100||right_y==100)   //丢失右脚  
 {
   mid_y=left_y;
   uint8 bottom=100;
   uint8 site_x=100;
   uint8 site_y=100;
   for( uint8 j=0;j<X;j++)
   {
     if(leftmap[0][XX-j]) 
     {
        bottom=XX-j;//找到底部左图边界
        break;
     }
   }
   if(bottom<19)			//对边界位置限定
   {
      for(uint8 j=0;j<X;j++)
      {
        for(uint8 i=0;i<Y;i++)
        {
          if(leftmap[i][XX-j])	//再找除底部外另一条边线点位置
          {
            site_x=XX-j;
            site_y=i;
            break;
          }
        }
        if(site_x!=100) break;	//找到即跳出
      }
      int r_b=19-bottom;							
      float r_k=(float)(site_x-bottom)/site_y;		//斜率
      for(uint8 i = 0; i < Y; ++ i)
      {
        float x=-r_k* i+ r_b;		//根据斜率算对应Y位置X
        lline[i] = (uint8)(19 - x);		//左边按斜率补线
        rline[i] =  rightline[i];		//右边补标准线
      }
   }
   else
   {
      for(uint8 i = 0; i < Y; ++ i)    //边线越过20行(图的中间)则补标准线
      {
        lline[i] = leftline[i];
        rline[i] = rightline[i];
      }
   }
 }
 else if(left_y==100||left_x==100)  //丢失左脚  
 {  
   mid_y=right_y;
   uint8 bottom=100;
   uint8 site_x=100;
   uint8 site_y=100;
   for( uint8 j=0;j<X;j++)
   {
     if(rightmap[0][j]) 
     {
        bottom=j;//找到底部右图边界
        break;
     }
   }
   if(bottom>20&&bottom<X)
   {
      for(uint8 j=0;j<X;j++)
      {
        for(uint8 i=0;i<Y;i++)
        {
          if(rightmap[i][j])
          {
            site_x=j;
            site_y=i;//找到最左边的点
            break;
          }
        }
        if(site_x!=100) break;
      }
      int r_b=bottom-20;
      float r_k=(float)(bottom-site_x)/site_y;
      for(uint8 i = 0; i < Y; ++ i)//两点连成一条直线rline，左边用标准线
      {
        float x=-r_k* i+ r_b;
        lline[i] = leftline[i];
        rline[i] = (uint8)(20 + x);
      }
   }
   else
   {
      for(uint8 i = 0; i < Y; ++ i)
      {
        lline[i] = leftline[i];
        rline[i] = rightline[i];
      }
   }   
 }
 else               						//左右角都在
 {
    mid_y=(left_y+right_y)/2;
    for(uint8 i = 0; i < Y; ++ i)
    {
      lline[i] = leftline[i];
      rline[i] = rightline[i];
    }   
 }
 uint8 whiteline=YY;
 for(uint8 i=mid_y;i<Y;i++)	//从角点顶部向上扫，找全白的线
 {
   uint8 j;
  for(j=0;j<X;j++)
  {
    if(basemap[i][j]) break;
  }
  if(j>=X)	//未遇到黑点，则更新全白线
  {
    whiteline=i;
    break;
  }
 }
 if(whiteline>=73) return 0;    //73行以上不管
 uint8 down=YY;
 uint8 downd=0;
 for(uint8 i=whiteline;i<Y;i++)	//从全白线向上扫，从左到右找黑点，即圆的下边缘
 {
  uint8 j;
  for(j=0;j<X;j++)
  {
    if(basemap[i][j])
      break;
  }
  if(j<X) 
  {
    down=i;	//记录黑点位置
    downd=j;
    break;
  }
 }
 if(down>=75 ) return 0;		//黑点太高，不管
 if(left_x==100 && downd>leftline[down] && right_x<rightline[right_y]-5) return 0;		//左丢角，黑点必须在标准线内，角点与标准线较接近
 if(right_x==100 && downd<rightline[down] && left_x>leftline[left_y]+5) return 0;

 uint8 flag=0;

  uint8 n=0;
  for(uint8 k=down;k<Y;k++)
  {
    if(n>4) //n>4
    {
      flag=1;break;
    }
    if( lline[k]>rline[k] || lline[k]<0 || rline[k]>XX) break;		//不正确情况
    if(leftmap[k][lline[k]]||rightmap[k][rline[k]]||leftmap[k][19]||rightmap[k][19])	//左右图沿斜率向上搜在圆内必定为0
      return 0;
    if(!leftmap[k][lline[k]]&&!rightmap[k][rline[k]]&&!rightmap[k][19]&&!leftmap[k][19]&&basemap[k][lline[k]]&&basemap[k][rline[k]]&&basemap[k][19])
     n++;		//满足圆的条件，大于5行，则跳出并置标志位
     
  }    
 if(flag)
 {
    uint8 up;
    up=down+3;
    for(uint8 k=0;k<X;k++)
    {
      if(basemap[up][k])			//圆底部向上三行开始搜黑点
      {
        for(uint8 m=k+1;m<X;m++)
        {
          if(m-k>10) return 1; //m-k>10	//黑色区域宽度大于10，断定是圆环
          if(basemap[up][m]==0)		//遇白色说明非圆环
              break;
        }
        break;
      }
    }
 }
return 0;
}
void addin_left()
{
  if(right_y==100||right_x==100||right_x<20)  //右边无角点或角点太靠左，过了中间
  {
   right_x=rightline[0];			//置为底部标准线的位置
   right_y=0;
  }
  uint8 whiteline=right_y+1;			//全白线在角点上一行
  for(uint8 i=right_y+1;i<Y;i++)
  {
    uint8 j;
    for(j=0;j<X;j++)
    {
      if(rightmap[i][j])
        break;
    }
    if(j>=X) 
    {
      whiteline=i;			//更新白线位置至最远处，即圆的下面一行
      break;
    }
  }
  for(uint8 i=whiteline;i<Y;i++)
  {
    uint8 j=rightline[i];			//自圆底部沿右边线向上扫
    if(basemap[i][j])
    {
      if(rightmap[i][j]!=1 && leftmap[i][j]!=1)		//遇到圆上及圆内的黑点则将其归入右图的边线
        searchmap(j, i, rightmap, basemap);		//实际效果即圆的左轮廓成为了右图的边线
      break;
    }
  }
  for(uint8 i=whiteline;i<Y;i++)
  {
    for(uint8 j=0;j<rightline[i];j++)	//圆环底部以上，标记右标准线以左的右边线
    {
      if(rightmap[i][j]==2)
      {
        rightmap[i][j]=3;
        break;
      }
    }
  }
  for(uint8 j=0;j<20;j++)			//右图中间以左再标记一遍
    for(uint8 i=0;i<Y;i++)
    {
      if(rightmap[i][j]==2||rightmap[i][j]==3)
      {
        rightmap[i][j]=3;
        break;
      }
    }
  }
  for(uint8 i = whiteline; i < Y; ++ i)//删除未标记的，将标记的转为边线
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(rightmap[i][j] == 2)
        rightmap[i][j] = 0;
      if(rightmap[i][j] == 3)
        rightmap[i][j] = 2;
    }
  }
  for(uint8 i=0;i<3;i++)		//右图底部3行无边线则补右标准线为边线
  {
    uint8 j;
    for(j=0;j<X;j++)
    {
      if(rightmap[i][j]==2)
        break;
    }
    if(j>=X)
      rightmap[i][rightline[i]]=2;	
  }
}
void addin_right()
{
  if(left_y==100||left_x==100||left_x>19)
  {
   left_x=leftline[0];
   left_y=0;
  }
  uint8 whiteline=left_y+1;
  for(uint8 i=left_y+1;i<Y;i++)
  {
    uint8 j;
    for(j=0;j<X;j++)
    {
      if(leftmap[i][j])
        break;
    }
    if(j>=X) 
    {
      whiteline=i;
      break;
    }
  }
  for(uint8 i=whiteline;i<Y;i++)
  {
    uint8 j=leftline[i];
    if(basemap[i][j])
    {
      if(rightmap[i][j]!=1 && leftmap[i][j]!=1)
        searchmap(j, i, leftmap, basemap);
      break;
    }
  }
  for(uint8 i=whiteline;i<Y;i++)
  {
    for(uint8 j=XX;j>leftline[i];j--)
    {
      if(leftmap[i][j]==2)
      {
        leftmap[i][j]=3;
        break;
      }
    }
  }
  for(uint8 j=20;j<X;j++)
  {
    for(uint8 i=0;i<Y;i++)
    {
      if(leftmap[i][j]==2||leftmap[i][j]==3)
      {
        leftmap[i][j]=3;
        break;
      }
    }
  }
  for(uint8 i = whiteline; i < Y; ++ i)//删除未标记的
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[i][j] == 2)
        leftmap[i][j] = 0;
      if(leftmap[i][j] == 3)
        leftmap[i][j] = 2;
    }
  }
  for(uint8 i=0;i<3;i++)//底部补3个点
  {
    uint8 j;
    for(j=0;j<X;j++)
    {
      if(leftmap[i][j]==2)
        break;
    }
    if(j>=X)
      leftmap[i][leftline[i]]=2;
  }
}
void deletearea(uint8 y,uint8 x,uint8 src[][X],DIRECTION d)	//根据参数，删除左上部分或右上部分的边线
{
  if(d==right)
  {
    for(uint8 i=y;i<Y;i++)
    {
      for(uint8 j=x;j<X;j++)
      {
        if(src[i][j]==2)
          src[i][j]=0;
      }
    }
  }
  else if(d==left)
  {
    for(uint8 i=y;i<Y;i++)
    {
      for(int j=x;j>=0;j--)
      {
        if(src[i][j]==2)
          src[i][j]=0;
      }
    }
  }
}
//void addline(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 src[][X])//y1>y2
//{
//  if(y1<=y2) return;
//  float k=(float)(x1-x2)/(y1-y2);
//  for(uint8 i=y2;i<y1;i++)
//  {
//    src[i][(int)(x2+k*(i-y2))]=2;
//  }
//}
void addout_left()
{
  if(bob_flag)
  {
    for(uint8 j=0;j<X;j++)		//顶部一行有白色则不补线
    {
      if(basemap[YY][j]==0)
        return;
    }
  }
    
    uint8 conor_x=100;
    uint8 conor_y=100;
    uint8 circle_x=100;
    uint8 circle_y=100;
    uint8 flag=0;
    uint8 leftborder=0;
    uint8 myflag=0;
    uint8 whiteline=YY;
    for(int j=XX;j>=0;j--)		//底部开始从右到左扫边线
    {
      for(uint8 i=0;i<Y;i++)
      {
        if(leftmap[i][j])
        {
          leftborder=j;
          myflag=1;
          if(leftborder>=10) return;		//左边线超过1/4处不补线
        }
      }
      if(myflag) break;
    }
    
    for(uint8 i = 56; i <Y; ++ i)		//56行以上最右边列有黑点即非左图也非右图			
    {
      if(basemap[i][XX] && leftmap[i][XX] != 1&&rightmap[i][XX]!=1)//这个东西matlab上放后面的
        searchmap(XX, i, rightmap, basemap);		//归入右图
    }
    
    for(int i=YY;i>=0;i--)		//顶部向下扫，找全白线，有则置标志位
    {
      uint8 j;
      for(j=0;j<X;j++)
      {
        if(rightmap[i][j]) break;
      }
      if(j>=X) 
      {
        whiteline=i;
        flag=1;
        break;
      }
    }
    
    if(flag==0||whiteline>=75) return;		//找不到或太高则退出
    
    flag=0;
    for(uint8 i=whiteline+1;i<Y;i++)  //对出圆环处右图的突出角
    {
      for(uint8 j=0;j<X;j++)
      {
        if(rightmap[i][j])
        {
          for(uint8 k=j+1;k<X;k++)
          {
            if(rightmap[i][k]==0)
            {
              conor_x=k-1;			//记录突出的角的位置
              conor_y=i;
              flag=1;
              break;
            }
          }
          break;
        }
      }
      if(flag) break;
    }
    
    myflag=0;
    if(conor_x<5)						//位置太低的话		
    {
      for(uint8 i=conor_y;i<Y;i++)
      {
        for(uint8 j=0;j<X;j++)
        {
          if(rightmap[i][j]==0)		//遇白
          {
            for(uint8 k=j+1;k<X;k++)
            {
              if(rightmap[i][k])		//往右再有黑
              {
                for(uint8 m=k+1;m<X;m++)
                {
                  if(rightmap[i][m]==0)		//再白
                  {
                    conor_x=m-1;
                    conor_y=i;
                    myflag=1;			//更新位置
                    break;
                  }
                }
                break;
              }
            }
            break;
          }
        }
        if(myflag) break;
      }
    }
    
    if(flag)
    {
      /*方法1*/
//      int nn=1;
//      for(uint8 j=0;j<X;j++)
//      {
//        for(uint8 i=0;i<Y;i++)
//        {
//          if(rightmap[YY-i][j]==0)
//          {
//            for(uint8 m=i+1;m<Y;m++)
//            {
//              if(rightmap[YY-m][j]==2&&circle_x==100&&circle_y==100)//记录一次便不再记录
//              {
//                circle_y=YY-m+1;
//                circle_x=j;
//                break;
//              }
//              else if(rightmap[YY-m][j]==2&&circle_x!=100&&circle_y!=100)
//              {if(nn<=0) rightmap[YY-m][j]=0;
//               else{--nn;break;}
//              }
//              else if(rightmap[YY-m][j]==1)
//                break;
//            }
//            break;
//          }
//        }
//      }
      /*方法2*/
      for(uint8 i=0;i<whiteline;i++)		//删突出角以上以右的边线
      {
        for(uint8 j=0;j<X;j++)
        {
          if(rightmap[i][j]==2)
            rightmap[i][j]=0;
        }
      }
      for(uint8 j=0;j<=rightline[0]+1;j++)
      {
        if(rightmap[0][j]==1)
        {
          circle_x=j-1;		//记录右图第一行从左到右，右标准线以左第一个黑点位置
          circle_y=0;
          break;
        }
      }
      if(circle_x==100)	//右图第一行全白
      {
        circle_x=rightline[0];	//设为标准线位置
        circle_y=0;
      }
      deletearea(conor_y,conor_x,rightmap,right);		//又删一遍		澹垮？澹垮？澹垮？澹�
      //补线
      if(circle_x==100&&circle_y==100){circle_x=XX;circle_y=0;}
      float k=(float)(circle_x-conor_x)/(conor_y-circle_y);			//计算斜率
      for(uint8 i=circle_y;i<=conor_y-2*(conor_y-circle_y)/3;i++)	//只补1/3
      {
        rightmap[i][(int)(circle_x-(i-circle_y)*k)]=2;		//补线
      }
    }
    
}
void addout_right()
{
  if(bob_flag)
  {
    for(uint8 j=0;j<X;j++)
    {
      if(basemap[YY][j]==0)
        return;
    }
  }
    
    uint8 conor_x=100;
    uint8 conor_y=100;
    uint8 circle_x=100;
    uint8 circle_y=100;
    uint8 flag=0;
    uint8 rightborder=XX;
    uint8 myflag=0;
    uint8 whiteline=YY;
    for(int j=0;j<X;j++)
    {
      for(uint8 i=0;i<Y;i++)
      {
        if(rightmap[i][j])
        {
          rightborder=j;
          myflag=1;
          if(rightborder<=XX-10) return;
        }
      }
      if(myflag) break;
    }
    
    for(uint8 i = 56; i <Y; ++ i)
    {
      if(basemap[i][0] && leftmap[i][0] != 1&&rightmap[i][0]!=1)//扫描顶部图像入左图
        searchmap(0, i, leftmap, basemap);
    }
    
    for(int i=YY;i>=0;i--)
    {
      uint8 j;
      for(j=0;j<X;j++)
      {
        if(leftmap[i][j]) break;
      }
      if(j>=X) 
      {
        whiteline=i;
        flag=1;
        break;
      }
    }
    
    if(flag==0||whiteline>=75) return;
    
    flag=0;
    for(uint8 i=whiteline+1;i<Y;i++)
    {
      for(uint8 j=0;j<X;j++)
      {
        if(leftmap[i][XX-j])
        {
          for(uint8 k=j+1;k<X;k++)
          {
            if(leftmap[i][XX-k]==0)
            {
              conor_x=XX-k+1;
              conor_y=i;
              flag=1;
              break;
            }
          }
          break;
        }
      }
      if(flag) break;
    }
    
    myflag=0;
    if(conor_x>34)
    {
      for(uint8 i=conor_y;i<Y;i++)
      {
        for(uint8 j=0;j<X;j++)
        {
          if(leftmap[i][XX-j]==0)
          {
            for(uint8 k=j+1;k<X;k++)
            {
              if(leftmap[i][XX-k])
              {
                for(uint8 m=k+1;m<X;m++)
                {
                  if(leftmap[i][XX-m]==0)
                  {
                    conor_x=XX-m+1;
                    conor_y=i;
                    myflag=1;
                    break;
                  }
                }
                break;
              }
            }
            break;
          }
        }
        if(myflag) break;
      }
    }
    
    if(flag)
    {
      /*方法1*/
//      int nn=1;
//      for(uint8 j=0;j<X;j++)
//      {
//        for(uint8 i=0;i<Y;i++)
//        {
//          if(leftmap[YY-i][j]==0)
//          {
//            for(uint8 m=i+1;m<Y;m++)
//            {
//              if(leftmap[YY-m][XX-j]==2&&circle_x==100&&circle_y==100)//记录一次便不再记录
//              {
//                circle_y=YY-m+1;
//                circle_x=XX-j;
//                break;
//              }
//              else if(leftmap[YY-m][XX-j]==2&&circle_x!=100&&circle_y!=100)
//              {if(nn<=0) leftmap[YY-m][XX-j]=0;
//               else{--nn;break;}
//              }
//              else if(leftmap[YY-m][XX-j]==1)
//                break;
//            }
//            break;
//          }
//        }
//      }
      /*方法2*/
      for(uint8 i=0;i<whiteline;i++)
      {
        for(uint8 j=0;j<X;j++)
        {
          if(leftmap[i][XX-j]==2)
            leftmap[i][XX-j]=0;
        }
      }
      for(uint8 j=XX;j>=leftline[0]-1;j--)
      {
        if(leftmap[0][j]==1)
        {
          circle_x=j+1;
          circle_y=0;
          break;
        }
      }
      if(circle_x==100)
      {
        circle_x=leftline[0];
        circle_y=0;
      }
      deletearea(conor_y,conor_x,leftmap,left);
      //补线
      if(circle_x==100&&circle_y==100){circle_x=0;circle_y=0;}//在方法一要加
      float k=(float)(circle_x-conor_x)/(conor_y-circle_y);
      for(uint8 i=circle_y;i<=conor_y-2*(conor_y-circle_y)/3;i++)
      {
        leftmap[i][(int)(circle_x-(i-circle_y)*k)]=2;
      }
    }
    
}
void ring_out()//该函数的删线功能同时解决了出环删线问题和十字删线不干净的问题
{
  int down_x1=0;//左
  int down_x2=39;//右
  int down = -1;
  for(int b=0;b<=YY;b++)//搜索71斜行
  {
    for(uint8 j=0;j<X;j++)    
    {
      if(j+b>YY) break;		//斜搜索左边部分
      if(leftmap[j+b][j]!=1)//白色
      {
          for(uint8 m=j+1;m<X;m++)
          {
            if(m+b>YY) break;
            if(leftmap[m+b][m]==1)//黑色
            {
              for(uint8 n=m+1;n<X;n++)
              {
                if(n+b>YY) break;
                if(leftmap[n+b][n]!=1)//白色
                {
                  if(down==-1)  
                  {
                    down = b;			//记录扫到的角的位置
                    down_x1=m;//左
                    down_x2=n;//右
                  }
                  break;
                }
                
              }
              break;
            }
          }
          break;
      }
    }
    if(down!=-1) break;
  }
  if(down>=0&&down_x2-down_x1<6)		//宽度符合条件，且扫到了角
  {
    /****删线处理****/
    for(int b=down;b<=YY;b++)		//角的点所在行以上左边一块三角型的边线被删
    {
      for(uint8 j=0;j<X;j++)
      {
        if(j+b>YY) break;
        if(leftmap[j+b][j]==2)
          leftmap[j+b][j]=0;
        if(leftmap[j+b][j]==1)  
          break;
      }
    }
         /*****补线处理***********/
//    for(uint8 i=0;i<Y;i++)
//    {
//      uint8 j;
//      for(j=0;j<X;j++)
//      {
//        if(leftmap[i][j]!=0)
//          break;
//      }
//      if(j<X)
//      {  //标准线补齐底部边线空缺
//        if(i>0)
//        { for(uint8 m=0;m<i*line_add;m++)//补一部分
//          {
//           // leftmap[m][leftline[m]]=2;
//          }
//        }
//        break;
//      }      
//    }

  }
  //搜索右图改用int防止死循环
  down_x1=0;//左
  down_x2=39;//右
  down = -1;
  for(int b=39;b<=118;b++)
  {
    for(int j=XX;j>=0;j--)    
    {
      if(-j+b>YY) break;
      if(rightmap[-j+b][j]!=1)//白色
      {
          for(int m=j-1;m>=0;m--)
          {
            if(-m+b>YY) break;
            if(rightmap[-m+b][m]==1)//黑色
            {
              for(int n=m-1;n>=0;n--)
              {
                if(-n+b>YY) break;
                if(rightmap[-n+b][n]!=1)//白色
                {
                  if(down==-1)  
                  {
                    down = b;
                    down_x2=m;//右
                    down_x1=n;//左
                  }
                  break;
                }
                
              }
              break;
            }
          }
          break;
      }
    }
    if(down!=-1) break;
  }
  if(down>=0&&down_x2-down_x1<6)
  {
    /****删线处理****/
    for(int b=down;b<=118;b++)
    {
      for(int j=XX;j>=0;j--)
      {
        if(-j+b>YY) break;
        if(rightmap[-j+b][j]==2)
        {
           rightmap[-j+b][j]=0;
         //leftmap[-j/4+b][j]=0;
        }
        if(rightmap[-j+b][j]==1)  
          break;
      }
    }
         /*****补线处理***********/
//    for(uint8 i=0;i<Y;i++)
//    {
//      uint8 j;
//      for(j=0;j<X;j++)
//      {
//        if(rightmap[i][j]!=0)
//          break;
//      }
//      if(j<X&&i>0)
//      {  //标准线补齐底部边线空缺
//        for(uint8 m=0;m<i*line_add;m++)
//        {
//         // rightmap[m][rightline[m]]=2;
//        }
//        break;
//      }
//    }

  }
}

void getline(uint8 src[][X], uint8 width[Y], float kb[2])		//左右图给左右线赋值
{
  float sumx = 0;
  float sumy = 0;
  float sumxy = 0;
  float sumx2 = 0;
  int n = 0;
  float x;
  float y;
  for(int i = 0; i < Y; i ++)           //68
  {
    for(int j = 0; j < X; j ++)
    {
      if(src[i][j] == 2)	
      {
        n ++;
        //换算成实际偏差
        y = (float)(width[i]-j)*xx[i];
        x = yy[i];
        sumx += x;
        sumy += y;
        sumxy += x * y;
        sumx2 += x * x;
      }
    }
  }
  kb[0] = (n * sumxy - sumx * sumy) / (n * sumx2 - sumx * sumx);
  kb[1] = sumy / n - kb[0] * sumx / n;
}

uint8 obstacle()//寻找障碍
{
  uint8 obmap[Y][X] = {0};
 // uint8 stmap[Y][X] = {0};//记录停车线的map
  uint8 deletemap[Y][X] = {0};
    
  for(uint8 i = Y * 2 / 3; i < Y; ++ i)//在图像的三分之二以上补左右边沿的缺失的黑色
  {
    if(basemap[i][0] && leftmap[i][0] != 1 && rightmap[i][0] != 1 && deletemap[i][0] != 1)
      searchmap(0, i, deletemap, basemap);
    if(basemap[i][XX] && leftmap[i][XX] != 1 && rightmap[i][XX] != 1 && deletemap[i][XX] != 1)
      searchmap(XX, i, deletemap, basemap);
  }
  for(uint8 i = 0; i < X; ++ i)//补图像上下边沿缺失的黑色
  {
    if(basemap[YY][i] && leftmap[YY][i] != 1 && rightmap[YY][i] != 1 && deletemap[YY][i] != 1)		//目前只找上边
      searchmap(i, YY, deletemap, basemap);
//    if(basemap[0][i] && leftmap[0][i] != 1 && rightmap[0][i] != 1 && deletemap[0][i] != 1)
//      searchmap(i, 0, deletemap, basemap);
  }

  for(uint8 i = 0; i < Y; ++ i)//寻找赛道中的黑色区域并写入obmap中
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(basemap[i][j] && leftmap[i][j] != 1 && rightmap[i][j] != 1 && deletemap[i][j] == 0)
      {
        obmap[i][j] = 1;		//图像特点为黑色障碍四周环绕一圈白色
     //   stmap[i][j] = 1;//拷贝到停车线地图中
      }
    }
  }
  
  for(uint8 i = 0; i < 70; ++ i)//在0到70范围内寻找
  {
    uint8 left = 0;
    uint8 right = XX;
    for(uint8 j = 0; j < X; ++ j)//从左向右扫赛道左边界
    {
      if(basemap[i][j] == 0)
      {
        left = j;
        break;
      }
    }
    for(uint8 j = 0; j < X; ++ j)//从右向左扫赛道右边界
    {
      if(basemap[i][XX - j] == 0)
      {
        right = XX - j;
        break;
      }
    }
    
    for(uint8 j = left; j <= right; ++ j)//在赛道边界内从左向右扫
    {
      if(obmap[i][j])//在障碍图里扫到黑色区域
      {
        int num = 0;
        searchmap(j, i, obstaclemap, obmap);//从扫到的黑色点开始扫到全部黑色区域
        for(uint8 i = 0; i < Y; ++ i)//数黑色区域的大小
        {
          for(uint8 j = 0; j < X; ++ j)
          {
            if(obstaclemap[i][j])
            {
              num ++;
             // stmap[i][j] = 3;//将停车线地图的相应位置置为3，为识别刹车线做准备
            }
          }
        }
        
        uint8 downy = i;//记录下黑色区域的最下边
        
        if(!ramp_flag && runTime >=4) //,起跑三秒内不识别起跑线 ，别忘了开车延时&&mystop_time>=300
        {
          for(uint8 i=downy;i<Y;i++)
          {
            uint8 myflag=0;
            uint8 n=0;
            for(uint8 j=left;j<=right;j++)
            {
              if(obmap[i][j]&&myflag==0)
              {
                n++;
                myflag=1;
              }
              if(obmap[i][j]==0&&myflag==1)
                myflag=0;
            }
            if(n>5) return 3;
            else if(n==0) break;
          }
        }//起跑线判断结束
//        display_realNum(site_numSign,site_numValue,num);
//        display_realNum(site_obdySign,site_obdyValue,obnum[downy]);
        if(num > obnum[downy]/1.7 && num <obnum[downy] + 150) //如果面积符合要求的话开始判断位置
        {
          uint8 direction = 0;//方向清零
          for(uint8 k = downy; k < Y; ++ k)//从黑色区域下面开始向上扫
          {
            int left = 0;
            int right = XX;
            
            for(uint8 m = 0; m < X; ++ m)	//记录左右边
            {
              if(basemap[k][m] == 0)
              {
                left = m;
                break;
              }
            }
            for(uint8 m = 0; m < X; ++ m)
            {
              if(basemap[k][XX - m] == 0)
              {
                right = XX - m;
                break;
              }
            }
            
            uint8 max = 4;//
            if(left == 0)//左边赛道丢失											
            {
              for(uint8 m = 1; m < max; ++ m)
              {
                if(right - m  < 0 || left + m > XX)//防止数组越界
                  break;
                if(obstaclemap[k][right - m])
                {
                  direction = 2; //障碍在右边
                  break;
                }
                if(obstaclemap[k][left + m])
                {
                  if(((float)(right-left-m) * xx[i])>23)//防止误判为左边障碍
                  {
                    direction = 1; //障碍在左边
                    break;
                  }
                  else 
                  {
                    direction = 2; //障碍在右边
                    break;
                  }
                }
              }
            }
	else if(right == XX)//右边赛道丢失
            {
              for(uint8 m = 1; m < max; ++ m)
              {
                if(right - m  < 0 || left + m > XX)//防止数组越界
                  break;
                if(obstaclemap[k][left + m])
                {
                  direction = 1; //障碍在左边
                  break;
                }
                if(obstaclemap[k][right - m])
                {
                  if(((float)(right-m-left) *xx[i])>23)//防止误判为右边障碍
                  {
                    direction = 2; //障碍在右边
                    break;
                  }
                  else 
                  {
                    direction = 1; //障碍在左边
                    break;
                  }
                }
              }
            }
            else
            {
              for(uint8 m = 1; m < max; ++ m)
              {
                if(right - m  < 0 || left + m > XX)
                  break;
                if(obstaclemap[k][left + m])
                {
                  direction = 1; //障碍在左边
                  break;
                }
                if(obstaclemap[k][right - m])
                {
                  direction = 2; //障碍在右边
                  break;
                }
              }
            }
            if(direction == 1)//障碍在左边
            {
              
           //   ob_direction = 1;
              
              uint8 x = X;
              uint8 y = Y;				//记录下边沿Y位置
              
              for(uint8 i = 0; i < Y; ++ i)
              {
                for(uint8 j = 0; j < X; ++ j)//寻找障碍的下边沿
                {
                  if(obstaclemap[i][j] == 1)
                  {
                    y = i;
                    break;
                  }
                }
                if(y != Y)
                  break;
              }
              for(uint8 i = 0; i < X; ++ i)//寻找障碍的右边沿
              {
                for(uint8 j = 0; j < Y; ++ j)
                {
                  if(obstaclemap[j][XX - i] == 1)
                  { 
                    x = XX - i;
                    break;
                  }
                }
                if(x != X)
                  break;
              }
              
              int add = (int)x - (int)leftline[y];//计算障碍与边界的位置
              
              for(uint8 i = 0; i < X; ++ i)
              {
                for(uint8 j = 0; j < Y; ++ j)//清除右图的边界
                {
                  if(rightmap[j][i] == 2)
                    rightmap[j][i] = 0;
                }
              }
              
              for(uint8 i = y; i < Y; ++ i)
              {
                for(uint8 j = 0; j < X; ++ j)//清除左图障碍以上的边界
                {
                  if(leftmap[i][j] == 2)
                     leftmap[i][j] = 0;
                }
             //   if(!gpio_get(PTE8))   LCD_num(site11,(uint32)((float)(((lpp-flpp)/(-70))*downy+lpp)*100), GREEN,RED);
                int position = (int)(leftline[i] + add * ((float)( (flpp/70) * downy+lpp)));//计算新的边界横坐标
                if(position < 0)
                  position = 0;
                else if(position > XX)
                  position = XX;
                leftmap[i][position] = 2;//刷进新的边界
              }
              
              return 1;
            }else if(direction == 2)
            {
              
            //  ob_direction = 2;
              
              uint8 x = X; 
              uint8 y = Y;
              

              
              for(uint8 i = 0; i < Y; ++ i)//寻找障碍的下边沿
              {
                for(uint8 j = 0; j < X; ++ j)
                {
                  if(obstaclemap[i][j] == 1)
                  {
                    y = i;
                    break;
                  }
                }
                if(y != Y)
                  break;
              }
              
               for(uint8 i = 0; i < X; ++ i)//寻找障碍的左边沿
              {
                for(uint8 j = 0; j < Y; ++ j)
                {
                  if(obstaclemap[j][i] == 1)
                  {
                    x = i;
                    break;
                  }
                }
                if(x != X)
                  break;
              }
             
              
//              float width = (float)(rightline[y] - leftline[y]);
//              float p = ((float)rightline[y] - (float)x) / width;
              int add = (int)rightline[y] - (int)x;
              
              for(uint8 i = 0; i < X; ++ i)
              {
                for(uint8 j = 0; j < Y; ++ j)
                {
                  if(leftmap[j][i] == 2)
                    leftmap[j][i] = 0;
                }
              }
              
              for(uint8 i = y; i < Y; ++ i)
              {
                for(uint8 j = 0; j < X; ++ j)
                {
                  if(rightmap[i][j] == 2)
                    rightmap[i][j] = 0;
                }
                
//                float width = (float)(rightline[i] - leftline[i]);
//                int position = (int)(rightline[i] - width * p * rpp);
           //     if(!gpio_get(PTE8))   LCD_num(site11,(uint32)((float)(((rpp-frpp)/(-70))*downy+rpp)*100), GREEN,RED);
                int position = (int)(rightline[i] - add * ((float)( (frpp/70) * downy+rpp)));
                if(position < 0)
                  position = 0;
                else if(position > XX)
                  position = XX;
                rightmap[i][position] = 2;
              }
              
              return 1;
            }
          }
        }
        
        return 0;
      }
    }
  }
  
  return 0;
}