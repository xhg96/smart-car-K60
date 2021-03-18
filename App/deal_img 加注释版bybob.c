#include "deal_img.h"
#include "include.h"

float xx[80]={1.2913,1.2898,1.2906,1.2937,1.3289,1.3361,1.3450,1.3556,1.3677,1.3813,1.3961,1.4121,1.4292,1.4473,1.4663,1.4861,1.5066,1.5277,1.5495,1.5418,1.5646,1.5879,1.6115,1.6356,1.6600,1.6847,1.7098,1.7352,1.7610,1.7871,1.8136,1.8404,1.8677,1.8955,1.9237,1.9525,1.9819,2.0120,2.0428,2.0745,2.1070,2.1406,2.1753,2.2111,2.2483,2.2869,2.3271,2.3690,2.4127,2.4584,2.5062,2.5562,2.6088,2.6639,2.7219,2.7829,2.8470,2.9145,2.9856,3.0605,3.1394,3.2225,3.3101,3.4025,3.4998,3.6023,3.7102,3.8239,3.9437,4.0697,4.2023,4.3417,4.4883,4.6424,4.8043,4.9743,5.1528,5.3400,5.5363,5.7421};
float yy[80]={0,0.56058,1.01680,1.52182,2.07163,2.66242,3.29057,3.95268,4.64552,5.36609,6.11156,6.87932,7.66693,8.47218,9.29305,10.12769,10.97449,11.83202,12.69903,13.57450,14.45759,15.34766,16.24428,17.14720,18.05637,18.97196,19.89431,20.82398,21.76171,22.70846,23.66538,24.63379,25.61526,26.61152,27.62450,28.65635,29.70940,30.78618,31.88943,33.02208,34.18726,35.38829,36.62870,37.91220,39.24273,40.62441,42.06154,43.55865,45.12045,46.75186,48.45798,50.24413,52.11580,54.07872,56.13878,58.30207,60.57492,62.96381,65.47543,68.11669,70.89468,73.81669,76.89021,80.12292,83.52272,87.09769,90.85611,94.80646,98.95743,103.31788,107.89690,112.70376,117.74792,123.03907,128.58708,134.40199,140.49410,146.87384,153.55190,160.53912};
int obnum[70]={402,399,393,386,379,367,363,356,346,340,331,325,314,307,299,299,290,281,276,267,263,255,253,244,240,231,227,223,218,212,205,195,187,178,170,163,157,153,152,146,141,138,131,124,120,119,116,110,105,99,96,90,88,86,82,79,77,72,71,66,62,60,56,51,51,51,48,44,44,39};
uint8 basemap[Y][X];	//°×µãÖÃ0£¬ºÚµãÖÃ2
uint8 leftmap[Y][X];		// 1ºÚ 2 ±ßÏß 0°×
uint8 bob_flag=0;
uint8 rightmap[Y][X];
uint8 obstaclemap[Y][X];
uint8 leftline[Y],rightline[Y];
uint8 speedlline[Y],speedrline[Y];//ÓÃÓÚ¼ÆËãother_top
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
//¸³ÖµÎª100±íÊ¾Î´¼ì²âµ½
uint8 left_x=100;
uint8 left_y=100;
uint8 right_x=100;
uint8 right_y=100;

//Site_t site_numSign={92, 55};        
//Site_t site_numValue={100, 55};
//Site_t site_obdySign={85-40, 55};        
//Site_t site_obdyValue={93-40, 55};
void standard()                            //±ê×¼Ïß£¬¸ù¾İÉãÏñÍ·¸ß¶È½Ç¶Èµ÷Õû
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
ÓÃµİ¹éÑ¹ËõÍ¼Ïñ£¬°×µãÖÃ0£¬ºÚµãÖÃ2
×îÖÕsrc[]ÀïÖ»ÄÜÓĞ0ºÍ2
*/
void searchimg(uint8 x, uint8 y, uint8 src[][X], int threshold)//¸øbasemap[]¸³Öµ
{
  if(!judge(x, y, threshold))
  {
    src[y][x] = 0;	
    if(x > 0 && src[y][x - 1] == 1)		//ËÄÖÜÓĞÎª1µÄµãÔò¼ÌĞøËÑË÷
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
½«scr2Ó³Éäµ½scr
scr[]ÖĞ×îÖÕ½öÓĞ1ºÍ2
2->1 ºÚ, 0->2	°×
*/
void searchmap(uint8 x, uint8 y, uint8 src[][X], uint8 src2[][X])//ÓÃbasemap[]¸ø×óÓÒÍ¼¸³Öµ
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
uint8 judge(uint8 x, uint8 y, int threshold)    //ÅĞºÚÉ«
{
  for(uint8 i = 1; i <= VERTICAL; ++ i)               //3ĞĞ²¢Ò»ĞĞ£¬240±ä80
  {
    if(imgbuff[(CAMERA_H - y * VERTICAL - i) * X + x] >= threshold)
      return 1;
  }
  return 0;
}


void deleteline(int deletenum)
{
 // uint8 deletenum = 0;  //Õâ¸öÖµ¿´Çé¿öµ÷½Ú
  uint8 down = 0;
  uint8 up = 0;
  for(uint8 i = 0; i < Y; ++ i)		//´Óµ×²¿ÏòÉÏ£¬´Ó×óÍùÓÒÉ¨£¬Óö°× ºÚ °×£¬¶ÔÓ¦YÖáµÄÖµ¸øup   
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[i][j] != 1)	//xÓö°×É«
      {
        for(uint8 k = j + 1; k < X; ++ k)	//ÓÒ±ß
        {
          if(leftmap[i][k] == 1)	//ÎªºÚÉ«
          {
            for(uint8 m = k + 1; m < X; ++ m)//ÔÙÓÒ
            {
              if(leftmap[i][m] != 1)		//°×É«x
              {
                if(i != up + 1)  //ÈôÓĞég¸ô„t¸üĞÂdown
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
  
  if(up != 0 && up - down > deletenum)        //ĞĞ”µ³¬ß^deletenum ²Å•ş„h¾Q
  {
    for(uint8 i = down; i < up - deletenum&&i<Y; ++ i)//»Ø­h²¢²»ÍêÈ«„hÍê
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
  /*****ÓÒ¾QÔÙíÒ»´Î****/
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
  for(uint8 i = 1; i < 12; ++ i)		//´ÓÉÏµ½ÏÂ£¬´ÓÓÒµ½×ó
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[YY - i][XX - j] == 2)	//×óÍ¼Óö±ßÏß
      {
        for(uint8 k = XX - j + 1; k < X; ++ k)  //k¼ÇÂ¼±ßÏßx×ø±ê£¬´Óx¿ªÊ¼ÏòÓÒÉ¨
        {
          if(rightmap[YY - i][k] == 2)		//ÓÒÍ¼Óöµ½±ßÏß
          {
            if(XX-j>rightline[YY-i] || k<leftline[YY-i])  //×ó±ßÏßÔ½¹ıÓÒ±ßÏß»òÓÒ±ßÏßÔ½¹ı×ó±ßÏß
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
  
  for(uint8 i = 13; i < max; ++ i)//13ĞĞµ½20ĞĞ
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[YY - i][XX - j] == 2)
      {
        for(uint8 k = XX - j + 1; k < X; ++ k)
        {
          if(rightmap[YY - i][k] == 2)
          {
            if(k - (XX - j) < rightline[YY - i] - leftline[YY - i] + (max - i) * 4 / max + 2)  //µ÷ÕûÎÒ
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
  for(uint8 i = 30; i < Y; ++ i)		//30ĞĞÒÔÉÏµÄÍ¼Ïñ£¬Çå³ı±ßÏß
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
        for(uint8 k=j+1;k<X;++k)	//30ĞĞÒÔÏÂ£¬ ×ó±ßÏßÒÔÓÒÇå0  ÎªÊ²Ã´?
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
  if(ramp()&&my_ramp_flag==0)      //ÉÏÆÂ¼ì²âµ½
  {
    my_ramp_flag=1;
    xiaodou=0;
  }
  
  
  if(my_ramp_flag==1)
  {
    xiaodou++;
    if(xiaodou>5)
    {
      if(!ramp())              //ÔÚÆÂÉÏ¼ì²â²»µ½
      {
        my_ramp_flag=2;
      }
    }
  }
  
  if(my_ramp_flag==2)
  {
    delay_time++;
    if(ramp())          //ÏÂÆÂÔÙ´Î¼ì²âµ½
    {
      my_ramp_flag=3;
      delay_time=0;
    }
    else if(delay_time>50)      //ÈôÎóÅĞ£¬Ôò×î¶àÎóÅĞÊ±¼äÎª50
    {
      my_ramp_flag=0;
      delay_time=0;
    }
  }
  
  if(my_ramp_flag==3||my_ramp_flag==4)
  {
    delay_time++;
    if(delay_time>ramp_delay)         //ÑÓÊ±£¬»Ö¸´³õÊ¼»¯
      my_ramp_flag=4;
    if(delay_time>50)         //ÑÓÊ±£¬»Ö¸´³õÊ¼»¯
    {
      my_ramp_flag=0;
      delay_time=0;
    }
  }
  
  if(my_ramp_flag==1||my_ramp_flag==2) rampline();  //ÆÂÉÏÖ»Ê¹ÓÃµ×²¿Í¼Ïñ¿ØÖÆ·½Ïò¡£
  
  return my_ramp_flag;
  //Ğ§¹û£¬·µ»ØÖµÎª1¡¢2¡¢3ÔÚÆÂµÀ£¬·µ»Ø0Ê±²»ÔÚ
}
uint8 crossroadforleft()
{
  left_x=100;
  left_y=100;
  uint8 high = 3;
  int upd;//¼ÇÂ¼½ÇµãÏÂ±ßÑØµÄYÎ»ÖÃ
  int upu;//¼ÇÂ¼½ÇµãÉÏ±ßÑØµÄYÎ»ÖÃ
  int up;//¼ÇÂ¼½ÇµãµÄXÎ»ÖÃ
  uint8 flag = 0;
  for(uint8 i = 0; i < X; ++ i)         
  {
    for(uint8 j = 0; j < Y; ++ j)//´ÓÏÂµ½ÉÏ£¬´ÓÓÒµ½×ó
    {
      if(leftmap[j][XX - i] == 1)//Óöµ½ºÚÉ«ÇøÓò¾Í¼ÇÏÂµ×²¿£¬½Ó×ÅÉ¨ÉÏÃæ±ßÑØ
      {
        upd = j;//¼ÇÂ¼½ÇµãÏÂ±ßÑØµÄYÎ»ÖÃ
        up = XX - i;//¼ÇÂ¼½ÇµãµÄXÎ»ÖÃ
        for(uint8 k = j + 1; k < Y; ++ k)//½Ó×Å¼ÌĞøÏòÉÏÉ¨½ÇµãµÄÉÏ±ßÑØ
        {
          if(leftmap[k][up] != 1)//Óöµ½·ÇºÚÉ«µÄÇøÓò,ÇÒÔÚ75ĞĞÒÔÏÂÊ±
          {
            upu = k - 1;//¼ÇÂ¼ÏÂ±ßÑØµÄÎ»ÖÃ
            if(upu<75)
            {
              flag = 1;//±íÊ¾¿ÉÒÔ½øÈëÏÂÒ»²½ÁË
            }
            break;
          } 
        }
        break;
      }
    }
    if(flag)//Èç¹ûÃ»É¨µ½ºÏÊÊµÄ¾Í¼ÌĞøÉ¨
      break;
  }
  
  int downu;//¼ÇÂ¼½Çµ×ÉÏ±ßÑØµÄYÎ»ÖÃ
  int downd;//¼ÇÂ¼½Çµ×ÏÂ±ßÑØµÄYÎ»ÖÃ
  uint8 down;
  if(flag)
  {
    down = up - high;//ÕÒµ½½Çµ× 
    uint8 upmax =15;//½ÇµãµÄ×î´ó¿í¶È£¬YÖáÉÏµÄ     
    if(upu >= 75 || up <= high  || upu - upd >= upmax)//Èç¹û½ÇµãµÄÌØÕ÷²»·ûºÏ¾ÍÍË³ö³ÌĞò|| upd <= 5
      return 0;
    
    
    
     for(uint8 i = upu; i < Y; ++ i)     //¶ÔÖ±½Ç £¬´Ó½Çµã¶¥²¿ÏòÉÏ
    {
      if(rightmap[i][up] == 1)
      {
        uint8 j;
        for(j=0;j<X;j++)
        {
          if(rightmap[YY][j]==0)	//ÓÒÍ¼¶¥²¿É¨
            break;
        }
        if(j>=X) return 0;	//ÓÒÍ¼¶¥²¿È«ºÚÉ«ÔòÍË³ö
        else break;
      }
      if(i - upu == (uint8)((Y - upu) / 1.2))	//iÉ¨µ½Ò»¶¨Öµ·ÅÆúÉ¨Ãè
        break;
    }
    
    
    
    flag = 0;//±êÖ¾Î»Çå0
    for(uint8 i = 0; i < Y; ++ i)//ÔÚ½Çµ×´ÓÏÂÍùÉÏÉ¨
    {
      if(leftmap[i][down] == 1)//Óöµ½ºÚÉ«ÇøÓò
      {
        downd = i;//¼ÇÂ¼½Çµ×ÏÂ±ßÑØµÄYÎ»ÖÃ
        downu = YY;//¼ÇÂ¼½Çµ×ÉÏ±ßÑØµÄYÎ»ÖÃ
        flag = 1;
        for(uint8 j = i; j < Y; ++ j)//¼ÌĞøÏòÉÏÉ¨
        {
          if(leftmap[j][down] != 1)//Óöµ½·ÇºÚÉ«ÇøÓò
          {
            downu = j - 1;//Ë¢ĞÂ½Çµ×ÉÏ±ßÑØµÄYÎ»ÖÃ
            break;
          }
        }
        break;
      }
    }
  }else
    return 0;
  
  uint8 downmax = 40;//µ×±ßµÄ×î´ó³¤¶È
  int pointnum = 0;
  int mynum = 0;
  if(flag && downu - downd < downmax)//µ×±ßÕÒµ½ÇÒ³¤¶È·ûºÏ
  {
    left_x=up;
    left_y=upu;
    for(uint8 i = down; i <= up; ++ i)
    {
      for(uint8 j = downd; j <= downu; ++ j)//¼ÇÂ¼¿íÎª½Ç¶¥µ½½Çµ×µÄ¾àÀë£¬³¤Îª½Çµ×µÄ¾ØĞÎÄÚµÄºÚµãÊı
      {
        if(leftmap[j][i] == 1)
          ++ pointnum;
      }
    }
    mynum = (downu - downd) * (high + 1) / 2;//ËãÈı½ÇĞÎÃæ»ı
    mynum += (downu - downd)*1.8;//¼ÓÉÏµ×±ßµÄ³¤¶ÈµÄ1.5
    if(mynum >= pointnum)//³õ²½Ìõ¼ş³ÉÁ¢,½øÈË¸üÑÏ¸ñµÄÅĞ¶¨
    {
      for(uint8 i = 0; i < up; ++ i)
      {
        for(uint8 j = 0; j < Y; ++ j)//´ÓÏÂµ½ÉÏ£¬´Ó×óµ½½Ç¶¥
        {
          if(leftmap[j][i] == 1)//Óöµ½ºÚÉ«ÇøÓò
          {
            for(uint8 k = j + 1; k < Y; ++ k)//¼ÌĞøÏòÉÏÕÒ
            {
              if(leftmap[k][i] == 2)//Óöµ½±ß½ç
              {
               // if(upd - 10 > k)//&& upd > 10 
                 // break;
                for(uint8 m = k; m < Y; ++ m)//¼ÌĞøÏòÉÏÕÒ
                {
                  if(leftmap[m][i] == 0)//Óöµ½°×É«ÇøÓò
                  {
                    for(uint8 n = m + 1; n < Y; ++ n)//¼ÌĞøÏòÉÏÕÒ
                    {
                      if(leftmap[n][i] == 2)//ÔÚ×óÍ¼Óöµ½±ß½ç
                      {
                        for(uint8 a = n; a < Y; ++ a)//¼ÌĞøÏòÉÏÕÒ
                        {
                          if(leftmap[a][i] != 2)
                            break;
                          leftmap[a][i] = 0;//É¾³ı×óÍ¼ÉÏ±ß½ç
                        }
                        break;
                      }
                      if(rightmap[n][i] == 2)//ÔÚÓÒÍ¼Óöµ½±ß½ç
                      {
                        for(uint8 a = n; a < Y; ++ a)//¼ÌĞøÏòÉÏÕÒ
                        {
                          if(rightmap[a][i] != 2)
                            break;
                          rightmap[a][i] = 0;//É¾³ıÓÒÍ¼ÉÏ±ß½ç
                        }
                        break;
                      }
                    }
                    break;
                  }
                  leftmap[m][i] = 0;//É¾³ıÏÂ±ß½ç
                }
                break;
              }
            }
            break;
          }
        }
      }
      
      if(upu < Y / 2&&!ring_flag)//µ±½Ç¶¥µÄÉÏ²¿ÔÚÆÁÄ»µÄÒ»°ëÒÔÏÂÊ±µÄ
      {
        for(uint8 i = 1; i < (Y - upu) * 2 / 3; ++ i)
        {
          if(i + upu > YY)//·ÀÖ¹Ô½½ç
            break;
          if(basemap[i + upu][up])	//ºÚ
          {
            if(leftmap[i + upu][up] != 1 && rightmap[i + upu][up] != 1 && basemap[i + upu][up])//ÓÒ£¬×óÍ¼Îª±ßÏß»ò°×É«£¬Ô­Í¼ºÚ
              searchmap(up, i + upu, leftmap, basemap);
            break;
          }
        }
      }
      //±ê¼Ç´ÓÓÒµ½×óµÄÉ¨µ½µÄ±ß½ç
      for(uint8 i = 0; i < Y; ++ i)//Ìù×ÅÓÒ±ßÑØ´ÓÏÂµ½ÉÏÉ¨   //Õâ¸öÉ¾Ïß¸Ğ¾õ¸´ÔÓ¶øÇÒĞ§¹û²»ºÃ£¬ÏÈ×¢ÊÍµôÊÔÊÔ
      {
        if(leftmap[i][XX] != 1)//Óöµ½·ÇºÚÉ«ÇøÓò
        {
          for(uint8 j = 1; j < X; ++ j)//ÔÚ·ÇºÚÉ«ÄÇÒ»ĞĞ´ÓÓÒÏò×óÉ¨
          {
            if(leftmap[i][XX - j] == 2)//Óöµ½±ß½ç¾Í±ê¼Ç
            {
              leftmap[i][XX - j] = 3;
              break;
            }
          }
        }
      }
      //±ê¼Ç´ÓÏÂµ½ÉÏÉ¨µ½µÄÓÒ°ë±ß½ç
      for(uint8 i = X / 2; i < X; ++ i)//ÔÚ×îµ×²¿ĞĞ£¬´ÓÖĞ¼äÍùÓÒÉ¨
      {
        if(leftmap[0][i] != 1)//Óöµ½·ÇºÚÉ«ÇøÓò
        {
          for(uint8 j = 1; j < Y; ++ j)//´Ó·ÇºÚÉ«ËùÔÚÁĞ´ÓÏÂÏòÉÏÉ¨
          {
            if(leftmap[j][i] == 2 || leftmap[j][i] == 3)//Óöµ½±ß½ç¾Í±ê¼Ç
            {
              leftmap[j][i] = 3;
              break;
            }
          }
        }
      }
      
      for(uint8 i = 0; i < Y; ++ i)//É¾³ıÎ´±ê¼ÇµÄ
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
    
     for(uint8 i = upu; i < Y; ++ i)     //¶ÔÖ±½Ç
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
    if(mynum >= pointnum)//³õ²½Ìõ¼ş³ÉÁ¢,½øÈË¸üÑÏ¸ñµÄÅĞ¶¨
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
             //   if( upd - 10 > k)//upd > 10 &&           ¾ÍÊÇÕâ¶«Î÷µ¼ÖÂĞÂÈüµÀÊ®×Ö³öÎÊÌâ£¬¸ãµôÁËÂèµÄ
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
                  rightmap[m][i] = 0;//É¾³ıµÄÊÇÏÂ±ß½ç
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
      
      for(uint8 i = 0; i < Y; ++ i)//Õâ¸öÉ¾Ïß¸Ğ¾õ¸´ÔÓ¶øÇÒĞ§¹û²»ºÃ£¬ÏÈ×¢ÊÍµôÊÔÊÔ
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
      for(uint8 i = 0; i < X / 2; ++ i) //ĞŞ¸Ä£¿
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
      if(basemap[0][j]) break;	//µ×²¿Í¼ÏñÈ«°×
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
    for(uint8 i=0;i<5;i++)		//µ×²¿5ĞĞ¶¼ÓĞºÚÉ«
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
  else if(ring_flag==3)			//ÔÚÔ²»·ÄÚÊ±
  {    
    uint8 down=0;
    uint8 up=0;
    uint8 whiteline=Y;  
    if(ring_direction==left)//×ß×ó±ß
    {
      addout_left();
      if(left_y<50&&left_x<30)		//½Çµã´¦ÔÚÕâ¸öÎ»ÖÃ¾Í³öÔ²»·
      {
        ring_flag=4;
        return;
      }
      for(uint8 i=0;i<Y;i++) //´ÓÏÂÍùÉÏ£¬´Ó×óÍùÓÒ£¬¼ÇÂ¼ÓÒÍ¼Àëµ×²¿ ×î½üµÄÒ»ÌõÈ«°×ĞĞ
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
      for(uint8 i=whiteline;i<Y;i++)	//´ÓÈ«°×ĞĞÍùÉÏÉ¨
      {
        for(uint8 j=0;j<X;j++)
        {
          if(rightmap[i][j]==0)	//ÓÒÍ¼Óö°×µã
          {
            for(uint8 k=j+1;k<X;k++)
            {
              if(rightmap[i][k]==1)	//ÔÙºÚ
              {
                for(uint8 m=k+1;m<X;m++)
                {
                  if(rightmap[i][m]==0)	//ÔÙ°×
                  {
                    if(i!=up+1)		//¸üĞÂ down ºÍ up
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
    else			//×ßÓÒ±ß
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
    }//×ßÓÒ±ß
    if(up-down>2&&down<75&&up-down<40)    //ÓöÌõ¼şÂú×ã³ö»·
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
uint8 ring_judge()//Ä¿Ç°³¢ÊÔm-k>8.n>2
{
 if( (right_x==100||right_y==100) && (left_y==100||left_x==100) )  return 0; 	//×óÓÒ½Çµã¶¼ÎŞ½Çµã
 int lline[Y],rline[Y];//ÓÃÓÚÔ²»·ÅĞ±ğ
 uint8 mid_y;
 if(right_x==100||right_y==100)   //¶ªÊ§ÓÒ½Å  
 {
   mid_y=left_y;
   uint8 bottom=100;
   uint8 site_x=100;
   uint8 site_y=100;
   for( uint8 j=0;j<X;j++)
   {
     if(leftmap[0][XX-j]) 
     {
        bottom=XX-j;//ÕÒµ½µ×²¿×óÍ¼±ß½ç
        break;
     }
   }
   if(bottom<19)			//¶Ô±ß½çÎ»ÖÃÏŞ¶¨
   {
      for(uint8 j=0;j<X;j++)
      {
        for(uint8 i=0;i<Y;i++)
        {
          if(leftmap[i][XX-j])	//ÔÙÕÒ³ıµ×²¿ÍâÁíÒ»Ìõ±ßÏßµãÎ»ÖÃ
          {
            site_x=XX-j;
            site_y=i;
            break;
          }
        }
        if(site_x!=100) break;	//ÕÒµ½¼´Ìø³ö
      }
      int r_b=19-bottom;							
      float r_k=(float)(site_x-bottom)/site_y;		//Ğ±ÂÊ
      for(uint8 i = 0; i < Y; ++ i)
      {
        float x=-r_k* i+ r_b;		//¸ù¾İĞ±ÂÊËã¶ÔÓ¦YÎ»ÖÃX
        lline[i] = (uint8)(19 - x);		//×ó±ß°´Ğ±ÂÊ²¹Ïß
        rline[i] =  rightline[i];		//ÓÒ±ß²¹±ê×¼Ïß
      }
   }
   else
   {
      for(uint8 i = 0; i < Y; ++ i)    //±ßÏßÔ½¹ı20ĞĞ(Í¼µÄÖĞ¼ä)Ôò²¹±ê×¼Ïß
      {
        lline[i] = leftline[i];
        rline[i] = rightline[i];
      }
   }
 }
 else if(left_y==100||left_x==100)  //¶ªÊ§×ó½Å  
 {  
   mid_y=right_y;
   uint8 bottom=100;
   uint8 site_x=100;
   uint8 site_y=100;
   for( uint8 j=0;j<X;j++)
   {
     if(rightmap[0][j]) 
     {
        bottom=j;//ÕÒµ½µ×²¿ÓÒÍ¼±ß½ç
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
            site_y=i;//ÕÒµ½×î×ó±ßµÄµã
            break;
          }
        }
        if(site_x!=100) break;
      }
      int r_b=bottom-20;
      float r_k=(float)(bottom-site_x)/site_y;
      for(uint8 i = 0; i < Y; ++ i)//Á½µãÁ¬³ÉÒ»ÌõÖ±Ïßrline£¬×ó±ßÓÃ±ê×¼Ïß
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
 else               						//×óÓÒ½Ç¶¼ÔÚ
 {
    mid_y=(left_y+right_y)/2;
    for(uint8 i = 0; i < Y; ++ i)
    {
      lline[i] = leftline[i];
      rline[i] = rightline[i];
    }   
 }
 uint8 whiteline=YY;
 for(uint8 i=mid_y;i<Y;i++)	//´Ó½Çµã¶¥²¿ÏòÉÏÉ¨£¬ÕÒÈ«°×µÄÏß
 {
   uint8 j;
  for(j=0;j<X;j++)
  {
    if(basemap[i][j]) break;
  }
  if(j>=X)	//Î´Óöµ½ºÚµã£¬Ôò¸üĞÂÈ«°×Ïß
  {
    whiteline=i;
    break;
  }
 }
 if(whiteline>=73) return 0;    //73ĞĞÒÔÉÏ²»¹Ü
 uint8 down=YY;
 uint8 downd=0;
 for(uint8 i=whiteline;i<Y;i++)	//´ÓÈ«°×ÏßÏòÉÏÉ¨£¬´Ó×óµ½ÓÒÕÒºÚµã£¬¼´Ô²µÄÏÂ±ßÔµ
 {
  uint8 j;
  for(j=0;j<X;j++)
  {
    if(basemap[i][j])
      break;
  }
  if(j<X) 
  {
    down=i;	//¼ÇÂ¼ºÚµãÎ»ÖÃ
    downd=j;
    break;
  }
 }
 if(down>=75 ) return 0;		//ºÚµãÌ«¸ß£¬²»¹Ü
 if(left_x==100 && downd>leftline[down] && right_x<rightline[right_y]-5) return 0;		//×ó¶ª½Ç£¬ºÚµã±ØĞëÔÚ±ê×¼ÏßÄÚ£¬½ÇµãÓë±ê×¼Ïß½Ï½Ó½ü
 if(right_x==100 && downd<rightline[down] && left_x>leftline[left_y]+5) return 0;

 uint8 flag=0;

  uint8 n=0;
  for(uint8 k=down;k<Y;k++)
  {
    if(n>4) //n>4
    {
      flag=1;break;
    }
    if( lline[k]>rline[k] || lline[k]<0 || rline[k]>XX) break;		//²»ÕıÈ·Çé¿ö
    if(leftmap[k][lline[k]]||rightmap[k][rline[k]]||leftmap[k][19]||rightmap[k][19])	//×óÓÒÍ¼ÑØĞ±ÂÊÏòÉÏËÑÔÚÔ²ÄÚ±Ø¶¨Îª0
      return 0;
    if(!leftmap[k][lline[k]]&&!rightmap[k][rline[k]]&&!rightmap[k][19]&&!leftmap[k][19]&&basemap[k][lline[k]]&&basemap[k][rline[k]]&&basemap[k][19])
     n++;		//Âú×ãÔ²µÄÌõ¼ş£¬´óÓÚ5ĞĞ£¬ÔòÌø³ö²¢ÖÃ±êÖ¾Î»
     
  }    
 if(flag)
 {
    uint8 up;
    up=down+3;
    for(uint8 k=0;k<X;k++)
    {
      if(basemap[up][k])			//Ô²µ×²¿ÏòÉÏÈıĞĞ¿ªÊ¼ËÑºÚµã
      {
        for(uint8 m=k+1;m<X;m++)
        {
          if(m-k>10) return 1; //m-k>10	//ºÚÉ«ÇøÓò¿í¶È´óÓÚ10£¬¶Ï¶¨ÊÇÔ²»·
          if(basemap[up][m]==0)		//Óö°×É«ËµÃ÷·ÇÔ²»·
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
  if(right_y==100||right_x==100||right_x<20)  //ÓÒ±ßÎŞ½Çµã»ò½ÇµãÌ«¿¿×ó£¬¹ıÁËÖĞ¼ä
  {
   right_x=rightline[0];			//ÖÃÎªµ×²¿±ê×¼ÏßµÄÎ»ÖÃ
   right_y=0;
  }
  uint8 whiteline=right_y+1;			//È«°×ÏßÔÚ½ÇµãÉÏÒ»ĞĞ
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
      whiteline=i;			//¸üĞÂ°×ÏßÎ»ÖÃÖÁ×îÔ¶´¦£¬¼´Ô²µÄÏÂÃæÒ»ĞĞ
      break;
    }
  }
  for(uint8 i=whiteline;i<Y;i++)
  {
    uint8 j=rightline[i];			//×ÔÔ²µ×²¿ÑØÓÒ±ßÏßÏòÉÏÉ¨
    if(basemap[i][j])
    {
      if(rightmap[i][j]!=1 && leftmap[i][j]!=1)		//Óöµ½Ô²ÉÏ¼°Ô²ÄÚµÄºÚµãÔò½«Æä¹éÈëÓÒÍ¼µÄ±ßÏß
        searchmap(j, i, rightmap, basemap);		//Êµ¼ÊĞ§¹û¼´Ô²µÄ×óÂÖÀª³ÉÎªÁËÓÒÍ¼µÄ±ßÏß
      break;
    }
  }
  for(uint8 i=whiteline;i<Y;i++)
  {
    for(uint8 j=0;j<rightline[i];j++)	//Ô²»·µ×²¿ÒÔÉÏ£¬±ê¼ÇÓÒ±ê×¼ÏßÒÔ×óµÄÓÒ±ßÏß
    {
      if(rightmap[i][j]==2)
      {
        rightmap[i][j]=3;
        break;
      }
    }
  }
  for(uint8 j=0;j<20;j++)			//ÓÒÍ¼ÖĞ¼äÒÔ×óÔÙ±ê¼ÇÒ»±é
    for(uint8 i=0;i<Y;i++)
    {
      if(rightmap[i][j]==2||rightmap[i][j]==3)
      {
        rightmap[i][j]=3;
        break;
      }
    }
  }
  for(uint8 i = whiteline; i < Y; ++ i)//É¾³ıÎ´±ê¼ÇµÄ£¬½«±ê¼ÇµÄ×ªÎª±ßÏß
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(rightmap[i][j] == 2)
        rightmap[i][j] = 0;
      if(rightmap[i][j] == 3)
        rightmap[i][j] = 2;
    }
  }
  for(uint8 i=0;i<3;i++)		//ÓÒÍ¼µ×²¿3ĞĞÎŞ±ßÏßÔò²¹ÓÒ±ê×¼ÏßÎª±ßÏß
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
  for(uint8 i = whiteline; i < Y; ++ i)//É¾³ıÎ´±ê¼ÇµÄ
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(leftmap[i][j] == 2)
        leftmap[i][j] = 0;
      if(leftmap[i][j] == 3)
        leftmap[i][j] = 2;
    }
  }
  for(uint8 i=0;i<3;i++)//µ×²¿²¹3¸öµã
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
void deletearea(uint8 y,uint8 x,uint8 src[][X],DIRECTION d)	//¸ù¾İ²ÎÊı£¬É¾³ı×óÉÏ²¿·Ö»òÓÒÉÏ²¿·ÖµÄ±ßÏß
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
    for(uint8 j=0;j<X;j++)		//¶¥²¿Ò»ĞĞÓĞ°×É«Ôò²»²¹Ïß
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
    for(int j=XX;j>=0;j--)		//µ×²¿¿ªÊ¼´ÓÓÒµ½×óÉ¨±ßÏß
    {
      for(uint8 i=0;i<Y;i++)
      {
        if(leftmap[i][j])
        {
          leftborder=j;
          myflag=1;
          if(leftborder>=10) return;		//×ó±ßÏß³¬¹ı1/4´¦²»²¹Ïß
        }
      }
      if(myflag) break;
    }
    
    for(uint8 i = 56; i <Y; ++ i)		//56ĞĞÒÔÉÏ×îÓÒ±ßÁĞÓĞºÚµã¼´·Ç×óÍ¼Ò²·ÇÓÒÍ¼			
    {
      if(basemap[i][XX] && leftmap[i][XX] != 1&&rightmap[i][XX]!=1)//Õâ¸ö¶«Î÷matlabÉÏ·ÅºóÃæµÄ
        searchmap(XX, i, rightmap, basemap);		//¹éÈëÓÒÍ¼
    }
    
    for(int i=YY;i>=0;i--)		//¶¥²¿ÏòÏÂÉ¨£¬ÕÒÈ«°×Ïß£¬ÓĞÔòÖÃ±êÖ¾Î»
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
    
    if(flag==0||whiteline>=75) return;		//ÕÒ²»µ½»òÌ«¸ßÔòÍË³ö
    
    flag=0;
    for(uint8 i=whiteline+1;i<Y;i++)  //¶Ô³öÔ²»·´¦ÓÒÍ¼µÄÍ»³ö½Ç
    {
      for(uint8 j=0;j<X;j++)
      {
        if(rightmap[i][j])
        {
          for(uint8 k=j+1;k<X;k++)
          {
            if(rightmap[i][k]==0)
            {
              conor_x=k-1;			//¼ÇÂ¼Í»³öµÄ½ÇµÄÎ»ÖÃ
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
    if(conor_x<5)						//Î»ÖÃÌ«µÍµÄ»°		
    {
      for(uint8 i=conor_y;i<Y;i++)
      {
        for(uint8 j=0;j<X;j++)
        {
          if(rightmap[i][j]==0)		//Óö°×
          {
            for(uint8 k=j+1;k<X;k++)
            {
              if(rightmap[i][k])		//ÍùÓÒÔÙÓĞºÚ
              {
                for(uint8 m=k+1;m<X;m++)
                {
                  if(rightmap[i][m]==0)		//ÔÙ°×
                  {
                    conor_x=m-1;
                    conor_y=i;
                    myflag=1;			//¸üĞÂÎ»ÖÃ
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
      /*·½·¨1*/
//      int nn=1;
//      for(uint8 j=0;j<X;j++)
//      {
//        for(uint8 i=0;i<Y;i++)
//        {
//          if(rightmap[YY-i][j]==0)
//          {
//            for(uint8 m=i+1;m<Y;m++)
//            {
//              if(rightmap[YY-m][j]==2&&circle_x==100&&circle_y==100)//¼ÇÂ¼Ò»´Î±ã²»ÔÙ¼ÇÂ¼
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
      /*·½·¨2*/
      for(uint8 i=0;i<whiteline;i++)		//É¾Í»³ö½ÇÒÔÉÏÒÔÓÒµÄ±ßÏß
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
          circle_x=j-1;		//¼ÇÂ¼ÓÒÍ¼µÚÒ»ĞĞ´Ó×óµ½ÓÒ£¬ÓÒ±ê×¼ÏßÒÔ×óµÚÒ»¸öºÚµãÎ»ÖÃ
          circle_y=0;
          break;
        }
      }
      if(circle_x==100)	//ÓÒÍ¼µÚÒ»ĞĞÈ«°×
      {
        circle_x=rightline[0];	//ÉèÎª±ê×¼ÏßÎ»ÖÃ
        circle_y=0;
      }
      deletearea(conor_y,conor_x,rightmap,right);		//ÓÖÉ¾Ò»±é		å£¿å£¿å£¿å£¿å£¿å£¿å£¿
      //²¹Ïß
      if(circle_x==100&&circle_y==100){circle_x=XX;circle_y=0;}
      float k=(float)(circle_x-conor_x)/(conor_y-circle_y);			//¼ÆËãĞ±ÂÊ
      for(uint8 i=circle_y;i<=conor_y-2*(conor_y-circle_y)/3;i++)	//Ö»²¹1/3
      {
        rightmap[i][(int)(circle_x-(i-circle_y)*k)]=2;		//²¹Ïß
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
      if(basemap[i][0] && leftmap[i][0] != 1&&rightmap[i][0]!=1)//É¨Ãè¶¥²¿Í¼ÏñÈë×óÍ¼
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
      /*·½·¨1*/
//      int nn=1;
//      for(uint8 j=0;j<X;j++)
//      {
//        for(uint8 i=0;i<Y;i++)
//        {
//          if(leftmap[YY-i][j]==0)
//          {
//            for(uint8 m=i+1;m<Y;m++)
//            {
//              if(leftmap[YY-m][XX-j]==2&&circle_x==100&&circle_y==100)//¼ÇÂ¼Ò»´Î±ã²»ÔÙ¼ÇÂ¼
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
      /*·½·¨2*/
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
      //²¹Ïß
      if(circle_x==100&&circle_y==100){circle_x=0;circle_y=0;}//ÔÚ·½·¨Ò»Òª¼Ó
      float k=(float)(circle_x-conor_x)/(conor_y-circle_y);
      for(uint8 i=circle_y;i<=conor_y-2*(conor_y-circle_y)/3;i++)
      {
        leftmap[i][(int)(circle_x-(i-circle_y)*k)]=2;
      }
    }
    
}
void ring_out()//¸Ãº¯ÊıµÄÉ¾Ïß¹¦ÄÜÍ¬Ê±½â¾öÁË³ö»·É¾ÏßÎÊÌâºÍÊ®×ÖÉ¾Ïß²»¸É¾»µÄÎÊÌâ
{
  int down_x1=0;//×ó
  int down_x2=39;//ÓÒ
  int down = -1;
  for(int b=0;b<=YY;b++)//ËÑË÷71Ğ±ĞĞ
  {
    for(uint8 j=0;j<X;j++)    
    {
      if(j+b>YY) break;		//Ğ±ËÑË÷×ó±ß²¿·Ö
      if(leftmap[j+b][j]!=1)//°×É«
      {
          for(uint8 m=j+1;m<X;m++)
          {
            if(m+b>YY) break;
            if(leftmap[m+b][m]==1)//ºÚÉ«
            {
              for(uint8 n=m+1;n<X;n++)
              {
                if(n+b>YY) break;
                if(leftmap[n+b][n]!=1)//°×É«
                {
                  if(down==-1)  
                  {
                    down = b;			//¼ÇÂ¼É¨µ½µÄ½ÇµÄÎ»ÖÃ
                    down_x1=m;//×ó
                    down_x2=n;//ÓÒ
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
  if(down>=0&&down_x2-down_x1<6)		//¿í¶È·ûºÏÌõ¼ş£¬ÇÒÉ¨µ½ÁË½Ç
  {
    /****É¾Ïß´¦Àí****/
    for(int b=down;b<=YY;b++)		//½ÇµÄµãËùÔÚĞĞÒÔÉÏ×ó±ßÒ»¿éÈı½ÇĞÍµÄ±ßÏß±»É¾
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
         /*****²¹Ïß´¦Àí***********/
//    for(uint8 i=0;i<Y;i++)
//    {
//      uint8 j;
//      for(j=0;j<X;j++)
//      {
//        if(leftmap[i][j]!=0)
//          break;
//      }
//      if(j<X)
//      {  //±ê×¼Ïß²¹Æëµ×²¿±ßÏß¿ÕÈ±
//        if(i>0)
//        { for(uint8 m=0;m<i*line_add;m++)//²¹Ò»²¿·Ö
//          {
//           // leftmap[m][leftline[m]]=2;
//          }
//        }
//        break;
//      }      
//    }

  }
  //ËÑË÷ÓÒÍ¼¸ÄÓÃint·ÀÖ¹ËÀÑ­»·
  down_x1=0;//×ó
  down_x2=39;//ÓÒ
  down = -1;
  for(int b=39;b<=118;b++)
  {
    for(int j=XX;j>=0;j--)    
    {
      if(-j+b>YY) break;
      if(rightmap[-j+b][j]!=1)//°×É«
      {
          for(int m=j-1;m>=0;m--)
          {
            if(-m+b>YY) break;
            if(rightmap[-m+b][m]==1)//ºÚÉ«
            {
              for(int n=m-1;n>=0;n--)
              {
                if(-n+b>YY) break;
                if(rightmap[-n+b][n]!=1)//°×É«
                {
                  if(down==-1)  
                  {
                    down = b;
                    down_x2=m;//ÓÒ
                    down_x1=n;//×ó
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
    /****É¾Ïß´¦Àí****/
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
         /*****²¹Ïß´¦Àí***********/
//    for(uint8 i=0;i<Y;i++)
//    {
//      uint8 j;
//      for(j=0;j<X;j++)
//      {
//        if(rightmap[i][j]!=0)
//          break;
//      }
//      if(j<X&&i>0)
//      {  //±ê×¼Ïß²¹Æëµ×²¿±ßÏß¿ÕÈ±
//        for(uint8 m=0;m<i*line_add;m++)
//        {
//         // rightmap[m][rightline[m]]=2;
//        }
//        break;
//      }
//    }

  }
}

void getline(uint8 src[][X], uint8 width[Y], float kb[2])		//×óÓÒÍ¼¸ø×óÓÒÏß¸³Öµ
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
        //»»Ëã³ÉÊµ¼ÊÆ«²î
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

uint8 obstacle()//Ñ°ÕÒÕÏ°­
{
  uint8 obmap[Y][X] = {0};
 // uint8 stmap[Y][X] = {0};//¼ÇÂ¼Í£³µÏßµÄmap
  uint8 deletemap[Y][X] = {0};
    
  for(uint8 i = Y * 2 / 3; i < Y; ++ i)//ÔÚÍ¼ÏñµÄÈı·ÖÖ®¶şÒÔÉÏ²¹×óÓÒ±ßÑØµÄÈ±Ê§µÄºÚÉ«
  {
    if(basemap[i][0] && leftmap[i][0] != 1 && rightmap[i][0] != 1 && deletemap[i][0] != 1)
      searchmap(0, i, deletemap, basemap);
    if(basemap[i][XX] && leftmap[i][XX] != 1 && rightmap[i][XX] != 1 && deletemap[i][XX] != 1)
      searchmap(XX, i, deletemap, basemap);
  }
  for(uint8 i = 0; i < X; ++ i)//²¹Í¼ÏñÉÏÏÂ±ßÑØÈ±Ê§µÄºÚÉ«
  {
    if(basemap[YY][i] && leftmap[YY][i] != 1 && rightmap[YY][i] != 1 && deletemap[YY][i] != 1)		//Ä¿Ç°Ö»ÕÒÉÏ±ß
      searchmap(i, YY, deletemap, basemap);
//    if(basemap[0][i] && leftmap[0][i] != 1 && rightmap[0][i] != 1 && deletemap[0][i] != 1)
//      searchmap(i, 0, deletemap, basemap);
  }

  for(uint8 i = 0; i < Y; ++ i)//Ñ°ÕÒÈüµÀÖĞµÄºÚÉ«ÇøÓò²¢Ğ´ÈëobmapÖĞ
  {
    for(uint8 j = 0; j < X; ++ j)
    {
      if(basemap[i][j] && leftmap[i][j] != 1 && rightmap[i][j] != 1 && deletemap[i][j] == 0)
      {
        obmap[i][j] = 1;		//Í¼ÏñÌØµãÎªºÚÉ«ÕÏ°­ËÄÖÜ»·ÈÆÒ»È¦°×É«
     //   stmap[i][j] = 1;//¿½±´µ½Í£³µÏßµØÍ¼ÖĞ
      }
    }
  }
  
  for(uint8 i = 0; i < 70; ++ i)//ÔÚ0µ½70·¶Î§ÄÚÑ°ÕÒ
  {
    uint8 left = 0;
    uint8 right = XX;
    for(uint8 j = 0; j < X; ++ j)//´Ó×óÏòÓÒÉ¨ÈüµÀ×ó±ß½ç
    {
      if(basemap[i][j] == 0)
      {
        left = j;
        break;
      }
    }
    for(uint8 j = 0; j < X; ++ j)//´ÓÓÒÏò×óÉ¨ÈüµÀÓÒ±ß½ç
    {
      if(basemap[i][XX - j] == 0)
      {
        right = XX - j;
        break;
      }
    }
    
    for(uint8 j = left; j <= right; ++ j)//ÔÚÈüµÀ±ß½çÄÚ´Ó×óÏòÓÒÉ¨
    {
      if(obmap[i][j])//ÔÚÕÏ°­Í¼ÀïÉ¨µ½ºÚÉ«ÇøÓò
      {
        int num = 0;
        searchmap(j, i, obstaclemap, obmap);//´ÓÉ¨µ½µÄºÚÉ«µã¿ªÊ¼É¨µ½È«²¿ºÚÉ«ÇøÓò
        for(uint8 i = 0; i < Y; ++ i)//ÊıºÚÉ«ÇøÓòµÄ´óĞ¡
        {
          for(uint8 j = 0; j < X; ++ j)
          {
            if(obstaclemap[i][j])
            {
              num ++;
             // stmap[i][j] = 3;//½«Í£³µÏßµØÍ¼µÄÏàÓ¦Î»ÖÃÖÃÎª3£¬ÎªÊ¶±ğÉ²³µÏß×ö×¼±¸
            }
          }
        }
        
        uint8 downy = i;//¼ÇÂ¼ÏÂºÚÉ«ÇøÓòµÄ×îÏÂ±ß
        
        if(!ramp_flag && runTime >=4) //,ÆğÅÜÈıÃëÄÚ²»Ê¶±ğÆğÅÜÏß £¬±ğÍüÁË¿ª³µÑÓÊ±&&mystop_time>=300
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
        }//ÆğÅÜÏßÅĞ¶Ï½áÊø
//        display_realNum(site_numSign,site_numValue,num);
//        display_realNum(site_obdySign,site_obdyValue,obnum[downy]);
        if(num > obnum[downy]/1.7 && num <obnum[downy] + 150) //Èç¹ûÃæ»ı·ûºÏÒªÇóµÄ»°¿ªÊ¼ÅĞ¶ÏÎ»ÖÃ
        {
          uint8 direction = 0;//·½ÏòÇåÁã
          for(uint8 k = downy; k < Y; ++ k)//´ÓºÚÉ«ÇøÓòÏÂÃæ¿ªÊ¼ÏòÉÏÉ¨
          {
            int left = 0;
            int right = XX;
            
            for(uint8 m = 0; m < X; ++ m)	//¼ÇÂ¼×óÓÒ±ß
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
            if(left == 0)//×ó±ßÈüµÀ¶ªÊ§											
            {
              for(uint8 m = 1; m < max; ++ m)
              {
                if(right - m  < 0 || left + m > XX)//·ÀÖ¹Êı×éÔ½½ç
                  break;
                if(obstaclemap[k][right - m])
                {
                  direction = 2; //ÕÏ°­ÔÚÓÒ±ß
                  break;
                }
                if(obstaclemap[k][left + m])
                {
                  if(((float)(right-left-m) * xx[i])>23)//·ÀÖ¹ÎóÅĞÎª×ó±ßÕÏ°­
                  {
                    direction = 1; //ÕÏ°­ÔÚ×ó±ß
                    break;
                  }
                  else 
                  {
                    direction = 2; //ÕÏ°­ÔÚÓÒ±ß
                    break;
                  }
                }
              }
            }
	else if(right == XX)//ÓÒ±ßÈüµÀ¶ªÊ§
            {
              for(uint8 m = 1; m < max; ++ m)
              {
                if(right - m  < 0 || left + m > XX)//·ÀÖ¹Êı×éÔ½½ç
                  break;
                if(obstaclemap[k][left + m])
                {
                  direction = 1; //ÕÏ°­ÔÚ×ó±ß
                  break;
                }
                if(obstaclemap[k][right - m])
                {
                  if(((float)(right-m-left) *xx[i])>23)//·ÀÖ¹ÎóÅĞÎªÓÒ±ßÕÏ°­
                  {
                    direction = 2; //ÕÏ°­ÔÚÓÒ±ß
                    break;
                  }
                  else 
                  {
                    direction = 1; //ÕÏ°­ÔÚ×ó±ß
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
                  direction = 1; //ÕÏ°­ÔÚ×ó±ß
                  break;
                }
                if(obstaclemap[k][right - m])
                {
                  direction = 2; //ÕÏ°­ÔÚÓÒ±ß
                  break;
                }
              }
            }
            if(direction == 1)//ÕÏ°­ÔÚ×ó±ß
            {
              
           //   ob_direction = 1;
              
              uint8 x = X;
              uint8 y = Y;				//¼ÇÂ¼ÏÂ±ßÑØYÎ»ÖÃ
              
              for(uint8 i = 0; i < Y; ++ i)
              {
                for(uint8 j = 0; j < X; ++ j)//Ñ°ÕÒÕÏ°­µÄÏÂ±ßÑØ
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
              for(uint8 i = 0; i < X; ++ i)//Ñ°ÕÒÕÏ°­µÄÓÒ±ßÑØ
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
              
              int add = (int)x - (int)leftline[y];//¼ÆËãÕÏ°­Óë±ß½çµÄÎ»ÖÃ
              
              for(uint8 i = 0; i < X; ++ i)
              {
                for(uint8 j = 0; j < Y; ++ j)//Çå³ıÓÒÍ¼µÄ±ß½ç
                {
                  if(rightmap[j][i] == 2)
                    rightmap[j][i] = 0;
                }
              }
              
              for(uint8 i = y; i < Y; ++ i)
              {
                for(uint8 j = 0; j < X; ++ j)//Çå³ı×óÍ¼ÕÏ°­ÒÔÉÏµÄ±ß½ç
                {
                  if(leftmap[i][j] == 2)
                     leftmap[i][j] = 0;
                }
             //   if(!gpio_get(PTE8))   LCD_num(site11,(uint32)((float)(((lpp-flpp)/(-70))*downy+lpp)*100), GREEN,RED);
                int position = (int)(leftline[i] + add * ((float)( (flpp/70) * downy+lpp)));//¼ÆËãĞÂµÄ±ß½çºá×ø±ê
                if(position < 0)
                  position = 0;
                else if(position > XX)
                  position = XX;
                leftmap[i][position] = 2;//Ë¢½øĞÂµÄ±ß½ç
              }
              
              return 1;
            }else if(direction == 2)
            {
              
            //  ob_direction = 2;
              
              uint8 x = X; 
              uint8 y = Y;
              

              
              for(uint8 i = 0; i < Y; ++ i)//Ñ°ÕÒÕÏ°­µÄÏÂ±ßÑØ
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
              
               for(uint8 i = 0; i < X; ++ i)//Ñ°ÕÒÕÏ°­µÄ×ó±ßÑØ
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