#include "fuzzy_pid.h"
float ufl(float x,float a,float b);//��������������ģ����
float uf(float x,float a,float b,float c);//��������������ģ����
float ufr(float x,float a,float b);//��������������ģ����

float cufl(float x,float a,float b);//�����η�ģ����
float cuf(float x,float a,float b,float c);//�����η�ģ����
float cufr(float x,float a,float b);//�����η�ģ����
int f_p[7][7]={ //kp�����;
                {PB,PB,PM,PM,PS,PS,ZO},
                {PB,PB,PM,PM,PS,ZO,ZO},
                {PM,PM,PS,PS,ZO,NS,NM},
                {PM,PS,PS,ZO,NS,NM,NM},
                {PS,PS,ZO,NS,NS,NM,NM},
                {ZO,ZO,NS,NM,NM,NM,NB},
                {ZO,NS,NS,NM,NM,NB,NB}
              };

int f_i[7][7]={ //ki����:����<-  ->����         //���
                //  ����: ���ϵ��£���������   
                 {PB,PS,PS,ZO,PS,PM,PM},
                 {PB,PS,PS,ZO,PS,PM,PM},
                 {PB,PM,PM,ZO,NS,PS,PM},
                 {PB,PM,PM,ZO,NS,PS,PM},
                 {PB,PB,PM,ZO,NS,NS,PM},
                 {PB,PB,PB,ZO,NM,NS,PS},
                 {PB,PB,PB,ZO,NM,NS,PS}
              };

int f_d[7][7]={
                 {PS,PS,ZO,ZO,ZO,PB,PB},
                 {NS,NS,NS,NS,ZO,NS,PM},
                 {NB,NB,NM,NS,ZO,PS,PM},
                 {NB,NM,NM,NS,ZO,PS,PM},
                 {NB,NM,NM,NS,ZO,PS,PS},
                 {NM,NS,NS,NS,ZO,PS,PS},
                 {PS,ZO,ZO,ZO,ZO,PB,PB}
              };



float fuzzy_kp(float e,float ec)//13.5,0
{
  float es[7];//����������
  float ecs[7];
  float form[7][7]; //�����ȱ�
  int i,j;
  
  int a=0,b=0;
  float lsd;
  int p;
  float detkp;
  //��������
  es[NB]=ufl(e,-3,-1);
  es[NM]=uf(e,-3,-2,0);
  es[NS]=uf(e,-3,-1,1);
  es[ZO]=uf(e,-2,0,2);
  es[PS]=uf(e,-1,1,3);
  es[PM]=uf(e,0,2,3);
  es[PB]=ufr(e,1,3);//1

  ecs[NB]=ufl(ec,-3,-1);
  ecs[NM]=uf(ec,-3,-2,0);
  ecs[NS]=uf(ec,-3,-1,1);
  ecs[ZO]=uf(ec,-2,0,2);//1
  ecs[PS]=uf(ec,-1,1,3);
  ecs[PM]=uf(ec,0,2,3);
  ecs[PB]=ufr(ec,1,3);
  
  for(i=0;i<7;i++)
  {
    for(j=0;j<7;j++)
    {
      form[i][j]=((es[j]<ecs[i])?es[j]:ecs[i]); //ȡ����
    }
  }
  
  
  for(i=0;i<7;i++)
  {
    for(j=0;j<7;j++)
    {
      if(form[a][b]<form[i][j])         //��λ���������
      {
        a=i;
        b=j;
      }
    }
  }
  
  lsd=form[a][b];  //������
  p=f_p[a][b];      //ģ����
 
  if(p==NB)                  //��p��[-6,6]
  detkp=cufl(lsd,-3,-1);        
  else if(p==NM)
  detkp=cuf(lsd,-3,-2,0);
  else if(p==NS)
  detkp=cuf(lsd,-3,-1,1);
  else if(p==ZO)
  detkp=cuf(lsd,-2,0,2);
  else if(p==PS)
  detkp=cuf(lsd,-1,1,3);
  else if(p==PM)
  detkp=cuf(lsd,0,2,3);
  else if(p==PB)
  detkp=cufr(lsd,1,3);
  
  return detkp;
}

float fuzzy_ki(float e,float ec)
{
  float es[7];//����������
  float ecs[7];
  float form[7][7]; //�����ȱ�
  int i,j;
  
  int a=0,b=0;
  float lsd;
  int p;
  float detki;
  //��������
  es[NB]=ufl(e,-3,-1);
  es[NM]=uf(e,-3,-2,0);
  es[NS]=uf(e,-3,-1,1);
  es[ZO]=uf(e,-2,0,2);
  es[PS]=uf(e,-1,1,3);
  es[PM]=uf(e,0,2,3);
  es[PB]=ufr(e,1,3);
  
  ecs[NB]=ufl(ec,-3,-1);
  ecs[NM]=uf(ec,-3,-2,0);
  ecs[NS]=uf(ec,-3,-1,1);
  ecs[ZO]=uf(ec,-2,0,2);
  ecs[PS]=uf(ec,-1,1,3);
  ecs[PM]=uf(ec,0,2,3);
  ecs[PB]=ufr(ec,1,3);
  
  for(i=0;i<7;i++)
  {
    for(j=0;j<7;j++)
    {
      form[i][j]=((es[j]<ecs[i])?es[j]:ecs[i]); //ȡ����
    }
  }
  
  for(i=0;i<7;i++)
  {
    for(j=0;j<7;j++)
    {
      if(form[a][b]<form[i][j])         //��λ���������
      {
        a=i;
        b=j;
      }
    }
  }
  lsd=form[a][b];  //������
  p=f_i[a][b];      //ģ����
  
  if(p==NB)                  //��i��[-0.18,0.18]
  detki=cufl(lsd,-0.24,-0.08);        
  else if(p==NM)
  detki=cuf(lsd,-0.24,-0.16,0);
  else if(p==NS)
  detki=cuf(lsd,-0.24,-0.08,0.08);
  else if(p==ZO)
  detki=cuf(lsd,-0.16,0,0.16);
  else if(p==PS)
  detki=cuf(lsd,-0.08,0.08,0.24);
  else if(p==PM)
  detki=cuf(lsd,0,0.16,0.24);
  else if(p==PB)
  detki=cufr(lsd,0.08,0.24);
  
  return detki;
}

float fuzzy_kd(float e,float ec)
{
  float es[7];//����������
  float ecs[7];
  float form[7][7]; //�����ȱ�
  int i,j;
  
  int a=0,b=0;
  float lsd;
  int p;
  float detkd;
  //��������
  es[NB]=ufl(e,-3,-1);
  es[NM]=uf(e,-3,-2,0);
  es[NS]=uf(e,-3,-1,1);
  es[ZO]=uf(e,-2,0,2);
  es[PS]=uf(e,-1,1,3);
  es[PM]=uf(e,0,2,3);
  es[PB]=ufr(e,1,3);
  
  ecs[NB]=ufl(ec,-3,-1);
  ecs[NM]=uf(ec,-3,-2,0);
  ecs[NS]=uf(ec,-3,-1,1);
  ecs[ZO]=uf(ec,-2,0,2);
  ecs[PS]=uf(ec,-1,1,3);
  ecs[PM]=uf(ec,0,2,3);
  ecs[PB]=ufr(ec,1,3);
  
  for(i=0;i<7;i++)
  {
    for(j=0;j<7;j++)
    {
      form[i][j]=((es[j]<ecs[i])?es[j]:ecs[i]); //ȡ����
    }
  }
  
  for(i=0;i<7;i++)
  {
    for(j=0;j<7;j++)
    {
      if(form[a][b]<form[i][j])         //��λ���������
      {
        a=i;
        b=j;
      }
    }
  }
  lsd=form[a][b];  //������
  p=f_d[a][b];      //ģ����
  
  if(p==NB)                  //��d��[-0.18,0.18]
  detkd=cufl(lsd,-0.09,-0.03);        
  else if(p==NM)
  detkd=cuf(lsd,-0.09,-0.06,0);
  else if(p==NS)
  detkd=cuf(lsd,-0.09,-0.03,0.03);
  else if(p==ZO)
  detkd=cuf(lsd,-0.06,0,0.06);
  else if(p==PS)
  detkd=cuf(lsd,-0.03,0.03,0.09);
  else if(p==PM)
  detkd=cuf(lsd,0,0.06,0.09);
  else if(p==PB)
  detkd=cufr(lsd,0.03,0.09);
  
  return detkd;
}

float ufl(float x,float a,float b)
{
  if(x<=a)return 1;
  else if(a<x&&x<=b)return ((b-x)/(b-a));
  else if(x>b)return 0;
  
  return 0;     //�������뾯��
}

float uf(float x,float a,float b,float c)
{
  if(x<=a)return 0;
  else if(a<x&&x<=b)return ((x-a)/(b-a));
  else if(b<x&&x<=c)return ((c-x)/(c-b));
  else if(x>c)return 0;
  
  return 0;     //�������뾯��
}

float ufr(float x,float a,float b)
{
  if(x<=a)return 0;
  else if(a<x&&x<=b)return ((x-a)/(b-a));
  else if(x>b)return 1;
  
  return 0;     //�������뾯��
}

float cufl(float x,float a,float b)
{
  return (b-(b-a)*x);
}

float cuf(float x,float a,float b,float c)
{
  float y,z;
  y=a+(b-a)*x;
  z=c-(c-b)*x;
  return (y+z)/2;
}

float cufr(float x,float a,float b)
{
  return (a+(b-a)*x);
}

