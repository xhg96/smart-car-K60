#include "common.h"
#include "include.h"
#include "motor.h"
#include "m_Motro.h"
/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

uint8 data_to_send[32];	//�������ݻ��棬ÿ��int����+8
extern float Pitch,Roll,Yaw;
extern uint16 absde[3];
extern short gyro[3];

//*****************************���ͺ���********************************///////
//����  ����Ŀ���ٶȺ�ʵ���ٶȵ���λ��
//aspeed   Ϊ��ǰĿ���ٶ�
//nspeed   Ϊ��ǰʵ���ٶ�
//pwm      ���PWM
///*********************************************************************//////
void Send_Speed_Info()
{
	uint32 _cnt=0;

	data_to_send[_cnt++]=0xAA;      //֡ͷ
	data_to_send[_cnt++]=0xAA;      //֡ͷ
	data_to_send[_cnt++]=0xF1;      //������
	data_to_send[_cnt++]=0;         //���ݰ�������0�����ֵ
          //ȡ��16λ��Ӱ�죬ʵ��ֵ������ô��
	data_to_send[_cnt++]=BYTE1(SPD.aimSpeed);  //aim_speed          1
	data_to_send[_cnt++]=BYTE0(SPD.aimSpeed);
        
	data_to_send[_cnt++]=BYTE1(SPD.nowSpeed);  //now_speed        2
	data_to_send[_cnt++]=BYTE0(SPD.nowSpeed);

        data_to_send[_cnt++]=BYTE3(SPD.AC_P);       //SPD.AC_P      3
	data_to_send[_cnt++]=BYTE2(SPD.AC_P);
        data_to_send[_cnt++]=BYTE1(SPD.AC_P);
	data_to_send[_cnt++]=BYTE0(SPD.AC_P);
        
        data_to_send[_cnt++]=BYTE1(Motro_PID.PID_Out);        //Motro_PID.PID_Out       4
	data_to_send[_cnt++]=BYTE0(Motro_PID.PID_Out);        
        
        data_to_send[_cnt++]=BYTE1(SPD.speed);//speed   5
	data_to_send[_cnt++]=BYTE0(SPD.speed);
        
        data_to_send[_cnt++]=BYTE1(Servo_PID.error);            //Servo_PID.error          6
	data_to_send[_cnt++]=BYTE0(Servo_PID.error);  
        
     //   data_to_send[_cnt++]=BYTE3(absde[0]);       //absde[0]          7
	//data_to_send[_cnt++]=BYTE2(absde[0]);
        data_to_send[_cnt++]=BYTE1(Servo_PID.PID_Out);//          7
	data_to_send[_cnt++]=BYTE0(Servo_PID.PID_Out); 
        
        int temp=(int)(Pitch*10);
        data_to_send[_cnt++]=BYTE3(temp);       //Pitch      8
	data_to_send[_cnt++]=BYTE2(temp);
        data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
                int tempg = gyro[0];
        data_to_send[_cnt++]=BYTE3(tempg);       //     9
	data_to_send[_cnt++]=BYTE2(tempg);
        data_to_send[_cnt++]=BYTE1(tempg);
	data_to_send[_cnt++]=BYTE0(tempg);
                tempg = gyro[1];
        data_to_send[_cnt++]=BYTE3(tempg);       //     10
	data_to_send[_cnt++]=BYTE2(tempg);
        data_to_send[_cnt++]=BYTE1(tempg);
	data_to_send[_cnt++]=BYTE0(tempg);
                tempg = gyro[2];
        data_to_send[_cnt++]=BYTE3(tempg);       //     11
	data_to_send[_cnt++]=BYTE2(tempg);
        data_to_send[_cnt++]=BYTE1(tempg);
	data_to_send[_cnt++]=BYTE0(tempg);
//        
//        data_to_send[_cnt++]=BYTE3(Yaw);       //Yaw      10
//	data_to_send[_cnt++]=BYTE2(Yaw);
//        data_to_send[_cnt++]=BYTE1(Yaw);
	//data_to_send[_cnt++]=BYTE0(Yaw);
	data_to_send[3] = _cnt-4;       //���ݰ�����
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;       //У��λ
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}

void SD5_Info(){

  uint32 _cnt=0;

	data_to_send[_cnt++]=0xAA;      //֡ͷ
	data_to_send[_cnt++]=0xAA;      //֡ͷ
	data_to_send[_cnt++]=0xF1;      //������
	data_to_send[_cnt++]=0;         //���ݰ�������0�����ֵ

        
        data_to_send[_cnt++]=BYTE1(Servo_PID.PID_Out);//   Servo_PID.PID_Out    1
	data_to_send[_cnt++]=BYTE0(Servo_PID.PID_Out);
        
        data_to_send[_cnt++]=BYTE1(Servo_PID.error);            //Servo_PID.error          2
	data_to_send[_cnt++]=BYTE0(Servo_PID.error); 
                
        data_to_send[_cnt++]=BYTE3(Din_part);       //     D_inpart         3
	data_to_send[_cnt++]=BYTE2(Din_part);
        data_to_send[_cnt++]=BYTE1(Din_part);
	data_to_send[_cnt++]=BYTE0(Din_part);
        
        data_to_send[_cnt++]=BYTE3(Dout_part);       //     D_outpart         4
	data_to_send[_cnt++]=BYTE2(Dout_part);
        data_to_send[_cnt++]=BYTE1(Dout_part);
	data_to_send[_cnt++]=BYTE0(Dout_part);
        

        data_to_send[_cnt++]=BYTE3(kb_send[0]);            //   k        5
	data_to_send[_cnt++]=BYTE2(kb_send[0]); 
        data_to_send[_cnt++]=BYTE1(kb_send[0]);   
	data_to_send[_cnt++]=BYTE0(kb_send[0]);
        
        data_to_send[_cnt++]=BYTE3(kb_send[1]);            //   b         6
	data_to_send[_cnt++]=BYTE2(kb_send[1]); 
        data_to_send[_cnt++]=BYTE1(kb_send[1]);        
	data_to_send[_cnt++]=BYTE0(kb_send[1]);        
          

	data_to_send[3] = _cnt-4;       //���ݰ�����
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;       //У��λ
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}
void Send_s0(int32 s)
{
	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF2;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(s);
	data_to_send[_cnt++]=BYTE2(s);

	data_to_send[_cnt++]=BYTE1(s);
	data_to_send[_cnt++]=BYTE0(s);
        

	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}
void Send2int(int s1,int s2,uint8 zhen)
{
	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=zhen;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(s1);
	data_to_send[_cnt++]=BYTE2(s1);
	data_to_send[_cnt++]=BYTE1(s1);
	data_to_send[_cnt++]=BYTE0(s1);
        
        data_to_send[_cnt++]=BYTE3(s2);
	data_to_send[_cnt++]=BYTE2(s2);
        data_to_send[_cnt++]=BYTE1(s2);
	data_to_send[_cnt++]=BYTE0(s2);

	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}
void Send3float(float s1,float s2,float s3,uint8 zhen)
{
	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=zhen;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(s1);
	data_to_send[_cnt++]=BYTE2(s1);
	data_to_send[_cnt++]=BYTE1(s1);
	data_to_send[_cnt++]=BYTE0(s1);
        
        data_to_send[_cnt++]=BYTE3(s2);
	data_to_send[_cnt++]=BYTE2(s2);
        data_to_send[_cnt++]=BYTE1(s2);
	data_to_send[_cnt++]=BYTE0(s2);

        data_to_send[_cnt++]=BYTE3(s3);
	data_to_send[_cnt++]=BYTE2(s3);
        data_to_send[_cnt++]=BYTE1(s3);
	data_to_send[_cnt++]=BYTE0(s3);
        
	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}
void Send_float(float s,uint8 zhen)
{
  	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=zhen;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(s);
	data_to_send[_cnt++]=BYTE2(s);

	data_to_send[_cnt++]=BYTE1(s);
	data_to_send[_cnt++]=BYTE0(s);

	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}
void Send_int(int s,uint8 zhen)
{
  	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=zhen;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(s);
	data_to_send[_cnt++]=BYTE2(s);

	data_to_send[_cnt++]=BYTE1(s);
	data_to_send[_cnt++]=BYTE0(s);

	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����  
}
void Send_uchar(uint8 s,uint8 zhen)
{
  	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=zhen;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=s;

	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����  
}
void Send_deviation(int32 dei)
{
	uint32 _cnt=0;
	
        
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF3;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(dei);
	data_to_send[_cnt++]=BYTE2(dei);

	data_to_send[_cnt++]=BYTE1(dei);
	data_to_send[_cnt++]=BYTE0(dei);

	data_to_send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
        
	uart_putbuff (VCAN_PORT,data_to_send, _cnt);  //����
}

void uart2send()
{
  //Send_s0((int32)s0);
 // Send_Speed_Info(aim_speed,now_speed);
}