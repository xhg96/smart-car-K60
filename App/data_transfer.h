#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H


void Send_Speed_Info();
void Send_Speed_s0(int32 s);
void Send_s0(int32 s);
void SD5_Info();
void Send_deviation(int32 dei);
void uart2send();
void Send_int(int s,uint8 zhen);
void Send_float(float s,uint8 zhen);
void Send_int(int s,uint8 zhen);
void Send_uchar(uint8 s,uint8 zhen);
void Send2int(int s1,int s2,uint8 zhen);
void Send3float(float s1,float s2,float s3,uint8 zhen);
#endif
