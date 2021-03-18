#ifndef __M_STRUCTURE__
#define __M_STRUCTURE__


#define FREE_CAR   0
#define LEADER_CAR 1
#define FOLLOW_CAR 2

// 双车状态标志位
#define LOST_ULTRASOUND_MASK 0x01    //超声波信号丢失
#define READY_OVERTAKE_MASK  0X02    //准备超车(双车处于超车状态中)
#define FINISH_OVERTAKE_MASK 0X04    //超车完成
#define FOLLOWER_START_MASK  0x08    //后车启动标志(开车时前后车大于一定距离后车自动开启)
#define LEADER_BREAKDOWN_MASK 0X10   //前车抛锚通知后车
#define FOLLOW_BREAKDOWN_MASK 0X20   //后车抛锚通知前车


// 前车路况标志位
#define STRAIGHTWAY_MASK    0X01    //前方长直道
#define BRAKE_MASK 	     0x02    //前车正在刹车
#define RAMP_MASK		     0X04    //前方坡道
#define CIRCLE_MASK	     0X08    //前方圆环
#define LEFTOBSTRACLE_MASK  0X10    //左障碍
#define RIGHTOBSTRACLE_MASK 0X20    //右障碍
#define STOPLINE_MASK       0X40    //停止线


typedef struct {

	vuint8 beepON;
	vuint8 stop;    //1故障停车，2临时停车
	vuint8 lowVoltage;    //低电量标志
	vuint8 upLoad_SpeedWave;
	
}Flag_st;  //封装各种标志的结构体


typedef struct
{
	uint16 car_ID;          //前后车ID
	vuint8 flag1;           //用于协调超车的一些标志位
	vuint8 flag_tracktype;  //前车发给后车的赛道类型信息
	
	vuint16 distance_cm;    //双车之间的距离
	uint16  aim_distance;   //双车的目标距离(用于距离PID控制)
	vuint16 follower_IntegralDistance;  //丢失超声波信号后编码器积分的距离(后车)
	vuint16 lastUpdata_time;  //记录上次接收到超声波信号的时间，以此来判断本次是否接收到超声波信号(因为超声波接收放在外部中断中，只能通过此种方法判断)

}DOUBLECAR_INFO_st;  //封装用于双车协调的信息的结构体


extern Flag_st Flag;
extern DOUBLECAR_INFO_st DoubleCar_Info;   //双车运行信息


#endif
