#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    



	
//CAN1接收RX0中断使能

#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.
#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

#define Get_Current_Velocity_Position_Mode  0x0B
#define Get_CTL1_CTL2_Mode                  0x0C

void CAN1_Configuration(void);

void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

void CAN_RoboModule_DRV_FeedbackMode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_Get_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_Get_CTL1_CTL2_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);

void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number);

extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];


								    
typedef struct 
{
	struct 
	{
		float p;
		float i;
		float d;		
	}motor[4];
}TYPE_Motor_Control_PID;
extern TYPE_Motor_Control_PID Motor_PID;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
void Motor_PID_Control(int16_t lf,int16_t rf,int16_t lb,int16_t rb);
int32_t SendSpeed(int32_t des1,int32_t des2,int32_t des3);
uint8_t CAN_Send(uint16_t StdId,int16_t lf,int16_t rf,int16_t lb);

u8 Position_Control(float X_target,float Y_target,float Theta_target);

u8 Trajectory_Planning1();//开始拿球
u8 Trajectory_Planning1_();//开始拿球
u8 Trajectory_Planning2();//去第一递位
u8 Trajectory_Planning3();//退后等待
u8 Trajectory_Planning3_5();//去第二递位（递彩球2）
u8 Trajectory_Planning4();//回去拿金球
u8 Trajectory_Planning5();//去第二递位（递金球）

u8 Pos_Laser_Control(float aim);
void poly(double s);
int find(float a,float b[],int len);
void GetMotorSpeed(float Vx,float Vy,float W);
float power(float base, int n);
//void Pos_Ctrl(float Des_X, float Des_Y, float Des_Angle, int time)；


#endif



