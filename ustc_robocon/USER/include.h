#ifndef __INCLUDE_H
#define __INCLUDE_H	 

//系统自带头文件

#include "sys.h"
#include "can.h"
#include "led.h"
#include "lcd.h"
#include "adc.h"
#include "key.h"
#include "can.h"
#include "pwm.h"
#include "show.h"
#include "math.h"
#include "delay.h"
#include "timer.h"
#include "delay.h"	 
#include "stdio.h"
#include "usart.h"
#include "string.h"	 
#include "stdarg.h"	 
#include "stm32f4xx.h"


//用户自定义头文件
#include "pstwo.h"
#include "motion.h"
#include "usart3.h"
#include "usart4.h"
//#include "show.h"
//#include "ESP8266.h"
//#include "rs485.h"
//#include "AQMD.h"
//#include "motor.h"
//#include "cmd.h"
//#include "LCD_Show.h"
//#include "pstwo.h"
//全场定位系统参数
typedef signed short int16;
extern double aim_speed;
extern float pos_x,pos_y,velo_x;
extern float zangle,xangle,yangle;
extern float w_z;
//extern int motor1,motor2,motor3;

//CAN通信参数
extern short Real_Current_Value[4] ;
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];
extern double Target_Pos_Y;
extern double Target_Pos_X;

extern double Init_Pos_X;
extern double Init_Angle;
extern double Init_Pos_Y;

extern u8 Data[9];
//extern short Real_Current_Value[3];
//extern short Real_Velocity_Value[3];
//extern long  Real_Position_Value[3];
//extern char  Real_Ctl1_Value[3];
//extern char  Real_Ctl2_Value[3];

extern int16_t v485_1,v485_2,v485_3,v485_4,v485_5,v485_6,v485_7,v485_8;
extern int32_t temp_push1,temp_push2,temp_rotate1,temp_rotate2;
//#define RED_SIDE

extern int32_t Time_ms; 
extern int32_t Time_pump;
extern int32_t Time_s;
extern int32_t Time_min;

extern int32_t temp_int32;
extern int16_t temp_int16;
extern uint16_t temp_time;

extern float Theta_0;

extern int judge;



#define abs(x) ((x)>0? (x):(-(x)))
#define PI 3.1415926
#define T 0.005 	

#endif
