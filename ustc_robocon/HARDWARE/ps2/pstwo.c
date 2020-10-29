#include "pstwo.h"
//#include "delay.c"
#include "include.h" 
/*********************************************************
opyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File：PS2驱动程序
Author：pinggai    Version:1.1     Data:2015/10/20
Description: PS2驱动程序
             增加功能：
			 1、软件设置“红灯模式”、“绿灯模式”，并可以设置“保存”，通过手柄“模式按键”无法改变
			 2、设置手柄震动：通过数值的设置，改变左侧大震动电机震动频率。
			                  通过数值的设置，开关右侧小震动电机。
History:  
V1.0: 	2015/05/16
1、手柄解码，识别按键值，读取模拟值。       
*********************************************
3,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,*************/	 
#define DELAY_TIME  delay_us(5 ); 
//#define DELAY_TIME  ; 
u16 Handkey;	// 按键值读取，零时存储。
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组

double Handle_Addup = 0;
//依次为1-16
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明

//手柄接口初始化    输入  DAT(DI)->PD11 
//                  输出  DO(CMD)->PB12    CS->PB13  CLK->PB14
void PS2_Init(void)
{
	    
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    //使能PORTB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);    //使能PORTB时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;//输入  DI->PD11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//下拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO

//	GPIOB->CRH&=0XFFF0FFFF;//PB12设置成输入	默认下拉  
//	GPIOB->CRH|=0X00080000;   

    //  DO->PB13    CS->PB14  CLK->PB15
//	RCC->APB2ENR|=1<<3;    //使能PORTB时钟  	   	  	 
//	GPIOB->CRH&=0X000FFFFF; 
//	GPIOB->CRH|=0X33300000;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;//PB12、PB13、PB14 推挽输出   	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
  CLK_H;
  DO_H;
   CS_H;
}

//向手柄发送命令
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出一位控制位
		}
		else DO_L;

		CLK_L;                        //时钟拉高
		DELAY_TIME;
		CLK_H;
		DELAY_TIME;
                
		//CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}//DO_H;
	//delay_us(16);
}
//判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
//返回值；0，红灯模式
//		  其他，其他模式
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{      
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	delay_us(50);
	CS_L;
        delay_us(30);
	PS2_Cmd(Comd[0]);  //开始命令
        delay_us(20);
	PS2_Cmd(Comd[1]);  //请求数据
        delay_us(20);
	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
                      else Data[byte]&=~(ref|0x00);
		}
        delay_us(16);
	}
	CS_H;
}

//对读出来的PS2的数据进行处理,只处理按键部分  
//只有一个按键按下时按下为0， 未按下为1

u8 PS2_DataKey_Once(){
  
  
	static u8 lastkey = 0;
	u8 key = 0;
	u8 result = 0;
	key = PS2_DataKey();
	if(key!=0 && lastkey!=key){
		result = key;
	}
	lastkey = key;
	return result;
}

u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
//进入配置
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0xEE); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//振动设置
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
//完成并保存配置
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
//手柄配置初始化
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
}

/*
void Key_Read()
{
	static int cnt = 0;
	int key_ps2=0;
		key_ps2=PS2_DataKey_Once();
		Pump(0);
		switch(key_ps2){
			case PSB_L2:{
				Pump(2);
				break;
			}
			case(PSB_R2):{
				Pump(1);
				break;
			}
			case(PSB_L1):{
				left_count++;
				if(left_count>4)left_count=1;
				break;
			}
			case(PSB_R1):{
				right_count++;
				if(right_count>3)right_count=1;
				break;
			}
			case(PSB_GREEN):{
				Cmd_Flag = 2;
				break;
			}
			case(PSB_PINK):{
				Handle_Addup = 200;
				cnt = 9;
				break;
			}
			case(PSB_BLUE):{
				Handle_Addup = -200;
				cnt = 9;
				break;
			}
			default:{
				cnt--;
				if(cnt<=0)
					Handle_Addup *= 0;
//				else
//					Handle_Addup *= 9/10;					
				break;
			}
		}

		if(PS2_RedLight())
		{
			if(key_ps2 == PSB_PAD_UP){
				if(cmd_left[left_count][0]>abs(cmd_left[left_count][0]))
				{
				cmd_left[left_count][0]+=100;
				cmd_left[left_count][2]+=100;		

				}
				else
				{
				cmd_left[left_count][1]-=100;
				cmd_left[left_count][3]-=100;		
				}

//				cmd_left[left_count][0]+=50;
//				cmd_left[left_count][1]-=50;
//				cmd_left[left_count][2]+=50;
//				cmd_left[left_count][3]-=50;		
			}
			else if(key_ps2 == PSB_PAD_DOWN){
				if(cmd_left[left_count][0]>abs(cmd_left[left_count][0]))
				{
				cmd_left[left_count][0]-=100;
				cmd_left[left_count][2]-=100;		

				}
				else
				{
				cmd_left[left_count][1]+=100;
				cmd_left[left_count][3]+=100;		
				}
			}
			else if(key_ps2 == PSB_PAD_LEFT){
				cmd_left[left_count][5]+=2000;
			}
			else if(key_ps2 == PSB_PAD_RIGHT){
				cmd_left[left_count][5]-=2000;
			}			
		}
		else
		{
//			LED0=!LED0;
			if(key_ps2 == PSB_PAD_UP){
				if(cmd_right[right_count][0]>abs(cmd_right[right_count][0]))
				{
				cmd_right[right_count][0]+=100;
				cmd_right[right_count][2]+=100;		

				}
				else
				{
				cmd_right[right_count][1]-=100;
				cmd_right[right_count][3]-=100;		
				}	
			}
			else if(key_ps2 == PSB_PAD_DOWN){
				if(cmd_right[right_count][0]>abs(cmd_right[right_count][0]))
				{
				cmd_right[right_count][0]-=100;
				cmd_right[right_count][2]-=100;		

				}
				else
				{
				cmd_right[right_count][1]+=100;
				cmd_right[right_count][3]+=100;		
				}		
			}
			else if(key_ps2 == PSB_PAD_LEFT){
				cmd_right[right_count][5]+=1500;
			}
			else if(key_ps2 == PSB_PAD_RIGHT){
				cmd_right[right_count][5]-=1500;
			}			
		}
}
*/













