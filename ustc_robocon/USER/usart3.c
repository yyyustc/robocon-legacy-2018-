#include "delay.h"
#include "usart3.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	 
#include "include.h"
//#include "usart4.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4开发板
//串口3驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/8/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//串口发送缓存区 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节
#ifdef USART3_RX_EN   								//如果使能了接收   	  
//串口接收缓存区 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.


//通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
u16 USART3_RX_STA=0;   	

float Theta_0 = 0;
float X_0 = 0;
float Y_0 = -400;
float Theta_q = 0 ;
//float Theta_Init = 0;
float Diff_Pos_X = 0;
//全场定位系统参数
double aim_speed;
float pos_x,pos_y,velo_x;
float zangle,xangle,yangle;
float w_z;
//int motor1,motor2,motor3;
//double Target_Pos_Y = 0;
//double Target_Pos_X = 0;
//double Init_Pos_X = 0;
//double Init_Pos_Y = 0;
double Init_Angle = 0;
extern float pos_x_offset;
extern float pos_y_offset;
extern float zangle_offset;
double Get_Angle_Diff(double a1, double a2){
	
	if(a1>a2){
		if(a1 - a2 > PI) return a2 + 2*PI - a1;
		else return a2 - a1;
	}
	else{
		if(a2 - a1 > PI) return a2 - a1 - 2*PI;
		else return a2 - a1;
	}
}



void USART3_IRQHandler(void)
{
	static int cnt;
	static uint8_t ch1;
	static union
	{
		uint8_t data[24];
		float Data[6];
	}Receive_Float;

	static float last_pos_x = 0;
	
	
	static uint8_t count=0;
	static uint8_t i=0;


  LED0=!LED0;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{
		//LED0=!LED0;
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		ch1=USART_ReceiveData(USART3);
    switch(count){
			case 0:
				if(ch1==0x0d)
					count++;
				else count=0;
				break;
			case 1:
				if(ch1==0x0a){
					count++;
					i=0;
				}
				else if(ch1==0x0d);
				else count=0;
				break;
			case 2:
				Receive_Float.data[i]=ch1;
				i++;
				if(i>=24){
					i=0;count++;
				}
				break;
			case 3:
				if(ch1==0x0a)
					count++;
				else
					count=0;
				break;
			case 4:
				if(ch1==0x0d){
//					LED0=!LED0;
//					LED0=!LED0;
					//得到完整一帧，可以处理得到的数据了
					zangle = Receive_Float.Data[0];
					xangle = Receive_Float.Data[1];
					yangle = Receive_Float.Data[2];
					pos_x = Receive_Float.Data[3];
					velo_x = (last_pos_x - pos_x)/0.005;
					last_pos_x = pos_x;
					pos_y = Receive_Float.Data[4];
					w_z = Receive_Float.Data[5];
					Theta_q = Get_Angle_Diff(Init_Angle, zangle);
					
				}
				
				count=0;
				break;
			default:
				count=0;
				break;
		}			
	}
	else{
		USART_ClearITPendingBit(UART4,USART_IT_PE);
		USART_ClearITPendingBit(UART4,USART_IT_TXE);
		USART_ClearITPendingBit(UART4,USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART4,USART_IT_TC);
		USART_ClearITPendingBit(UART4,USART_IT_IDLE);
		USART_ClearITPendingBit(UART4,USART_IT_LBD);
		USART_ClearITPendingBit(UART4,USART_IT_CTS);
		USART_ClearITPendingBit(UART4,USART_IT_ERR);
		USART_ClearITPendingBit(UART4,USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART4,USART_IT_NE);
		USART_ClearITPendingBit(UART4,USART_IT_FE);

	}		
	
	//USART_SendData(USART1,ch1);
} 

#endif	
//初始化IO 串口3
//bound:波特率	  
void usart3_init(u32 bound)
{  

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟

 	USART_DeInit(USART3);  //复位串口3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;//波特率 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
 

	USART_Cmd(USART3, ENABLE);               //使能串口 
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断   
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//	TIM7_Int_Init(1000-1,8400-1);		//100ms中断
	USART3_RX_STA=0;		//清零
//	TIM_Cmd(TIM7, DISABLE); //关闭定时器7

}

//串口3,printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void u3_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);//此次发送数据的长度
	for(j=0;j<i;j++)//循环发送数据
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  //等待上次传输完成 
		USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); 	 //发送数据到串口3 
	}
	
}
 
 
 











