//#include "delay.h"
//#include "usart4.h"
//#include "stdarg.h"	 	 
//#include "stdio.h"	 	 
//#include "string.h"	  
//#include "timer.h"
////////////////////////////////////////////////////////////////////////////////////	 
////本程序只供学习使用，未经作者许可，不得用于其它任何用途
////ALIENTEK STM32F4开发板
////串口3驱动代码	   
////正点原子@ALIENTEK
////技术论坛:www.openedv.com
////修改日期:2014/8/9
////版本：V1.0
////版权所有，盗版必究。
////Copyright(C) 广州市星翼电子科技有限公司 2009-2019
////All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////// 	   
//#include "include.h"

////串口发送缓存区 	
//__align(8) u8 UART4_TX_BUF[UART4_MAX_SEND_LEN]; 	//发送缓冲,最大UART4_MAX_SEND_LEN字节
//#ifdef UART4_RX_EN   								//如果使能了接收   	  
////串口接收缓存区 	
//u8 UART4_RX_BUF[UART4_MAX_RECV_LEN]; 				//接收缓冲,最大UART4_MAX_RECV_LEN个字节.


////通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
////如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
////任何数据,则表示此次接收完毕.
////接收到的数据状态
////[15]:0,没有接收到数据;1,接收到了一批数据.
////[14:0]:接收到的数据长度

//u16 UART4_RX_STA=0;   	   	

//int32_t Callib_Angle;
//int32_t Callib_X = -1;
//int32_t Cross_Pos_X;
//int32_t Cross_Pos_Y;
//int32_t Resolution_X;
//int32_t Resolution_Y;

//uint8_t Received_OK = 0;
//uint8_t Received_ERROR = 0;

////uint8_t Wifi_State = WIFI_INIT;
//uint8_t Show_Flag = 0;


//void UART4_IRQHandler(void)
//{
//	static uint8_t ch2;
//	static union
//	{
//		int8_t data[24];
//		int32_t Data[6];
//	}Receive_Int;

//	static uint8_t count=0;
//	static uint8_t i=0;
//	static uint8_t Init_Flag=0;


////		LED0=!LED0;
//	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
//	{
//   	USART_ClearITPendingBit(UART4,USART_IT_RXNE);
//		ch2=USART_ReceiveData(UART4);
//		    switch(count){
//			case 0:
//				if(ch2==0x0d)
//					count++;
//				else count=0;
//				break;
//			case 1:
//				if(ch2==0x0a){
//					count++;
//					i=0;
//				}
//				else if(ch2==0x0d);
//				else count=0;
//				break;
//			case 2:
//				Receive_Int.data[i]=ch2;
//				i++;
//				if(i>=24){
//					i=0;count++;
//				}
//				break;
//			case 3:
//				if(ch2==0x0a)
//					count++;
//				else
//					count=0;
//				break;
//			case 4:
//				if(ch2==0x0d){
////					LED0=!LED0;
////					LED0=!LED0;
//					//得到完整一帧，可以处理得到的数据了
//					LED0=!LED0;  
//					Callib_X = Receive_Int.Data[0];          //和中间白线的偏移量0-640
//					Callib_Angle = Receive_Int.Data[1];      //-1700~1700，与竖线的偏移角，单位：0.01度
//					Cross_Pos_X = Receive_Int.Data[2];       //和十字的水平位移
//					Cross_Pos_Y = Receive_Int.Data[3];       //和十字的竖直位移
//					Resolution_X = Receive_Int.Data[4];      //图像大小横像素数
//					Resolution_Y = Receive_Int.Data[5];					//图像大小纵像素数
////					itoa(Callib_X,Show_Content[0],10);
////					strncpy(NVI_Content,"recieved",8);
////					NVI_Str_Num = MAX_NVI_BUFF;
//					
//				}
//				
//				count=0;
//				break;
//			default:
//				count=0;
//				break;
//		}		
//	}
//	else{
//		USART_ClearITPendingBit(UART4,USART_IT_PE);
//		USART_ClearITPendingBit(UART4,USART_IT_TXE);
//		USART_ClearITPendingBit(UART4,USART_IT_ORE_RX);
//		USART_ClearITPendingBit(UART4,USART_IT_TC);
//		USART_ClearITPendingBit(UART4,USART_IT_IDLE);
//		USART_ClearITPendingBit(UART4,USART_IT_LBD);
//		USART_ClearITPendingBit(UART4,USART_IT_CTS);
//		USART_ClearITPendingBit(UART4,USART_IT_ERR);
//		USART_ClearITPendingBit(UART4,USART_IT_ORE_ER);
//		USART_ClearITPendingBit(UART4,USART_IT_NE);
//		USART_ClearITPendingBit(UART4,USART_IT_FE);

//	}
//	//USART_SendData(USART1,ch2);
//	
//} 

//	
//#endif	
//  
//void uart4_init(u32 bound)
//{  

//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOB时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART4时钟

// 	USART_DeInit(UART4);  //复位串口4
//	
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOB11复用为USART4
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOB10复用为USART4	
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOC11和GPIOC10初始化
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10
//	
//	USART_InitStructure.USART_BaudRate = bound;//波特率 
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//  
//	USART_Init(UART4, &USART_InitStructure); //初始化串口4
// 

//	USART_Cmd(UART4, ENABLE);               //使能串口 
//	
//  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断   
//	
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
////	TIM7_Int_Init(1000-1,8400-1);		//100ms中断
//	UART4_RX_STA=0;		//清零
////	TIM_Cmd(TIM7, DISABLE); //关闭定时器7
//  	

//}

////串口3,printf 函数
////确保一次发送数据不超过USART4_MAX_SEND_LEN字节
//void u4_printf(char* des)  
//{  
//	int i,j;
//	j=strlen(des);
//	for(i=0;i<j;i++){
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);  //等待上次传输完成 
//		USART_SendData(UART4,(uint8_t)des[i]); 	 //发送数据到串口 		
//	}
//}
// 











