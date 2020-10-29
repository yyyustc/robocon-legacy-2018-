#include "delay.h"
#include "usart3.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	 
#include "include.h"
//#include "usart4.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4������
//����3��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/8/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//���ڷ��ͻ����� 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
#ifdef USART3_RX_EN   								//���ʹ���˽���   	  
//���ڽ��ջ����� 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.


//ͨ���жϽ�������2���ַ�֮���ʱ������100ms�������ǲ���һ������������.
//���2���ַ����ռ������100ms,����Ϊ����1����������.Ҳ���ǳ���100msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 USART3_RX_STA=0;   	

float Theta_0 = 0;
float X_0 = 0;
float Y_0 = -400;
float Theta_q = 0 ;
//float Theta_Init = 0;
float Diff_Pos_X = 0;
//ȫ����λϵͳ����
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
					//�õ�����һ֡�����Դ���õ���������
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
//��ʼ��IO ����3
//bound:������	  
void usart3_init(u32 bound)
{  

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��

 	USART_DeInit(USART3);  //��λ����3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��GPIOB11����GPIOB10
	
	USART_InitStructure.USART_BaudRate = bound;//������ 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
 

	USART_Cmd(USART3, ENABLE);               //ʹ�ܴ��� 
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�   
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//	TIM7_Int_Init(1000-1,8400-1);		//100ms�ж�
	USART3_RX_STA=0;		//����
//	TIM_Cmd(TIM7, DISABLE); //�رն�ʱ��7

}

//����3,printf ����
//ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
void u3_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);//�˴η������ݵĳ���
	for(j=0;j<i;j++)//ѭ����������
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
		USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); 	 //�������ݵ�����3 
	}
	
}
 
 
 











