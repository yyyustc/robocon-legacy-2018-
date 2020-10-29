#include "delay.h"
#include "usart2.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	 
#include "include.h"
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
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
#ifdef USART2_RX_EN   								//���ʹ���˽���   	  
//���ڽ��ջ����� 	
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.


//ͨ���жϽ�������2���ַ�֮���ʱ������100ms�������ǲ���һ������������.
//���2���ַ����ռ������100ms,����Ϊ����1����������.Ҳ���ǳ���100msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 USART2_RX_STA=0;   	

int32_t Callib_Angle;
int32_t Callib_X = -1;
int32_t Cross_Pos_X;
int32_t Cross_Pos_Y;
int32_t Resolution_X;
int32_t Resolution_Y;

uint8_t Received_OK = 0;
uint8_t Received_ERROR = 0;

//uint8_t Wifi_State = WIFI_INIT;
uint8_t Show_Flag = 0;


void USART2_IRQHandler(void)
{
	static uint8_t ch2;
	static union
	{
		int8_t data[24];
		int32_t Data[6];
	}Receive_Int;

	static uint8_t count=0;
	static uint8_t i=0;
	static uint8_t Init_Flag=0;


//		LED0=!LED0;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{
   	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		ch2=USART_ReceiveData(USART2);
		    switch(count){
			case 0:
				if(ch2==0x0d)
					count++;
				else count=0;
				break;
			case 1:
				if(ch2==0x0a){
					count++;
					i=0;
				}
				else if(ch2==0x0d);
				else count=0;
				break;
			case 2:
				Receive_Int.data[i]=ch2;
				i++;
				if(i>=24){
					i=0;count++;
				}
				break;
			case 3:
				if(ch2==0x0a)
					count++;
				else
					count=0;
				break;
			case 4:
				if(ch2==0x0d){
//					LED0=!LED0;
//					LED0=!LED0;
					//�õ�����һ֡�����Դ���õ���������
					LED0=!LED0;  
					Callib_X = Receive_Int.Data[0];          //���м���ߵ�ƫ����0-640
					Callib_Angle = Receive_Int.Data[1];      //-1700~1700�������ߵ�ƫ�ƽǣ���λ��0.01��
					Cross_Pos_X = Receive_Int.Data[2];       //��ʮ�ֵ�ˮƽλ��
					Cross_Pos_Y = Receive_Int.Data[3];       //��ʮ�ֵ���ֱλ��
					Resolution_X = Receive_Int.Data[4];      //ͼ���С��������
					Resolution_Y = Receive_Int.Data[5];					//ͼ���С��������
//					itoa(Callib_X,Show_Content[0],10);
//					strncpy(NVI_Content,"recieved",8);
//					NVI_Str_Num = MAX_NVI_BUFF;
					
				}
				
				count=0;
				break;
			default:
				count=0;
				break;
		}		
	}
	else{
		USART_ClearITPendingBit(USART2,USART_IT_PE);
		USART_ClearITPendingBit(USART2,USART_IT_TXE);
		USART_ClearITPendingBit(USART2,USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART2,USART_IT_TC);
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);
		USART_ClearITPendingBit(USART2,USART_IT_LBD);
		USART_ClearITPendingBit(USART2,USART_IT_CTS);
		USART_ClearITPendingBit(USART2,USART_IT_ERR);
		USART_ClearITPendingBit(USART2,USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART2,USART_IT_NE);
		USART_ClearITPendingBit(USART2,USART_IT_FE);

	}
	//USART_SendData(USART1,ch2);
	
} 

	
#endif	
//��ʼ��IO ����3
//bound:������	  
void usart2_init(u32 bound)
{  

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART3ʱ��

 	USART_DeInit(USART2);  //��λ����3
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOB10����ΪUSART3	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��GPIOA3����GPIOA2
	
	USART_InitStructure.USART_BaudRate = bound;//������ 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  
	USART_Init(USART2, &USART_InitStructure); //��ʼ������3
 

	USART_Cmd(USART2, ENABLE);               //ʹ�ܴ��� 
	
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�   
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//	TIM7_Int_Init(1000-1,8400-1);		//100ms�ж�
	USART2_RX_STA=0;		//����
//	TIM_Cmd(TIM7, DISABLE); //�رն�ʱ��7
  	

}

//����3,printf ����
//ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
void u2_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART2_TX_BUF);//�˴η������ݵĳ���
	for(j=0;j<i;j++)//ѭ����������
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
		USART_SendData(USART2,(uint8_t)USART2_TX_BUF[j]); 	 //�������ݵ�����3 
	}
	
}
 
 
 











