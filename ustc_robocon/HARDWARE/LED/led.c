#include "led.h" 
#include "include.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"


//��ʼ��PF9��PF10Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_9|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
	
	GPIO_SetBits(GPIOF,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10���øߣ�����

}   

 long offset=0;

 u8 flagna=0;
int reset()                                                    //��ʼǰ��λ
{  

	if(Real_Ctl1_Value[0]==1){
		flagna=1;
		offset=Real_Position_Value[0];
	  }

	if(!flagna)CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,1000,1000,-200000);//��ת 

	if(flagna){
		//CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,1000,0,0);
		
		CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,25000+offset);
		servo1(68);	
    servo2(70);
    if(Real_Position_Value[0]>(25000+offset))return 1;//������λ����
		else return 0;		
	}
	return 0;  
	
}
int Catch_Ball()                                               //ȡ��
{
	int end;
	static float t=0;
	static int state=2;
	t+=0.005;
	switch(state)
	{
		case 0:
			if(reset()){
				state=1;
				return 0;
			}
		  break;
		case 1:
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+27000);
		  servo1(75);	
      servo2(75);		
	    if(Real_Position_Value[0]>(27000+offset)){
				state=2;
				return 0;
			}
		  break;
		case 2:
			
				GPIO_ResetBits(GPIOF,GPIO_Pin_1);//ץ�ֹر�
				GPIO_ResetBits(GPIOF,GPIO_Pin_2);//ץ�ֹر�
				state=3;
				return 0;
			break;
		case 3:
			return 1;
		  break;
		default:
			break;
	}
}
int Catch_Ball2()                                               //ȡ��
{
	int end;
	static float t=0;
	static int state=2;
	t+=0.005;
	switch(state)
	{
		case 0:
			if(reset()){
				state=1;
				return 0;
			}
		  break;
		case 1:
				CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+27000);
		  servo1(63);	
      servo2(65);	
	    if(Real_Position_Value[0]>=(27000+offset)){
				state=2;
				return 0;
			}
		  break;
		case 2:
			
				GPIO_ResetBits(GPIOF,GPIO_Pin_1);//ץ�ֹر�
				GPIO_ResetBits(GPIOF,GPIO_Pin_2);//ץ�ֹر�
				state=3;
				return 0;
			break;
		case 3:
			return 1;
		  break;
		default:
			break;
	}
}



u8 Transfer_Color1()                                         //���Ӳ���1
{
  static float t=0;
	static int state=1;
	t+=0.005;
  switch (state)
  {
		case 1:
			GPIO_ResetBits(GPIOF,GPIO_Pin_1);//ץ�ֹر�
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+90000);
      servo1(45);		
	    if(Real_Position_Value[0]>(90000+offset))state=2;
		  break;
		case 2:
			GPIO_ResetBits(GPIOF,GPIO_Pin_0);//�������  
      state=3;
		  break;
		case 3:
		  if(t>=2.8){
		  GPIO_SetBits(GPIOF,GPIO_Pin_1); 		//ץ�ִ�
			state=4;}
			break;
		case 4:
			if(t>2.9){GPIO_SetBits(GPIOF,GPIO_Pin_0);	 //�����ջ�
		  
			}
		  break;
		default:
			break;
	}
	if(t>3.4)return 1;
	else return 0;
}
//u8 Transfer_Color1()                                         //���Ӳ���2
//{
//  static float t=0;
//	static int state=1;
//	
//  switch (state)
//  {
//		case 1:
//			GPIO_ResetBits(GPIOF,GPIO_Pin_1);//ץ�ֹر�
//			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+90000);
//      servo1(45);		
//	    if(Real_Position_Value[0]>(90000+offset))state=2;
//		  break;
//		case 2:
//			t+=0.005;if(t>5){
//			GPIO_ResetBits(GPIOF,GPIO_Pin_0);//�������  
//      state=3;
//			}
//		  break;
//		case 3:
//		  t+=0.005;
//		if(t>6){
//		  GPIO_SetBits(GPIOF,GPIO_Pin_1); 		//ץ�ִ�
//			state=4;}
//			break;
//		case 4:
//			t+=0.005;
//		GPIO_SetBits(GPIOF,GPIO_Pin_0);
//		if(t>9){
//				return 1; //�����ջ�
//			}
//		  break;
//		default:
//			break;
//	}	
//}
u8 Transfer_Color2()                                         //���Ӳ���2
{
  static float t=0;
	static int state=1;
	
  switch (state)
  {
		case 1:
			GPIO_ResetBits(GPIOF,GPIO_Pin_2);//ץ�ֹر�
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+48000);
      servo2(35);		
	    if(Real_Position_Value[0]>=(48000+offset))state=2;
		  break;
		case 2:
			t+=0.005;if(t>0.8){
			GPIO_ResetBits(GPIOF,GPIO_Pin_0);//�������  
      state=3;
			}
		  break;
		case 3:
		  t+=0.005;
		if(t>2.8){
		  GPIO_SetBits(GPIOF,GPIO_Pin_2); 		//ץ�ִ�
			state=4;}
			break;
		case 4:
			t+=0.005;
		
		if(t>3.6){
			GPIO_SetBits(GPIOF,GPIO_Pin_0);
				
			}
		if(t>4.2){
			return 1;
		}
		  break;
		default:
			break;
	}
		
	 return 0;
}

u8 Transfer_Gold()                                           //���ӽ���
{
  static float t=0;
	static int state=1;
	t+=0.005;
  switch (state)
  {
		case 1:
			GPIO_ResetBits(GPIOF,GPIO_Pin_1);//ץ�ֹر�
		  GPIO_ResetBits(GPIOF,GPIO_Pin_2);//ץ�ֹر�
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+87000);
      servo1(45);	
      servo2(60);		
	    if(Real_Position_Value[0]>=(86900+offset))state=2;
		  break;
		case 2:
			GPIO_ResetBits(GPIOF,GPIO_Pin_0);//�������  
      state=3;
		  break;
		case 3:
		  if(t>=1.9){
		  GPIO_SetBits(GPIOF,GPIO_Pin_1); 		//ץ�ִ�
			GPIO_SetBits(GPIOF,GPIO_Pin_2); 		//ץ�ִ�
			state=4;}
			break;
		case 4:
			if(t>2.4)GPIO_SetBits(GPIOF,GPIO_Pin_0);	 //�����ջ�
		  break;
		default:
			break;
	}
		if(t>3.8)return 1;
	else return 0;
}







