#include "pstwo.h"
//#include "delay.c"
#include "include.h" 
/*********************************************************
opyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File��PS2��������
Author��pinggai    Version:1.1     Data:2015/10/20
Description: PS2��������
             ���ӹ��ܣ�
			 1��������á����ģʽ�������̵�ģʽ�������������á����桱��ͨ���ֱ���ģʽ�������޷��ı�
			 2�������ֱ��𶯣�ͨ����ֵ�����ã��ı������𶯵����Ƶ�ʡ�
			                  ͨ����ֵ�����ã������Ҳ�С�𶯵����
History:  
V1.0: 	2015/05/16
1���ֱ����룬ʶ�𰴼�ֵ����ȡģ��ֵ��       
*********************************************
3,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,*************/	 
#define DELAY_TIME  delay_us(5 ); 
//#define DELAY_TIME  ; 
u16 Handkey;	// ����ֵ��ȡ����ʱ�洢��
u8 Comd[2]={0x01,0x42};	//��ʼ�����������
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����

double Handle_Addup = 0;
//����Ϊ1-16
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
	};	//����ֵ�밴����

//�ֱ��ӿڳ�ʼ��    ����  DAT(DI)->PD11 
//                  ���  DO(CMD)->PB12    CS->PB13  CLK->PB14
void PS2_Init(void)
{
	    
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    //ʹ��PORTBʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);    //ʹ��PORTBʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;//����  DI->PD11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO

//	GPIOB->CRH&=0XFFF0FFFF;//PB12���ó�����	Ĭ������  
//	GPIOB->CRH|=0X00080000;   

    //  DO->PB13    CS->PB14  CLK->PB15
//	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��  	   	  	 
//	GPIOB->CRH&=0X000FFFFF; 
//	GPIOB->CRH|=0X33300000;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;//PB12��PB13��PB14 �������   	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
  CLK_H;
  DO_H;
   CS_H;
}

//���ֱ���������
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //���һλ����λ
		}
		else DO_L;

		CLK_L;                        //ʱ������
		DELAY_TIME;
		CLK_H;
		DELAY_TIME;
                
		//CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}//DO_H;
	//delay_us(16);
}
//�ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
//����ֵ��0�����ģʽ
//		  ����������ģʽ
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{      
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	delay_us(50);
	CS_L;
        delay_us(30);
	PS2_Cmd(Comd[0]);  //��ʼ����
        delay_us(20);
	PS2_Cmd(Comd[1]);  //��������
        delay_us(20);
	for(byte=2;byte<9;byte++)          //��ʼ��������
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

//�Զ�������PS2�����ݽ��д���,ֻ����������  
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1

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

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //��ʼ����
	PS2_Cmd(0x42);  //��������
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
//��������
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
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ
	PS2_Cmd(0xEE); //Ox03�������ã�������ͨ��������MODE������ģʽ��
				   //0xEE������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//������
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
//��ɲ���������
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
//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
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













