#define StandBy 0x00 //�������� JS
#define Trace1 0x14//
#define Pos1 0x01  //�ò���
#define GetBall1 0x02 //����
#define Trace2 0x03   //ȥ�ݲ���
#define Pos2 0x04 //��λ����һ��λ
#define Handle1 0x05 //�ݲ���
//#define Trace3 0x06 //ȥ���ݵڶ���
#define Wait1 0x07 //���ŵݵڶ��� JS
//#define Trace3_5 0x15
#define Pos3 0x08 //��λ���ڶ���λ
#define Handle2 0x09 //�ݲ���
#define Trace4 0x0A //�ܻ�ȥ�ý���
#define Pos4 0x0B //��λ�ý���λ
#define Wait2 0x0C //�ȴ�Ͷ�� JS
#define GetBall2 0x13//�ڶ�������
#define Trace5 0x0D //ȥ�ݽ���
#define Pos5 0x0E //��λ�ݽ���
#define Wait3 0x17
#define Handle3 0x0F //�ݽ���
#define Trace6 0x10 //��ȡ����
#define Pos6 0x11 //�� ȡ����λ
//#define GetBall 0x01
#define Finished 0x12 //��ɱ���
#include "include.h"
#include "led.h"
float t=0;
short temp_pwm = 5000;
#define Analog (Data[1]==0x73)
#define BACKMOVE if(GlobalStatus==Wait2)GlobalStatus=Pos4;if(GlobalStatus==Wait1)GlobalStatus=Pos3;if(GlobalStatus==Wait3)GlobalStatus=Pos5;
extern u8 flagna;
extern long offset;
////*ȫ�ֱ���*//////////////////////////////////////////////////////////////////////////////////////

//u8 GlobalStatus=0x00;
u8 GlobalStatus=0x00;
float pos_x_offset=0;
float pos_y_offset=0;
float zangle_offset=0;
u8 DATAWBefore=0xFF;
u8 DATAYBefore=0xFF;
//u8 PosEnabled=0;//���ȫ�ֱ�־λ�������ض�ʱ���е�POSCTRL(timer.c
//float DesX=0,DesY=0,DesW=0;//ȫ�ֱ�����������POSCTRL��Ŀ��λ��
//u8 PosArrived=0;//�����ж���û�е���Ŀ��λ��
//u8 Catched=0;

///////////////////////////////////////////////////////////////////////////////////////

//uint8_t Reach_position = 0;
//uint8_t Limit_switch1 = 0;
//uint8_t Limit_switch2 = 0;
//uint8_t Limit_switch3 = 0;
//uint8_t Light_LED = 0;
int status = 0;
extern float Y_control;
int tst1=0;
//u8 SendCnt=0;
u8 sendbuf=0;
CanRxMsg CMFB1;//ChassisMotorFeedBackBuf
CanRxMsg CMFB2;
CanRxMsg CMFB3;

int main(void)
{    //float JS_x=0;
	    delay_ms(10000);
	    
	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	    
    //  CAN1_Configuration();                               //CAN1��ʼ��
	   // CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN��ʼ��Ϊ����ģʽ��������1000Kbps    
	    CAN1_Configuration();
		  PS2_Init();
	    LED_Init();
	    Adc_Init();         //��ʼ��ADC
      delay_init(168);    //��ʼ����ʱ����
  	  uart_init(115200);
      
      delay_ms(500);                                      //�տ�ʼҪ���㹻����ʱ��ȷ���������Ѿ���ʼ����� 
      CAN_RoboModule_DRV_Reset(0,0);                      //��0���������������и�λ 
      delay_ms(500); 
	    CAN_RoboModule_DRV_Config(0,1,10,10);               //1������������Ϊ1ms����һ������          
      delay_us(200);                                      //�˴���ʱΪ�˲��ô�������ʱ��4����һ��
		  CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Position_Mode);//0��������λ���ٶ�ģʽ
		  
		  usart3_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	    	                     //��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms
									//printf("%f\n",pos_x);					 
		  //delay_ms(10500);
		//	while(1){
   //   				if(Real_Ctl1_Value[0]==1||Real_Ctl2_Value[0]==1){
	//	flagna=1;							
	//  }
							
//	}
//while(1){}
	  //  ahha:
		//	tst1=reset();
		//		if(Real_Ctl1_Value[0]==1){
		//flagna=1;
	 // }
			//if(!tst1){goto ahha;}
				
				TIM3_Int_Init(50-1,8400-1);
//	    while(1){
//		  PS2_ReadData();
//				if(Data[1]==0x73)GetMotorSpeed(-(Data[5]-132)*0.5/132.0,(Data[6]-132)*0.5/132.0,0) ;
//	     }
//	    while(!reset()){}
//			Transfer_Ball();

      while(1){
				delay_ms(100);
				printf("%f %f %f\r\n",pos_x,pos_y,zangle);
				//		  PS2_ReadData();
			 //	if(Data[1]==0x73&&(Data[5]>133||Data[5]<131||Data[6]>133||Data[6]<131))GetMotorSpeed(-(Data[5]-132)*0.5/132.0,(Data[6]-132)*0.5/132.0,0) ;
				//��Ƭ
				PS2_ReadData();
				if(Analog){
				if((~Data[3]&0x10)&&(!(~DATAWBefore&0x10))){pos_y_offset+=20.0;BACKMOVE}//������������⵽UP�������أ��������ƫ�ƣ��������ӽǣ� 
				if((~Data[3]&0x20)&&(!(~DATAWBefore&0x20))){pos_x_offset+=20.0;BACKMOVE}//������������⵽RIGHT�������أ������ƫ
				if((~Data[3]&0x40)&&(!(~DATAWBefore&0x40))){pos_y_offset-=20.0;BACKMOVE}//�����ø������⵽DOWN�������أ������
				if((~Data[3]&0x80)&&(!(~DATAWBefore&0x80))){pos_x_offset-=20.0;BACKMOVE}//�����ø�����⵽LEFT�������أ������
					
				if((~Data[4]&0x04)&&(!(~DATAYBefore&0x04))){zangle_offset-=2.0;BACKMOVE}//�����ø���L1
				if((~Data[4]&0x08)&&(!(~DATAYBefore&0x08))){zangle_offset+=2.0;BACKMOVE}//�����ø�  R1		

				
				DATAWBefore=Data[3];
				DATAYBefore=Data[4];
					}
			//	sendbuf=(int)pos_x;
			//USART_SendData  (USART1,sendbuf);
			}



}
