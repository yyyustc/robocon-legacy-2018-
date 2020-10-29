#include "timer.h"
#include "led.h"
#include "include.h" 
#include "can.h" 
#define T 0.005 	 
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
#define Repos 0x18


//ʹ�õ�0x18
//////////////////////////////////////////////////////////////////////////
extern u8 GlobalStatus;
extern long offset;

extern float pos_x_offset;
extern float pos_y_offset;
extern float zangle_offset;
//extern u8 PosEnabled;
//extern u8 PosArrived;
//extern float DesX,DesY,DesW;//ȫ�ֱ���ȫ��main��
//extern u8 Catched;
//////////////////////////////////////////////////////////////////////////

void Schedule(u8 temp){
static int Pos4OverTime=0;
	switch(temp){
		case StandBy: 
			reset();
			PS2_ReadData();
		  if(((~Data[3]&0x08))&&Data[1]==0x73)GlobalStatus=Trace1;
		break;
		
				case Trace1: 
					
			if(Trajectory_Planning1_())GlobalStatus=Pos1;
				PS2_ReadData();
				if(((~Data[4]&0x40))&&Data[1]==0x73)GlobalStatus=GetBall1;
		break;
				
		case Pos1:
			PS2_ReadData();
     if(Position_Control(2800,-1,0)||(((~Data[4]&0x40))&&Data[1]==0x73))GlobalStatus=GetBall1;
		break;
		
		case GetBall1:
			if (Catch_Ball())GlobalStatus=Trace2;
		break;	
		
		case Trace2: 
			if(Trajectory_Planning2())GlobalStatus=Pos2;
		break;
		
		case Pos2 :
			if(Position_Control(-6168,-1171,0)){
				GlobalStatus=Repos;//��������
				SendSpeed(0,0,0);
			}
		break;
			
		case Repos :
			PS2_ReadData();
			if(Data[1]==0x73&&(Data[5]>133||Data[5]<131||Data[6]>133||Data[6]<131))GetMotorSpeed(-(Data[5]-132)*0.1/132.0,(Data[6]-132)*0.1/132.0,0) ;
				//PS2_ReadData();
					 if(((~Data[4]&0x10))&&Data[1]==0x73){

		 GlobalStatus=Handle1;
					 }
	    else{
				SendSpeed(0,0,0);
			}					 
		break;			 
		case Handle1 :
			SendSpeed(0,0,0);
						if(Transfer_Color1()){
							GlobalStatus=Pos3;
													 CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+47900);
      servo2(35);	
						}
		break;
		case Pos3 :
//			if(Position_Control(-6172-pos_x_offset,-1181-pos_y_offset,0-zangle_offset)){
//				SendSpeed(0,0,0);
//						 CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+47000);
//      servo2(35);	
//				GlobalStatus=Wait1;//��������
//				//��ס�������ס����һ�����������ֶ���Ƭʱ��ʱͣס
//			}
					PS2_ReadData();
			if(Data[1]==0x73&&(Data[5]>133||Data[5]<131||Data[6]>133||Data[6]<131))GetMotorSpeed(-(Data[5]-132)*0.1/132.0,(Data[6]-132)*0.1/132.0,0) ;
				//PS2_ReadData();
					 if(((~Data[4]&0x10))&&Data[1]==0x73){

		 GlobalStatus=Handle2;//����Wait
					 }
	    else{
				SendSpeed(0,0,0);
			}
		break;
			
//		case Pos3 :
//			if(Position_Control(0-pos_x_offset,0-pos_y_offset,0-zangle_offset)){
//				SendSpeed(0,0,0);GlobalStatus=Wait1;//��������
//				//��ס�������ס����һ�����������ֶ���Ƭʱ��ʱͣס
//			}
//		break;
			
		case Wait1 :
			PS2_ReadData();
		SendSpeed(0,0,0);
		 if(((~Data[4]&0x10))&&Data[1]==0x73){
	
		 GlobalStatus=Handle2;}
		break;
		
		/* 		
		case Trace3_5 :
						if(Trajectory_Planning3_5())GlobalStatus=Pos3;//��������
		break;
				
		case Pos3 :
			if(Position_Control(-8305,-1060,0))GlobalStatus=Handle2;//��������
		break;
		*/
		case Handle2 :
			 SendSpeed(0,0,0);
			if(Transfer_Color2()){GlobalStatus=Trace4;//��������
				CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,17000+offset);
					servo1(63);	
          servo2(60);
			}
		break;
		
		case Trace4 ://��ȥ�ý�����
						if(Trajectory_Planning4()){GlobalStatus=Pos4;
							
						}
						PS2_ReadData();
				if(((~Data[4]&0x40))&&Data[1]==0x73)GlobalStatus=GetBall2;//ֱ�Ӵ��
		break;
			
		case Pos4 :
			      //Pos4OverTime++;
//						if(Position_Control(1900-pos_x_offset,30-pos_y_offset,0-zangle_offset)||Pos4OverTime>1200){
//							SendSpeed(0,0,0);
//							GlobalStatus=Wait2;//��������
//		        //��ס�������ס����һ�����������ֶ���Ƭʱ��ʱͣס
//						}
//						PS2_ReadData();
//				if(((~Data[4]&0x40))&&Data[1]==0x73)GlobalStatus=GetBall2;//ֱ�Ӵ��
		
		PS2_ReadData();
			if(Data[1]==0x73&&(Data[5]>133||Data[5]<131||Data[6]>133||Data[6]<131))GetMotorSpeed(-(Data[5]-132)*0.1/132.0,(Data[6]-132)*0.1/132.0,0) ;
				//PS2_ReadData();
					 if(((~Data[4]&0x10))&&Data[1]==0x73){

		 GlobalStatus=GetBall2;//����Wait
					 }
	    else{
				SendSpeed(0,0,0);
			}
		break;
		
		case Wait2 :
			SendSpeed(0,0,0);
			PS2_ReadData();
		  
		  if(((~Data[4]&0x10))&&Data[1]==0x73){
				GlobalStatus=GetBall2;

			}
		break;
			
		case GetBall2:
						if (Catch_Ball2()){
							GlobalStatus=Trace5;
						}
		break;	

		case Trace5 ://ȥ������
						if(Trajectory_Planning5())GlobalStatus=Pos5;//·�����滮
		break;
    case Pos5   :		//ȥ��λ������
			if(Position_Control(-8205-pos_x_offset,-1161-pos_y_offset,0-zangle_offset))GlobalStatus=Wait3;
								
    			 // if(Position_Control(-8205-pos_x_offset,-1161-pos_y_offset,0-zangle_offset))GlobalStatus=Wait3;//��������
		break;
    
				case Wait3 :
			SendSpeed(0,0,0);
PS2_ReadData();
			if(Data[1]==0x73&&(Data[5]>133||Data[5]<131||Data[6]>133||Data[6]<131))GetMotorSpeed(-(Data[5]-132)*0.2/132.0,(Data[6]-132)*0.2/132.0,0) ;
				//PS2_ReadData();
					 if(((~Data[4]&0x10))&&Data[1]==0x73){

		 GlobalStatus=Handle3;//����Wait
					 }
	    else{
				SendSpeed(0,0,0);
			}
		break;
		
		
    case Handle3 :
			SendSpeed(0,0,0);
				    if(Transfer_Gold())GlobalStatus=Finished;//��������
		
		break;
		
    case Trace6  :
		break;
		
    case Pos6  :
		break;
		
   //case: GetBall 
    case Finished :SendSpeed(0,0,0);
	  break;
		default:
		break;
		
	}
}
//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(int32_t arr,int32_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

extern int status;
int judge = 0;
//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{ 
	
	static float cnt = 0;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)     //����ж�
	{
//	cnt+=0.005;
//	if(cnt<2&&cnt>1){
//		GetMotorSpeed(0.5,0,0);
//		}
//	else SendSpeed(0,0,0);	
//   switch (judge){
//		 case 0:
//			 if(reset())judge=1;
//		 else judge=0;
//		 break;
//		 case 1:
//			 if(Trajectory_Planning1())judge=2;
//		   else judge=1;
//		 break;
//		 case 2:
//			 if(Catch_Ball())judge=3;
//		   else judge=2;
//		 case 3:
//			 SendSpeed(0,0,0);
//		 break;
//		 default:
//		 break;
//	 }
		//Trajectory_Planning4();
		Schedule(GlobalStatus);
		//Position_Control(300,0,0);
		//Trajectory_Planning5();
	}	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);       //����жϱ�־λ

	}
