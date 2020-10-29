
#include "include.h"
#include "adc.h"
#define L 502
#define theta zangle
#define r 0.077
#define T 0.005
#define min_pos 0
#define max_pos 1
float A,B;
#define TooFarYet	(error_x>10||error_y>10||error_x<-10||error_y<-10)//||error_theta>2||error_theta<-2) 
#define MinPower 800
extern CanRxMsg CMFB1;//ChassisMotorFeedBackBuf
extern CanRxMsg CMFB2;
extern CanRxMsg CMFB3;
extern long offset;



/****************************************CAN��ʼ��********************************************************

tsjw:����ͬ����Ծʱ�䵥Ԫ. ��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
tbs2:ʱ���2��ʱ�䵥Ԫ.    ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
tbs1:ʱ���1��ʱ�䵥Ԫ.    ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
brp :�����ʷ�Ƶ��.         ��Χ:1~1024; tq=(brp)*tpclk1
mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ.
������ = Fpclk1/((tbs1+1+tbs2+1+1)*brp);
Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
������Ϊ:42M/((6+7+1)*6)=500Kbps
����ֵ:0,��ʼ��OK;����,��ʼ��ʧ��.   
CAN1_RX:PA11
CAN1_TX:PA12	

************************************************************************************************************/

uint8_t CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    
	  CAN_InitTypeDef        CAN_InitStructure;
  	GPIO_InitTypeDef       GPIO_InitStructure; 
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure; 
	
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
	
    
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);    //ʹ��PORTAʱ��	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);     //ʹ��CAN1ʱ��	
	
    /******************��ʼ��GPIO**************************************************/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             //���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);                   //��ʼ��PA11,PA12
	
	  /******************���Ÿ���ӳ������********************************************/
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	/******************CAN��Ԫ����*************************************************/
   	CAN_InitStructure.CAN_TTCM=DISABLE;	          //��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;           //����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;           //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	          //��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;         	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	          //���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	            //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;             	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1;               //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;               //Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;          //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);           // ��ʼ��CAN1 
    
		/*******************���ù�����**************************************************/
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	                      //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;    //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;                  //32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;              //32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;              //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);                         //�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                        //FIFO0��Ϣ�Һ��ж�����.		    
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	
	  return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	         //ʹ��RX0�ж� 

		    
void CAN1_RX0_IRQHandler(void)   //�жϷ�����	
{
  	CanRxMsg RxMessage;
	  int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	//for(i=0;i<8;i++)
	//printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
}
#endif

#define STOP_MAX 2
#define MaxPower 6400
uint8_t CAN_Send(uint16_t StdId,int16_t motor1,int16_t motor2,int16_t motor3)
{
	CanTxMsg tx;	
//	if(motor1>MaxPower)  motor1=MaxPower;
//	if(motor1<-MaxPower) motor1=-MaxPower;
//	if(motor2>MaxPower)  motor2=MaxPower;
//	if(motor2<-MaxPower) motor2=-MaxPower;
//	if(motor3>MaxPower)  motor3=MaxPower;
//	if(motor3<-MaxPower) motor3=-MaxPower;
	tx.StdId=StdId;
	tx.IDE=CAN_ID_STD;
	tx.RTR=CAN_RTR_DATA;
	tx.DLC=8;
	tx.Data[0]=motor1>>8;       //���������8λ
	tx.Data[1]=motor1&0x00ff;   //���������8λ
	tx.Data[2]=motor2>>8;       //���������8λ
	tx.Data[3]=motor2&0x00ff;   //���������8λ
	tx.Data[4]=motor3>>8;       //���������8λ
	tx.Data[5]=motor3&0x00ff;   //���������8λ
	CAN_Transmit(CAN1,&tx);  
	return 1;
}




//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//����,ʧ��;

uint8_t CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	      // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	      // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		        // ʹ����չ��ʶ��
  TxMessage.RTR=0;		        // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;				  // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];	  // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	
  if(i>=0XFFF)return 1;
  return 0;		

}
//TYPE_Motor_Control_PID Motor_PID;
//int16_t Motor_Filter_Speed[5];



int32_t MotorPID(CanRxMsg* RxMessage,int16_t *now, int32_t des,int32_t *bef,int32_t *diff,int32_t *num)
	{ 
		  float kp=0.18,ki=0,kd=9;
	    int d;
      int send;
	    *now=RxMessage->Data[2];
	  	*now<<=8;
		  *now|=RxMessage->Data[3];
			*now = *now * 0.3 + *bef * 0.7;
			*diff = *diff * 0.8 + (*now-des);
	    d=(int)*now-*bef;
	    *bef=(int)*now;	         
	    //send=-(kp*(now-des)+ki*(*diff)+kd*d);//kp,ki,kd;
			send=kp*(des-*now)+ki*(*diff)-kd*d;
	    *num+=send;
	    return (*num);
  }

	int32_t MotorPID2(CanRxMsg* RxMessage,int16_t *now, int32_t des,int32_t *bef,int32_t *diff,int32_t *num)
	{ 
		  float kp=0.3,ki=0,kd=10;
	    int d;
      int send;
	    *now=RxMessage->Data[2];
	  	*now<<=8;
		  *now|=RxMessage->Data[3];
			*now = *now * 0.3 + *bef * 0.7;
			*diff = *diff * 0.8 + (*now-des);
	    d=(int)*now-*bef;
	    *bef=(int)*now;	         
	    //send=-(kp*(now-des)+ki*(*diff)+kd*d);//kp,ki,kd;
			send=kp*(des-*now)+ki*(*diff)-kd*d;
	    *num+=send;
	    return (*num);
  }
//int32_t SendSpeed(int32_t des1,int32_t des2,int32_t des3)
//	{
//	CanRxMsg RxMessage;
//	static int bef1,bef2,bef3;
//	static int diff1,diff2,diff3;
//	static int num1,num2,num3;
//	short now1, now2, now3;
//	int flag1,flag2,flag3;
//  static int send1,send2,send3;
//  static int cnt;
//  int time=3;	
////		flag1=1;
////		flag2=1;
////		flag3=1;
//  while(time--)
//		{							
//			CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);				
//	    if(RxMessage.StdId==0x201)
//			{
//	    send1=MotorPID(&RxMessage,&now1,des1,&bef1,&diff1,&num1);
////			flag1=0;
//			}
//	    else if(RxMessage.StdId==0x202)
//			{
//	    send2=MotorPID(&RxMessage,&now2,des2,&bef2,&diff2,&num2);
////flag2=0;
//			}				
//	    else if(RxMessage.StdId==0x203)
//			{
//	    send3=MotorPID(&RxMessage,&now3,des3,&bef3,&diff3,&num3);
////flag3=0;
//			}
//		}
//							 
//      CAN_Send(0x200,send1,send2,send3);
//	
//}
int32_t SendSpeed(int32_t des1,int32_t des2,int32_t des3)
	{
	
	static int bef1,bef2,bef3;
	static int diff1,diff2,diff3;
	static int num1,num2,num3;
	short now1, now2, now3;
	int time=3;
  static int send1,send2,send3;
  static int cnt;	
	    send1=MotorPID(&CMFB1,&now1,des1,&bef1,&diff1,&num1);
	    send2=MotorPID2(&CMFB2,&now2,des2,&bef2,&diff2,&num2);						 
	    send3=MotorPID(&CMFB3,&now3,des3,&bef3,&diff3,&num3);//CMFB???ChassisMoTorFeedBackBuf
		  if(send1>MaxPower){send1=MaxPower;send2=MaxPower*send2/send1;send3=MaxPower*send3/send1;
			}
			if(send2>MaxPower){send2=MaxPower;send1=MaxPower*send1/send2;send3=MaxPower*send3/send2;
			}
			if(send3>MaxPower){send3=MaxPower;send1=MaxPower*send1/send3;send2=MaxPower*send2/send3;
			}
			CAN_Send(0x200,send1,send2,send3);
      
}
	
/************************can�ڽ������ݲ�ѯ****************************************

buf:���ݻ�����;	 
����ֵ:0,�����ݱ��յ�;����,���յ����ݳ���.

*********************************************************************************/
uint8_t CAN1_Receive_Msg(uint8_t *buf)
{		   		   
 	  uint32_t i;
	  CanRxMsg RxMessage;
	  int16_t speed;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
			return 0;		                                //û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);     //��ȡ����	
		if(RxMessage.StdId==0x201)
			{
				for(i=0;i<RxMessage.DLC;i++)
        buf[i]=RxMessage.Data[i];  
		    speed=RxMessage.Data[2];
		    speed<<=8;
		    speed|=RxMessage.Data[3];
		    OutData[0]=speed;
		    OutPut_Data();
	    }
			return RxMessage.DLC;	
}







float XErrorBuf[3]={0,0,0};
float YErrorBuf[3]={0,0,0};
float YawErrorBuf[3]={0,0,0};
u8 Arrived=0;
u8 XBufIndex=0;
u8 YBufIndex=0;
u8 YawBufIndex=0;

   float Kp_x=2;
	 float Ki_x=0.2;
	 float Kd_x=2;
	
	 float Kp_y=2;
	 float Ki_y=0.2;
	 float Kd_y=2;
	
	 float Kp_theta=0.1;
	 float Ki_theta=0.03;
	 float Kd_theta=0.1;

u8 Position_Control(float X_target,float Y_target,float Theta_target)
{  
	

  float X =0;
	float Y =0;
	float Theta =0;
	int motor1=0,motor2=0,motor3=0;

	static float integral_x=0;
	static float integral_y=0;
	static float integral_theta=0;

	static float error_x = 0, error_last1_x=0,error_last2_x=0;
	static float error_y = 0, error_last1_y=0,error_last2_y=0;
	static float error_theta = 0, error_last1_theta=0, error_last2_theta=0;///////////////////////
	
	error_x = X_target-pos_x;
	error_y = Y_target-pos_y;
	error_theta = Theta_target-zangle;

	
	
	integral_x-=XErrorBuf[XBufIndex];
	integral_x+=error_x;                  //���»�����
	XErrorBuf[XBufIndex]=error_x;         //���»���
	XBufIndex=(XBufIndex+1)%3;
	
	integral_y-=YErrorBuf[YBufIndex];
	integral_y+=error_y;                  //���»�����
	YErrorBuf[YBufIndex]=error_y;         //���»���
	YBufIndex=(YBufIndex+1)%3;
	
	integral_theta-=YawErrorBuf[YawBufIndex];
	integral_theta+=error_theta;                  //���»�����
	YawErrorBuf[YawBufIndex]=error_theta;//���»���
	YawBufIndex=(YawBufIndex+1)%3;
	

	X = Kp_x*error_x+Ki_x*integral_x+Kd_x*(error_x-error_last1_x);
	Y = Kp_y*error_y+Ki_y*integral_y+Kd_y*(error_y-error_last1_y);
	Theta = Kp_theta*error_theta+Ki_theta*integral_theta+Kd_theta*(error_theta-error_last1_theta);
	motor2 = -(int)(sin(theta /180.0f*3.1415926f)* Y + cos(theta / 180.0f*3.1415926f) * X + L * Theta);                   //1
	motor3 = -(int)(sin((60 - theta) / 180.0f*3.1415926f)* Y - cos((60 - theta) / 180.0f * 3.1415926f)* X + L * Theta);   //2
  motor1 = -(int)(-sin((theta + 60) / 180.0f*3.1415926f)* Y - cos((60 + theta) / 180.0f * 3.1415926f)* X + L * Theta);  //3
  

  

	error_last2_x =error_last1_x;
	error_last2_y =error_last1_y;
	error_last1_x = error_x;
	error_last1_y = error_y;
	error_last2_theta=error_last1_theta;
	error_last1_theta = error_theta;                //����error����
	
		if((abs(error_x)<4)&&(abs(error_y)<4)&&abs(error_theta)<0.8){
SendSpeed(0,0,0);
		return 0x01;}
	else{
		SendSpeed( motor1, motor2, motor3);
		return 0x00;
  }

}








void GetMotorSpeed(float Vx,float Vy,float W)         //�����������ٶ��Լ����ٶ�ת�����������ת��
{ 
	float motor1,motor2,motor3;
	int k=2;
	//static int cnt;
	
	motor2 = (sin(theta /180.0f*3.1415926f)* Vy + cos(theta / 180.0f*3.1415926f) * Vx + L * W);  //1
	motor3 = (sin((60 - theta) / 180.0f*3.1415926f)* Vy - cos((60 - theta) / 180.0f * 3.1415926f)* Vx + L * W);//2
  motor1 = (-sin((theta + 60) / 180.0f*3.1415926f)* Vy - cos((60 + theta) / 180.0f * 3.1415926f)* Vx + L * W);//3
	motor2 =(int)(-motor2*30*19/(3.1415926f*r));
	motor1 =(int)(-motor1*30*19/(3.1415926f*r));
	motor3 =(int)(-motor3*30*19/(3.1415926f*r));
	
	SendSpeed(motor1,motor2,motor3);
}


float power(float base, int n)                        //��base��n�η�                  
{
	  int i;
	  float p;
    p = 1;
    for(i = 1; i <= n; ++i)
        p = p * base;
    return p;
}



//extern float t;

/*************************************************************************
                                ·���滮
���������ݹ滮�õ�·�������߼�����һʱ���ٶȣ����ݷ���λ����Ϣ���ٶȽ��в�����
Trajectory_Planning1(): ��ȡ��㵽��һ������λ��
Trajectory_Planning2()���ӵ�һ������λ�õ�ȡ���
Trajectory_Planning3()����ȡ��㵽�ڶ�������λ��
*************************************************************************/
//int judge;
u8 Trajectory_Planning1()                //�ӳ����㵽ȡ����
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

//	float ax = 0.0444444444;       
//	float bx = -0.33333333;  
//	float cx = 0.6666666667;//����1.8��

	float ax = 0.52500;       
	float bx = -2.625;  
	float cx = 3.5;//����1.8��
////////////////////////////////ֱ��///////////////////////  
  Vgx = 5*ax*power(t,4)+4*bx*power(t,3)+3*cx*power(t,2); 
  Vgy = 0.018*Vgx;
//////////////////////////////////////////////////////////// 

  t+=0.005;

	if(t>=2)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(Vgx,Vgy,0);
	return 0x00;
	}
}
u8 


u8 Trajectory_Planning1_()                //�ӳ����㵽ȡ����
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

////////////////////////////////ֱ��///////////////////////  
	if(t<=0.5){Vgx=1.86667/0.5*t;}
  else if(t<=1.5){Vgx=1.86667;}
	else if(t<=2.0){Vgx=1.86667/0.5*(2.0-t);}
  Vgy = 0.018*Vgx;
//////////////////////////////////////////////////////////// 

  t+=0.005;

	if(t>2)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(Vgx,Vgy,0);
	return 0x00;
	}
}


u8 Trajectory_Planning2()                //��ȡ��������һ������
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

//	float ax = -0.0014804;       
//	float bx = 0.02961;  
//	float cx = -0.15791;//8��8.085��
	float ax = -0.0032433;       
	float bx = 0.0567576;  
	float cx = -0.2648688;//8-9085

////////////////////////////////ֱ��////////////////////////  
  Vgx = 5*ax*power(t,4)+4*bx*power(t,3)+3*cx*power(t,2); 

  Vgy = 1.2*0.13111*Vgx; 
//////////////////////////////////////////////////////////// 
	  t+=0.005;
	if(t>2){
					CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+89900);
     servo1(45);
	}

	if(t>=6.5)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(Vgx,Vgy,0);
	return 0x00;
	}
}


u8 Trajectory_Planning3()                //�˺�ȴ�
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

	float ax = -0.04938;       
	float bx = 0.37037037;  
	float cx = -0.740741; //x����2�ף�y�����˺�1.5��
	

////////////////////////////////б��ֱ��////////////////////////  
  Vgx = 5*ax*power(t,4)+4*bx*power(t,3)+3*cx*power(t,2); 

  Vgy = -0.55*Vgx; 
//////////////////////////////////////////////////////////// 

  t+=0.005;
	if(t>=3)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(Vgx,Vgy,0);
	return 0x00;
	}
}






u8 Trajectory_Planning3_5()                //ǰ���ڶ����������ݲ���2��
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

	float ay = -0.1875;       
	float by = 0.9375;  
	float cy = -1.25;//y����ǰ��1.5��
	

////////////////////////////////��Բ��////////////////////////  
  Vgy = 5*ay*power(t,4)+4*by*power(t,3)+3*cy*power(t,2); 
//////////////////////////////////////////////////////////// 

  t+=0.005;
	if(t>=2)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(0,Vgy,0);
	return 0x00;
	}
}

u8 Trajectory_Planning4()                //��ȥȡ����
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

	//float ax = 0.0015015;       
	//float bx = -0.023548;  
//	float cx = 0.14129;
	
//	float ax = 0.0015;       
//	float bx = -0.0304;  
//	float cx = 0.1621;
	
	float ax = 0.001700134;       
	float bx = -0.0340027;  
	float cx = 0.1813476;//8 9285
////////////////////////////////��Բ��////////////////////////  
  Vgx = 5*ax*power(t,4)+4*bx*power(t,3)+3*cx*power(t,2); 

  Vgy = 0.22*Vgx; 
//////////////////////////////////////////////////////////// 
  if(t>5){
					CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+27000);
		  servo1(63);	
      servo2(65);	
	}
  t+=0.005;
	if(t>=7.4)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(Vgx,Vgy,0);
	return 0x00;
	}
}


u8 Trajectory_Planning5()                //ǰ���ڶ����������ݽ���
{
  static float t=0;

	float Vgx;                 //volocity goal Ŀ���ٶ�
	float Vgy;

	float ax = -0.0011487;       
	float bx = 0.0258459;  
	float cx = -0.1550754;//9-11.305
	

////////////////////////////////��Բ��////////////////////////  
  Vgx = 5*ax*power(t,4)+4*bx*power(t,3)+3*cx*power(t,2); 

  Vgy = 1.3*0.1029*Vgx; 
//////////////////////////////////////////////////////////// 
  
  t+=0.005;
	if(t>3){
			CAN_RoboModule_DRV_Velocity_Position_Mode(0,1,5000,-1500,offset+86900);
      servo1(45);	
      servo2(60);
	}		
	if(t>=8.5)
	{
	SendSpeed(0,0,0);
	return 0x01;
	}
	else { 
	GetMotorSpeed(Vgx,Vgy,0);
	return 0x00;
	}
}
float Y_control;
float temp_t,distance;
u8 Pos_Laser_Control(float aim)          //������pid��������
{ 
	u16 adcx;
	
	static float err_last;
	float err;
	static float err_i;
	static float err_d;
	
	float Kp=1,Ki=0,Kd=1.5;
	
//	if(aim<min_pos||aim>max_pos)
//		return;

	adcx=Get_Adc(ADC_Channel_5);//��ȡͨ��5��ת��
	temp_t=(float)adcx*(3.3/4096);          //��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
	distance=(87.055*temp_t-26.33)/100;//ʵ�ʾ��� ��
	err=distance-aim;
	err_i+=err; 
	err_d=err-err_last;
	err_last=err;
	Y_control=Kp*err+Ki*err_i+Kd*err_d;
	if(abs(distance-0.18)<0.02)
	   SendSpeed(0,0,0);
	else GetMotorSpeed(0,-Y_control,0);
}




unsigned int CAN_Time_Out = 0;

static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}


void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x0000;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}



unsigned char can_tx_success_flag = 0;
/*************************************************************************
                          CAN1_TX_IRQHandler
������CAN1�ķ����жϺ���
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}

/****************************************************************************************
                                       ��λָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                     ģʽѡ��ָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

Mode    ȡֵ��Χ

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                     ģʽѡ��ָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

Mode    ȡֵ��Χ

Get_Current_Velocity_Position_Mode                              0x0B
Get_CTL1_CTL2_Mode                                              0x0C
*****************************************************************************************/
void CAN_RoboModule_DRV_FeedbackMode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

void CAN_RoboModule_DRV_Get_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{ 
    unsigned short can_id = 0x00B;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = 0x0A;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
	}

void CAN_RoboModule_DRV_Get_CTL1_CTL2_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{ 
    unsigned short can_id = 0x00C;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x05;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
	}
	
	

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = ��5000ʱ����������ѹΪ��Դ��ѹ

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_current��ȡֵ��Χ���£�
-32768 ~ +32767����λmA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   �ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  �ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  �����ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  ����λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  �����ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      ����ָ��
Temp_Time��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�رյ����ٶ�λ�÷�������
Ctl1_Ctl2��ȡֵ��Χ��0 or 1 ������Ϊ0 or 1������Ϊ��0��Ϊ�ر�������λ��⹦��
�ر���ʾ��Ctl1��Ctl2�Ĺ��ܽ�������102 301������汾��������Ctl1_Ctl2 = 0 ����
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = Temp_Time1;
    tx_message.Data[1] = Temp_Time2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      ���߼��
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

//���������ݵĺ�����Ĭ��Ϊ4����������������0�飬���Ϊ1��2��3��4
/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/

uint8_t Reach_position;
uint8_t Limit_switch1;
uint8_t Limit_switch2;
uint8_t Limit_switch3;
uint8_t Light_LED;


void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        	    if(rx_message.StdId==0x201){
	    CMFB1=rx_message;
				goto DR;
			}
			 else   if(rx_message.StdId==0x202){
	    CMFB2=rx_message;
				 goto DR;
			 }
			 else   if(rx_message.StdId==0x203){
	    CMFB3=rx_message;
				 goto DR;
			 }
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //��׼֡������֡�����ݳ���Ϊ8
        {
            if(rx_message.StdId == 0x1B)
            {
                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
							  if(Real_Velocity_Value[0]>=2990)
									Light_LED = 1;
                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
                if(Real_Position_Value[0]>=141996)
									Reach_position = 1;
						}
            else if(rx_message.StdId == 0x2B)
            {
                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
                if(Real_Position_Value[1]>=236650&&Real_Position_Value[1]<=236670)

								Reach_position = 1;
						}
            else if(rx_message.StdId == 0x3B)
            {
                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
                if(Real_Position_Value[2]>=236650&&Real_Position_Value[2]<=236670)
									Reach_position = 1;           
						}
            else if(rx_message.StdId == 0x4B)
            {
                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
                if(Real_Position_Value[3]>=236650&&Real_Position_Value[3]<=236670)
									Reach_position = 1;
						}
//            else if(rx_message.StdId == 0x1F)
//            {
//                Real_Online[0] = 1;
//            }
//            else if(rx_message.StdId == 0x2F)
//            {
//                Real_Online[1] = 1;
//            }
//            else if(rx_message.StdId == 0x3F)
//            {
//                Real_Online[2] = 1;
//            }
//            else if(rx_message.StdId == 0x4F)
//            {
//                Real_Online[3] = 1;
//            }
            else if(rx_message.StdId == 0x1C)
            {
                Real_Ctl1_Value[0] = rx_message.Data[0];
                Real_Ctl2_Value[0] = rx_message.Data[1];
							if(Real_Ctl1_Value[0]==1)
								Limit_switch3=1;
            }
            else if(rx_message.StdId == 0x2C)
            {
                Real_Ctl1_Value[1] = rx_message.Data[0];
                Real_Ctl2_Value[1] = rx_message.Data[1];
							//if(Real_Ctl1_Value[1]==1)
								//GPIO_ResetBits(GPIOF,GPIO_Pin_2);//Limit_switch1 = 1;
							//if(Real_Ctl2_Value[1]==1)
								//GPIO_ResetBits(GPIOF,GPIO_Pin_2);//Limit_switch2 = 1;
            }
            else if(rx_message.StdId == 0x3C)
            {
                Real_Ctl1_Value[2] = rx_message.Data[0];
                Real_Ctl2_Value[2] = rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x4C)
            {
                Real_Ctl1_Value[3] = rx_message.Data[0];
                Real_Ctl2_Value[3] = rx_message.Data[1];
            }

        }
                
				}DR:;
}
