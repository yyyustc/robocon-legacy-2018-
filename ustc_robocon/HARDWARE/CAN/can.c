
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



/****************************************CAN初始化********************************************************

tsjw:重新同步跳跃时间单元. 范围:CAN_SJW_1tq~ CAN_SJW_4tq
tbs2:时间段2的时间单元.    范围:CAN_BS2_1tq~CAN_BS2_8tq;
tbs1:时间段1的时间单元.    范围:CAN_BS1_1tq ~CAN_BS1_16tq
brp :波特率分频器.         范围:1~1024; tq=(brp)*tpclk1
mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式.
波特率 = Fpclk1/((tbs1+1+tbs2+1+1)*brp);
Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
则波特率为:42M/((6+7+1)*6)=500Kbps
返回值:0,初始化OK;其他,初始化失败.   
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
	
    
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);    //使能PORTA时钟	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);     //使能CAN1时钟	
	
    /******************初始化GPIO**************************************************/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             //复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                   //初始化PA11,PA12
	
	  /******************引脚复用映射配置********************************************/
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	/******************CAN单元设置*************************************************/
   	CAN_InitStructure.CAN_TTCM=DISABLE;	          //非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;           //软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;           //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	          //禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;         	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	          //优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	            //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;             	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1;               //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;               //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;          //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);           // 初始化CAN1 
    
		/*******************配置过滤器**************************************************/
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	                      //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;    //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;                  //32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;              //32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;              //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);                         //滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                        //FIFO0消息挂号中断允许.		    
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	
	  return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	         //使能RX0中断 

		    
void CAN1_RX0_IRQHandler(void)   //中断服务函数	
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
	tx.Data[0]=motor1>>8;       //电机电流高8位
	tx.Data[1]=motor1&0x00ff;   //电机电流地8位
	tx.Data[2]=motor2>>8;       //电机电流高8位
	tx.Data[3]=motor2&0x00ff;   //电机电流地8位
	tx.Data[4]=motor3>>8;       //电机电流高8位
	tx.Data[5]=motor3&0x00ff;   //电机电流地8位
	CAN_Transmit(CAN1,&tx);  
	return 1;
}




//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//其他,失败;

uint8_t CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	      // 标准标识符为0
  TxMessage.ExtId=0x12;	      // 设置扩展标示符（29位）
  TxMessage.IDE=0;		        // 使用扩展标识符
  TxMessage.RTR=0;		        // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;				  // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];	  // 第一帧信息          
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
	
/************************can口接收数据查询****************************************

buf:数据缓存区;	 
返回值:0,无数据被收到;其他,接收的数据长度.

*********************************************************************************/
uint8_t CAN1_Receive_Msg(uint8_t *buf)
{		   		   
 	  uint32_t i;
	  CanRxMsg RxMessage;
	  int16_t speed;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
			return 0;		                                //没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);     //读取数据	
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
	integral_x+=error_x;                  //更新积分量
	XErrorBuf[XBufIndex]=error_x;         //更新缓存
	XBufIndex=(XBufIndex+1)%3;
	
	integral_y-=YErrorBuf[YBufIndex];
	integral_y+=error_y;                  //更新积分量
	YErrorBuf[YBufIndex]=error_y;         //更新缓存
	YBufIndex=(YBufIndex+1)%3;
	
	integral_theta-=YawErrorBuf[YawBufIndex];
	integral_theta+=error_theta;                  //更新积分量
	YawErrorBuf[YawBufIndex]=error_theta;//更新缓存
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
	error_last1_theta = error_theta;                //更新error缓存
	
		if((abs(error_x)<4)&&(abs(error_y)<4)&&abs(error_theta)<0.8){
SendSpeed(0,0,0);
		return 0x01;}
	else{
		SendSpeed( motor1, motor2, motor3);
		return 0x00;
  }

}








void GetMotorSpeed(float Vx,float Vy,float W)         //将机器人线速度以及角速度转换成三个电机转速
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


float power(float base, int n)                        //求base的n次方                  
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
                                路径规划
描述：根据规划好的路径，在线计算下一时刻速度，根据反馈位置信息对速度进行补偿。
Trajectory_Planning1(): 从取球点到第一个交接位置
Trajectory_Planning2()：从第一个交接位置到取球点
Trajectory_Planning3()：从取球点到第二个交接位置
*************************************************************************/
//int judge;
u8 Trajectory_Planning1()                //从出发点到取球区
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
	float Vgy;

//	float ax = 0.0444444444;       
//	float bx = -0.33333333;  
//	float cx = 0.6666666667;//三秒1.8米

	float ax = 0.52500;       
	float bx = -2.625;  
	float cx = 3.5;//三秒1.8米
////////////////////////////////直线///////////////////////  
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


u8 Trajectory_Planning1_()                //从出发点到取球区
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
	float Vgy;

////////////////////////////////直线///////////////////////  
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


u8 Trajectory_Planning2()                //从取球区到第一交接区
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
	float Vgy;

//	float ax = -0.0014804;       
//	float bx = 0.02961;  
//	float cx = -0.15791;//8秒8.085米
	float ax = -0.0032433;       
	float bx = 0.0567576;  
	float cx = -0.2648688;//8-9085

////////////////////////////////直线////////////////////////  
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


u8 Trajectory_Planning3()                //退后等待
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
	float Vgy;

	float ax = -0.04938;       
	float bx = 0.37037037;  
	float cx = -0.740741; //x方向2米，y方向退后1.5米
	

////////////////////////////////斜后方直线////////////////////////  
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






u8 Trajectory_Planning3_5()                //前往第二交接区（递彩球2）
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
	float Vgy;

	float ay = -0.1875;       
	float by = 0.9375;  
	float cy = -1.25;//y方向前进1.5米
	

////////////////////////////////椭圆弧////////////////////////  
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

u8 Trajectory_Planning4()                //回去取金球
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
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
////////////////////////////////椭圆弧////////////////////////  
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


u8 Trajectory_Planning5()                //前往第二交接区（递金球）
{
  static float t=0;

	float Vgx;                 //volocity goal 目标速度
	float Vgy;

	float ax = -0.0011487;       
	float bx = 0.0258459;  
	float cx = -0.1550754;//9-11.305
	

////////////////////////////////椭圆弧////////////////////////  
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
u8 Pos_Laser_Control(float aim)          //激光测距pid调整距离
{ 
	u16 adcx;
	
	static float err_last;
	float err;
	static float err_i;
	static float err_d;
	
	float Kp=1,Ki=0,Kd=1.5;
	
//	if(aim<min_pos||aim>max_pos)
//		return;

	adcx=Get_Adc(ADC_Channel_5);//获取通道5的转换
	temp_t=(float)adcx*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
	distance=(87.055*temp_t-26.33)/100;//实际距离 米
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
描述：CAN1的发送中断函数
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
                                       复位指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

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
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

Get_Current_Velocity_Position_Mode                              0x0B
Get_CTL1_CTL2_Mode                                              0x0C
*****************************************************************************************/
void CAN_RoboModule_DRV_FeedbackMode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                  电流速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                  电流位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    
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
                                  电流速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                      配置指令
Temp_Time的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Ctl1_Ctl2的取值范围：0 or 1 ，当不为0 or 1，则认为是0，为关闭左右限位检测功能
特别提示：Ctl1，Ctl2的功能仅存在于102 301，其余版本驱动器，Ctl1_Ctl2 = 0 即可
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                      在线检测
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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

//本接收数据的函数，默认为4个驱动器，都挂在0组，编号为1、2、3、4
/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
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
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
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
