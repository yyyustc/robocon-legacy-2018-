#define StandBy 0x00 //开机待命 JS
#define Trace1 0x14//
#define Pos1 0x01  //拿彩球
#define GetBall1 0x02 //拿球
#define Trace2 0x03   //去递彩球
#define Pos2 0x04 //定位到第一递位
#define Handle1 0x05 //递彩球
//#define Trace3 0x06 //去待递第二个
#define Wait1 0x07 //等着递第二个 JS
//#define Trace3_5 0x15
#define Pos3 0x08 //定位到第二递位
#define Handle2 0x09 //递彩球
#define Trace4 0x0A //跑回去拿金球
#define Pos4 0x0B //定位拿金球位
#define Wait2 0x0C //等待投进 JS
#define GetBall2 0x13//第二次拿球
#define Trace5 0x0D //去递金球
#define Pos5 0x0E //定位递金球
#define Wait3 0x17
#define Handle3 0x0F //递金球
#define Trace6 0x10 //后备取金球
#define Pos6 0x11 //后备 取金球定位
//#define GetBall 0x01
#define Finished 0x12 //完成比赛
#include "include.h"
#include "led.h"
float t=0;
short temp_pwm = 5000;
#define Analog (Data[1]==0x73)
#define BACKMOVE if(GlobalStatus==Wait2)GlobalStatus=Pos4;if(GlobalStatus==Wait1)GlobalStatus=Pos3;if(GlobalStatus==Wait3)GlobalStatus=Pos5;
extern u8 flagna;
extern long offset;
////*全局变量*//////////////////////////////////////////////////////////////////////////////////////

//u8 GlobalStatus=0x00;
u8 GlobalStatus=0x00;
float pos_x_offset=0;
float pos_y_offset=0;
float zangle_offset=0;
u8 DATAWBefore=0xFF;
u8 DATAYBefore=0xFF;
//u8 PosEnabled=0;//这个全局标志位用来开关定时器中的POSCTRL(timer.c
//float DesX=0,DesY=0,DesW=0;//全局变量用来传递POSCTRL的目标位置
//u8 PosArrived=0;//用来判断有没有到达目标位置
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
    //  CAN1_Configuration();                               //CAN1初始化
	   // CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN初始化为正常模式，波特率1000Kbps    
	    CAN1_Configuration();
		  PS2_Init();
	    LED_Init();
	    Adc_Init();         //初始化ADC
      delay_init(168);    //初始化延时函数
  	  uart_init(115200);
      
      delay_ms(500);                                      //刚开始要有足够的延时，确保驱动器已经初始化完成 
      CAN_RoboModule_DRV_Reset(0,0);                      //对0组所有驱动器进行复位 
      delay_ms(500); 
	    CAN_RoboModule_DRV_Config(0,1,10,10);               //1号驱动器配置为1ms传回一次数据          
      delay_us(200);                                      //此处延时为了不让传回数据时候4个不一起传
		  CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Position_Mode);//0组电机进入位置速度模式
		  
		  usart3_init(115200);	//初始化串口波特率为115200
	    	                     //定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数10次为1ms
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
				//胶片
				PS2_ReadData();
				if(Analog){
				if((~Data[3]&0x10)&&(!(~DATAWBefore&0x10))){pos_y_offset+=20.0;BACKMOVE}//负负得正，检测到UP键上升沿，零点向上偏移（操作者视角） 
				if((~Data[3]&0x20)&&(!(~DATAWBefore&0x20))){pos_x_offset+=20.0;BACKMOVE}//负负得正，检测到RIGHT键上升沿，零点右偏
				if((~Data[3]&0x40)&&(!(~DATAWBefore&0x40))){pos_y_offset-=20.0;BACKMOVE}//负正得负觳獾紻OWN键上升沿，零点下
				if((~Data[3]&0x80)&&(!(~DATAWBefore&0x80))){pos_x_offset-=20.0;BACKMOVE}//负正得负，检测到LEFT键上升沿，零点左
					
				if((~Data[4]&0x04)&&(!(~DATAYBefore&0x04))){zangle_offset-=2.0;BACKMOVE}//负正得负L1
				if((~Data[4]&0x08)&&(!(~DATAYBefore&0x08))){zangle_offset+=2.0;BACKMOVE}//负正得负  R1		

				
				DATAWBefore=Data[3];
				DATAYBefore=Data[4];
					}
			//	sendbuf=(int)pos_x;
			//USART_SendData  (USART1,sendbuf);
			}



}
