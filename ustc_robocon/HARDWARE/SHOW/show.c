#include "show.h"



void showint16(int i, ...)  
{  
  va_list arg_ptr;  
  u16 j=0;   
  int k = 0;  
	printf("%c%c",0x03,0xFC);
	va_start(arg_ptr, i);  
	for(k=0;k<i;k++){
		j=va_arg(arg_ptr, uint16_t);
		printf("%c%c",(u8)(j&0x00FF),(u8)(j>>8));
	}
		va_end(arg_ptr);  
	for(k=i;k<8;k++){
		printf("%c%c",0x00,0x00);	
	}
	printf("%c%c",0xFC,0x03);
	return;  
} 

void showint32(int i, ...)  
{  
  va_list arg_ptr;  
  u32 j=0;   
  int k = 0;  
	printf("%c%c",0x03,0xFC);
	va_start(arg_ptr, i);  
	for(k=0;k<i;k++){
		j=va_arg(arg_ptr, u32);
		printf("%c%c%c%c",(u8)(j&0x000000FF),(u8)((j>>8)&0x000000FF),(u8)((j>>16)&0x000000FF),(u8)((j>>24)&0x000000FF));
	}
		va_end(arg_ptr);  
	for(k=i;k<8;k++){
		printf("%c%c%c%c",0x00,0x00);	
	}
	printf("%c%c",0xFC,0x03);
	return;  
} 


void showfloat(int i, ...)  
{  
	static union{
		u8 a[4];
		float x;
	}unit[8];
	
  va_list arg_ptr;   
  int k = 0;  
	printf("%c%c",0x03,0xFC);
	va_start(arg_ptr, i);  
	for(k=0;k<i;k++){
		unit[k].x=va_arg(arg_ptr, float);
		printf("%c%c%c%c",unit[k].a[0],unit[k].a[1],unit[k].a[2],unit[k].a[3]);
	}
		va_end(arg_ptr);  
	for(k=i;k<8;k++){
		printf("%c%c%c%c",0x00,0x00,0x00,0x00);	
	}
	printf("%c%c",0xFC,0x03);
	return;  
} 

void showdouble(int i, ...)  
{  
	static union{
		u8 a[8];
		float x;
	}unit[8];
	
  va_list arg_ptr;    
  int k = 0;  
	printf("%c%c",0x03,0xFC);
	va_start(arg_ptr, i);  
	for(k=0;k<i;k++){
		unit[k].x=va_arg(arg_ptr, float);
		printf("%c%c%c%c%c%c%c%c",unit[k].a[0],unit[k].a[1],unit[k].a[2],unit[k].a[3],unit[k].a[4],unit[k].a[5],unit[k].a[6],unit[k].a[7]);
	}
		va_end(arg_ptr);  
	for(k=i;k<8;k++){
		printf("%c%c%c%c%c%c%c%c",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);	
	}
	printf("%c%c",0xFC,0x03);
	return;  
} 




