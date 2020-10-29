#ifndef __MOTION_H
#define __MOTION_H	 

#include "include.h"
#define POSITION 0
#define SPEED 1
 

typedef signed short int16;
 

extern int16 speed1, speed2, speed3, speed4;
extern double speedy, speedx;

extern int16 pos1, pos2, pos3, pos4;
extern int8_t Motion_Flag;







void Pos_Ctrl(float Des_X, float Des_Y, float, int time);

#endif
