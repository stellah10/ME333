#ifndef POSITIONCONTROL__H__
#define POSITIONCONTROL__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro


volatile float position_Kp;
volatile float position_Ki;
volatile float position_Kd;
volatile int desired_angle;

void set_position_Kp(float kp);
void set_position_Ki(float ki);
void set_position_Kd(float kd);

volatile float get_position_Kp();
volatile float get_position_Ki();

#endif 