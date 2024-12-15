#ifndef CURRENTCONTROL__H__
#define CURRENTCONTROL__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro


volatile float current_Kp;
volatile float current_Ki;

void set_current_Kp(float kp);
void set_current_Ki(float ki);
volatile float get_current_Kp();
volatile float get_current_Ki();

#endif 
