#ifndef UTILITIES__H__
#define UTILITIES__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

#define DIRECTION LATBbits.LATB12 // motor direction 

volatile int cur_pwm;
volatile int cur_mode;

// int pos_len = 0;


// #include "nu32dip.h"

// an enum of possible states, public in the utilities h file
enum mode_t {IDLE, PWM, ITEST, HOLD, TRACK}; 

// volatile int pwm = 0;

// volatile enum mode_t mode; 

// The set and get prevent outsiders from messing with the variables directly

// function definitions in the utilities.c file:
volatile enum mode_t get_mode(void);
void set_mode(enum mode_t s);
void set_pwm(int p);
int get_pwm(void);

// void set_pos_length(int length);
// int get_pos_length();

#endif 
