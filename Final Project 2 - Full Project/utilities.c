#include "utilities.h"

// variable
//volatile enum mode_t mode; 
// volatile int pwm = 0;



volatile enum mode_t get_mode(void){
    return cur_mode;
}

void set_mode(enum mode_t s) {
    cur_mode = s;
}

// void set_pos_length(int length) {
//     pos_len = length;
//     return;
// }

// int get_pos_length() {
//     return pos_len;
// }

// void set_pwm(int p) {
//     pwm = p;
// }

// int get_pwm(void) {
//     return pwm;
// }
