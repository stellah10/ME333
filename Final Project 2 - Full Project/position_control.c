#include "position_control.h"

void set_position_Kp(float kp) {
    position_Kp = kp;
    return;
}

void set_position_Ki(float ki) {
    position_Ki = ki;
    return;
}

void set_position_Kd(float kd) {
    position_Kd = kd;
    return;
}

volatile float get_position_Kp() {
    return position_Kp;
}

volatile float get_poistion_Ki() {
    return position_Ki;
}

volatile float get_poistion_Kd() {
    return position_Kd;
}