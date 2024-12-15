#include "current_control.h"

void set_current_Kp(float kp) {
    current_Kp = kp;
    return;
}

void set_current_Ki(float ki) {
    current_Ki = ki;
    return;
}

volatile float get_current_Kp() {
    return current_Kp;
}

volatile float get_current_Ki() {
    return current_Ki;
}