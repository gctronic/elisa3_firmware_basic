#include "constants.h"

#ifndef SPEED_CONTROL
#define SPEED_CONTROL

void start_vertical_speed_control(signed long int *pwm_left, signed long int *pwm_right);
void start_orizzontal_speed_control(signed long int *pwm_left, signed long int *pwm_right);
void init_speed_control();

#endif
