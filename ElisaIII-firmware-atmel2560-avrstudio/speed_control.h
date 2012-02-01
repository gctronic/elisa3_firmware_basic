#include "constants.h"

#ifndef SPEED_CONTROL
#define SPEED_CONTROL

void start_vertical_speed_control_left(signed int *pwm_left);
void start_vertical_speed_control_right(signed int *pwm_right);
void start_orizzontal_speed_control_left(signed int *pwm_left);
void start_orizzontal_speed_control_right(signed int *pwm_right);
void init_speed_control();

#endif
