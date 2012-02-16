#include "variables.h"

#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

void start_vertical_speed_control_left(signed int *pwm_left);
void start_vertical_speed_control_right(signed int *pwm_right);
void start_horizontal_speed_control_left(signed int *pwm_left);
void start_horizontal_speed_control_right(signed int *pwm_right);
void init_speed_control();

#endif
