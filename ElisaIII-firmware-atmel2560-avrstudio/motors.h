
#ifndef MOTORS_H
#define MOTORS_H


/**
 * \file motors.h
 * \brief Motors module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The motors are handled with two timers: timer3 for right motor and timer4 for left motor.
 Two independent timers are needed because the timer will reset when the output compare register 
 matches the timer value. The settings for the two timers are the same; the related pwm frequency 
 is 122 Hz and the duty cycle define the motors speed.
 In order to measure the consumption and speed of the motors respectively in the active and passive 
 phase of the pwm, two interrupts per timer are generated: one at the beginning of the cycle (timer 
 overflow) and another one when when the output compare matches (beginning of passive phase).

*/


#include "variables.h"
#include <avr\io.h>
#include <avr\interrupt.h>

/**
 * \brief Configure the timer3 and timer4 registers to work at about 122 Hz.
 * \return none
 */
void initMotors();

#endif

