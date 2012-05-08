
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
#include "behaviors.h"
#include "speed_control.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure the timer3 and timer4 registers to work at about 122 Hz.
 * \return none
 */
void initMotors();

/**
 * \brief Change the motors speed (pwm registers) accordingly to the obstacle avoidance (if enabled).
 * In this function both the motors velocities measurements are also updated (even if not needed).
 * \return none
 */
void handleMotorsWithSpeedController();

/**
 * \brief Change the motors speed (pwm registers) accordingly to the obstacle avoidance (if enabled) and speed controller.
 * In this function both the motors velocities measurements are also updated.
 * \return none
 */
void handleMotorsWithNoController();

/**
 * \brief Set the desired speed for the left motor.
 * \param vel desired velocity to set (from -100 to 100)
 * \return none
 */
void setLeftSpeed(signed char vel);

/**
 * \brief Set the desired speed for the right motor.
 * \param vel desired velocity to set (from -100 to 100)
 * \return none
 */
void setRightSpeed(signed char vel);


#ifdef __cplusplus
} // extern "C"
#endif

#endif

