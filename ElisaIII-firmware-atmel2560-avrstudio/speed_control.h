
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H


/**
 * \file speed_control.h
 * \brief Speed controler module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The PID controller for the robot velocity is based on the back-emf technique for measuring the 
 current speed of each motor. The module contains both the vertical and horizontal controllers that 
 are activated automatically based on the current orientation of the robot.
 The two controllers work basically the same way, they differ only on the feed forward parameter handling, 
 that in the case of the vertical controller depends on the vetical orientation of the robot (when the robot 
 is moving up then the feed forward is increased, whereas when the robot is moving down then the feed forward 
 is decreased). The controllers are separated for each motor.
 The values of the input arguments and parameters are choosen in order to work only with 2 bytes integers.

*/


#include "variables.h"


/**
 * \brief Initialize the speed controller parameters. Actually not used.
 * \return none
 */
void init_speed_control();

/**
 * \brief Control the speed of the left motor in a vertical wall.
 * \param pwm_left it's a reference; input => desired speed; output => pwm value
 * \return none
 */
void start_vertical_speed_control_left(signed int *pwm_left);

/**
 * \brief Control the speed of the right motor in a vertical wall.
 * \param pwm_right it's a reference; input => desired speed; output => pwm value
 * \return none
 */
void start_vertical_speed_control_right(signed int *pwm_right);

/**
 * \brief Control the speed of the left motor in a flat surface.
 * \param pwm_left it's a reference; input => desired speed; output => pwm value
 * \return none
 */
void start_horizontal_speed_control_left(signed int *pwm_left);

/**
 * \brief Control the speed of the right motor in a flat surface.
 * \param pwm_right it's a reference; input => desired speed; output => pwm value
 * \return none
 */
void start_horizontal_speed_control_right(signed int *pwm_right);

#endif
