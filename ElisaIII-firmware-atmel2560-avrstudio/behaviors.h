
#ifndef BEHAVIORS_H
#define BEHAVIORS_H

/**
 * \file adc.h
 * \brief Adc module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The adc peripheral is initialized to work in free running mode, raising an interrupt at
 each conversion completion. Within the interrupt service routine the value is saved in its
 correct position and the next channel to sample is selected.
 This is the biggest interrupt in the project and it's used also as the base time for timed 
 processes/funtions (resolution 104 us).

*/


#include "variables.h"


/**
 * \brief Obstacle avoidance behavior based on a simplified force field method.
 * It works for both forward and backward motion. The function need to be called before
 * the speed controller and with a cadency of at least 122 Hz (motors pwm frequency).
 * \return none
 */
void obstacleAvoidance();

/**
 * \brief Cliff avoidance behavior simple implementation in which the robot is stopped when 
 * a cliff is detected with the ground sensors. The threshold used to detect a cliff is extracted
 * from a situation in which the robot is moving in a white surface; in others surfaces the thershold
 * has to be calibrated. The function need to be called before the speed controller and with a 
 * cadency of at least 122 Hz (motors pwm frequency).
 * \return none
 */
char cliffDetected();


#endif
