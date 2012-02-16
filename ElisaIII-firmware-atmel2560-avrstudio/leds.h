#ifndef LEDS_H
#define LEDS_H

/**
 * \file leds.h
 * \brief Leds module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The RGB leds are handled with the timer1 that can manage three different pwm, one used for each
 color. The frequency for the three pwm remains the same, but the duty cycle (led luminosity) can be 
 changed independently from each other, letting creating a wide range of different colors.
 Moreover the module contains functions for handling the small green leds placed around the robot.

*/


#include "variables.h"

/**
 * \brief Configure the adc registers and start the sampling.
 * \return none
 */
void initRGBleds();

/**
 * \brief Toggle the state of the blue led.
 * \return none
 */
void toggleBlueLed();

/**
 * \brief Update the value of the red led accordingly to the received commands.
 * \return none
 */
void updateRedLed(unsigned char value);

/**
 * \brief Update the value of the green led accordingly to the received commands.
 * \return none
 */
void updateGreenLed(unsigned char value);

/**
 * \brief Update the value of the blue led accordingly to the received commands.
 * \return none
 */
void updateBlueLed(unsigned char value);

#endif

