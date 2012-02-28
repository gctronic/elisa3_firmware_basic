
#ifndef UTILITY_H
#define UTILITY_H


/**
 * \file utility.h
 * \brief Utility module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The module contains various functions that aren't related to any other module, they 
 are instead of general use.
*/


#include "variables.h"
#include <avr\io.h>
#include <avr\interrupt.h>
#include <avr\sleep.h>
#include <avr\eeprom.h>
#include "ports_io.h"
#include "adc.h"
#include "motors.h"
#include "leds.h"
#include "spi.h"
#include "mirf.h"
#include "usart.h"
#include "sensors.h"
#include "ir_remote_control.h"


/**
 * \brief Return the current selector postion.
 * \return byte representing the selector postion (0..15).
 */
unsigned char getSelector();

/**
 * \brief Initialize all the port pins and peripherals calling their "init" functions.
 * \return none
 */
void initPeripherals();

/**
 * \brief Let the roboot go in extended standby mode.
 * \param seconds number of seconds to stay in sleep
 * \return none
 */
void sleep(unsigned char seconds);

#endif
