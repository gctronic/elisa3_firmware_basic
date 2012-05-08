
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

#ifdef __cplusplus
extern "C" {
#endif

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

/**
 * \brief A global variable "clockTick" is incremented at each adc interrupt; this variable is
 * used as base time (104 us resolution). This function is useful for instance to create non-blocking delays, calling 
 * the function at start of delay and then re-calling it to check when the desired delay is passed.
 * \return the current clock ticks since the start
 */
unsigned long int getTime100MicroSec();

/**
 * \brief Simply set the flag that indicates when the battery has to be read.
 * \return none
 */
void readBatteryLevel();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
