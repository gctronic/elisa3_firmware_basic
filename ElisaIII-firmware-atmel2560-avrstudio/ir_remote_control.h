
#ifndef IR_REMOTE_CONTROL
#define IR_REMOTE_CONTROL


/**
 * \file ir_remote_control.h
 * \brief TV IR remote control module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

The code of this module is based on the TV IR remote control of the e-puck library (www.e-puck.org) and 
is adapted to work with Atmel microprocessor. A major difference is that it does work without "Agenda"; 
it uses directly the Timer2 for timing the reading of the signal.

*/


#include <avr\io.h>
#include <avr\interrupt.h>
#include "variables.h"
#include "leds.h"


/**
 * \brief Initialize the IR receiver port pin in order to generate an interrupt (external interrupt pin PCINT15).
 * \return none
 */
void init_ir_remote_control(void);

/**
 * \brief Return the last check bit.
 * \return check check bit of the signal
 */
unsigned char ir_remote_get_check(void);

/**
 * \brief Return the address of the last command.
 * \return address address part of the signal
 */
unsigned char ir_remote_get_address(void);

/**
 * \brief Return the data of the last command.
 * \return data_ir data part of the signal
 */
unsigned char ir_remote_get_data(void);

#endif
