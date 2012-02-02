#include "variables.h"
#include <avr\io.h>
#include <avr\interrupt.h>
#include <avr\sleep.h>
#include "ports_io.h"
#include "adc.h"
#include "motors.h"
#include "leds.h"
#include "spi.h"
#include "mirf.h"
#include "usart.h"
#include "accelerometer.h"
#include "e_remote_control.h"


#ifndef UTILITY_H
#define UTILITY_H

unsigned char getSelector();
void initPeripherals();
/** \fn int open(const char *pathname,int flags)
 * \brief Opens a file descriptor.
 * \param pathname The name of the descriptor.
 * \param flags Opening flags.
*/
void sleep(unsigned char seconds);

#endif
