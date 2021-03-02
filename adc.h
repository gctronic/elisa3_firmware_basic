
#ifndef ADC_H
#define ADC_H

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
 All the proximity and ground sensors are updated at 80 Hz (both active and passive phase).

*/


#include "variables.h"
#include <avr\io.h>
#include <avr\interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure the adc registers and start the sampling.
 * \return none
 */
void initAdc();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
