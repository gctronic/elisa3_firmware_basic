
#ifndef PORTS_IO_H
#define PORTS_IO_H

/**
 * \file ports_io.h
 * \brief Ports io module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The module is responsible for configuring the direction and state of all the 
 microprocessor pins. 

*/


#include "variables.h"
#include <avr\io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Set pins direction and their initial state based on hardware revision. 
 * Unused pins are set to output in low state. Pull-up feature is disabled.
 * \return none
 */
void initPortsIO();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
