
/**
 * \file
 * \brief Accelerometer module
 * \author  Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3
 *
 * The I2C communication is initialized to work with the accelerometer. There are functions
 * to work with both the Freescale MMA7455 and the Analog Device ADXL345 accelerometers.
 */

/*! \fn int open(const char *pathname,int flags)
    \brief Opens a file descriptor.

    \param pathname The name of the descriptor.
    \param flags Opening flags.
*/

/** 
 * \fn void initAccelerometer()
 * \brief Test which device is mounted on the robot and configure it.
 */


#include <stdlib.h>
#include <math.h>
#include "variables.h"
#include "twimaster.h"

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


void initAccelerometer();

/** 
 * \fn unsigned char initADXL345()
 * \brief Configure the ADXL345 accelerometer (2g sensitivity, 10 bits resolution).
 * \return 1 in case of errors, 0 otherwise.
 */
unsigned char initADXL345();

/** 
 * \fn unsigned char initMMA7455L()
 * \brief Configure the MMA74565L accelerometer (2g sensitivity, 10 bits resolution).
 * \return 1 in case of errors, 0 otherwise.
 */
unsigned char initMMA7455L();


void readAccelXY();
void readAccelXYZ();
void computeAngle();

#endif
