
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

/**
 * \file accelerometer.h
 * \brief Accelerometer module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The I2C communication is initialized to work with the accelerometer. There are functions
 to work with both the Freescale MMA7455 and the Analog Device ADXL345 accelerometers.
*/


#include <stdlib.h>
#include <math.h>
#include "variables.h"
#include "twimaster.h"


/**
 * \brief Test which device is mounted on the robot and configure it.
 * \return none
 */
void initAccelerometer();

/**
 * \brief Configure the ADXL345 accelerometer (2g sensitivity, 10 bits resolution).
 * \retval 0 configuration ok
 * \retval 1 communication error
 */
unsigned char initADXL345();

/**
 * \brief Configure the MMA74565L accelerometer (2g sensitivity, 10 bits resolution).
 * \retval 0 configuration ok
 * \retval 1 communication error
 */
unsigned char initMMA7455L();

/**
 * \brief Request the X and Y values (10 bit data, 2's complement) and save them in their
 * respective global variables accX and accY.
 * \return none
 */
void readAccelXY();

/**
 * \brief Request the X, Y and Z values (10 bit data, 2's complement) and save them in their
 * respective global variables accX, accY and accZ.
 * \return none
 */
void readAccelXYZ();

/**
 * \brief Compute the angle of the robot using the X and Y axes; the resulting angle is saved in the 
 * global variable "currentAngle". The angle refers to a classical reference system where the 0 points 
 * to the right, 90 to the top, 180 to the left and 270 to the bottom in a vertical wall. In order for
 * the angle to be computed correctly, the robot has to be calibrated leaving it flat on the ground.
 * Moreover this function update the robot motion plane (horizontal or vertical) based on the Z axis; this
 * information is then used to switch between horizontal and vertical speed controller.
 * \return none
 */
void computeAngle();

#endif
