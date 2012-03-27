
#ifndef SENSORS_H
#define SENSORS_H


/**
 * \file sensors.h
 * \brief Sensors module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 General module that contains functions to interact with the sensors mounted on the 
 robot (proximity, ground, accelerometer).
 The I2C communication is initialized to work with the accelerometer. There are functions
 to work with both the Freescale MMA7455 and the Analog Device ADXL345 accelerometers.

*/


#include <stdlib.h>
#include <math.h>
#include "variables.h"
#include <avr\io.h>
#include "leds.h"
#include "twimaster.h"

#ifdef __cplusplus
extern "C" {
    #include <math.h>
#endif

/**
 * \brief Calibrate all the sensors (proximity, ground and accelerometer). Pay attention for the robot
 * to be positionated in a flat surface and that no objects obstruct the sensors.
 * Peripherals need to be already initialized before calling this function.
 * \return none
 */
void calibrateSensors();

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

// the following two functions split "readAccelXYZ", used with aseba.
void readAccelXYZ_1();
void readAccelXYZ_2();

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

#ifdef __cplusplus
} // extern "C"
#endif

#endif

