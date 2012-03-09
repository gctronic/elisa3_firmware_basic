
#include "sensors.h"


void calibrateSensors() {

	unsigned int i=0;

	pwm_red = 0;
	pwm_green = 0;
	pwm_blue = 0;
	updateRedLed(pwm_red);
	updateGreenLed(pwm_green);
	updateBlueLed(pwm_blue);

	calibrationCycle = 0;
	startCalibration = 1;

	while(startCalibration) {

		readAccelXYZ();

		if(calibrationCycle<=CALIBRATION_CYCLES) {

			if(proxUpdated) {

				proxUpdated = 0;

				if(calibrationCycle==0) {		// reset all variables
					for(i=0; i<12; i++) {
						proximitySum[i] = 0;
						proximityOffset[i] = 0;
					}
					accOffsetXSum = 0;
					accOffsetYSum = 0;
					accOffsetZSum = 0;

					calibrationCycle++;

					continue;					// the first time "proxUpdated" is set, all the proximity values saved in the array 
												// "proximityResult" hasn't the offset reset to 0. so we start the actual calibration
												// the next time
				}

				for (i=0;i<12;i++) {
					proximitySum[i] += proximityResult[i];
				}

				accOffsetXSum += accX;
				accOffsetYSum += accY;
				accOffsetZSum += accZ;

				calibrationCycle++;

			}

			continue;

		} else if(calibrationCycle > CALIBRATION_CYCLES) {

			for(i=0;i<12;i++) {
				proximityOffset[i] = proximitySum[i]>>4;
			}

			for(i=8; i<12; i++) {
				proximityOffset[i] -= 512;
			}

			accOffsetX = accOffsetXSum>>4;
			accOffsetY = accOffsetYSum>>4;
			accOffsetZ = accOffsetZSum>>4;

			startCalibration = 0;

		}

	}

	pwm_red = 255;
	pwm_green = 255;
	pwm_blue = 255;
	updateRedLed(pwm_red);
	updateGreenLed(pwm_green);
	updateBlueLed(pwm_blue);

}

void initAccelerometer() {

	unsigned char ret;

	i2c_init();		// init I2C bus

	ret = initMMA7455L();

	if(ret) {		// MMA7455L doesn't respond, try with ADXL345
		accelAddress = ADXL345_ADDR;
		ret = initADXL345();
		if(ret) {	// accelerometer not available
			useAccel = USE_NO_ACCEL;
		} else {
			useAccel = USE_ADXL345;
		}
	}

}

unsigned char initMMA7455L() {

	unsigned char ret = 0;

	// configure device
	ret = i2c_start(accelAddress+I2C_WRITE);	// set device address and write mode
    if (ret) {				// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x16);	// power register
        i2c_write(0x45);	// measurement mode; 2g
        i2c_stop();			// set stop conditon = release bus
    }

	return 0;				// configuration ok

}

unsigned char initADXL345() {
	
	unsigned char ret = 0;

	// configure device
	ret = i2c_start(accelAddress+I2C_WRITE);	// set device address and write mode
    if (ret) {				// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x2D);	// power control register
        i2c_write(0x08);	// measurement mode
        i2c_stop();			// set stop conditon = release bus
    }

	ret = i2c_start(accelAddress+I2C_WRITE);	// set device address and write mode
    if (ret) {				// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x31);	// Data format register
        i2c_write(0x00);	// set to 10-bits resolution; 2g sensitivity
        i2c_stop();			// set stop conditon = release bus
    }

	ret = i2c_start(accelAddress+I2C_WRITE);	// set device address and write mode
    if (ret) {				// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x2C);	// data rate register
        i2c_write(0x09);	// set 50 Hz output data rate
        i2c_stop();			// set stop conditon = release bus
    }

	return 0;

}

void readAccelXY() {

	int i = 0;
	signed char buff[4];

	if(useAccel == USE_MMAX7455L) {

		// values returned from the accelerometer are signed byte data (2’s complement)
		// reg 0x00: 10 bits output value X LSB
		// reg 0x01: 10 bits output value X MSB 
		// reg 0x02: 10 bits output value Y LSB
		// reg 0x03: 10 bits output value Y MSB
		// reg 0x04: 10 bits output value Z LSB
		// reg 0x05: 10 bits output value Z MSB

		i2c_start(accelAddress+I2C_WRITE);							// set device address and write mode
		i2c_write(0x00);											// sends address to read from (X LSB)
		i2c_rep_start(accelAddress+I2C_READ);						// set device address and read mode

		for(i=0; i<3; i++) {
			buff[i] = i2c_readAck();								// read one byte at a time
		}
		buff[i] = i2c_readNak();									// read last byte sending NACK
		i2c_stop();													// set stop conditon = release bus

		if(startCalibration) {										// if performing the calibration, then return the raw values
			accX = ((signed int)buff[1]<<8)|buff[0];				// X axis
			accY = ((signed int)buff[3]<<8)|buff[2];				// Y axis
		} else {													// else return the calibrated values
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;	// X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;	// Y axis
		}

	} else if(useAccel == USE_ADXL345) {

		// values returned from the accelerometer are signed byte data (2’s complement)
		// reg 0x32: 10 bits output value X LSB
		// reg 0x33: 10 bits output value X MSB 
		// reg 0x34: 10 bits output value Y LSB
		// reg 0x35: 10 bits output value Y MSB
		// reg 0x36: 10 bits output value Z LSB
		// reg 0x37: 10 bits output value Z MSB

		i2c_start(accelAddress+I2C_WRITE);							// set device address and write mode
		i2c_write(0x32);											// sends address to read from (X LSB)
		i2c_rep_start(accelAddress+I2C_READ);						// set device address and read mode

		for(i=0; i<3; i++) {
			buff[i] = i2c_readAck();								// read one byte at a time
		}
		buff[i] = i2c_readNak();									// read last byte sending NACK
		i2c_stop();													// set stop conditon = release bus

		if(startCalibration) {										// if performing the calibration, then return the raw values
			accX = ((signed int)buff[1]<<8)|buff[0];    			// X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    			// Y axis
		} else {													// else return the calibrated values
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;	// X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;	// Y axis
		}

	} else {

		accX = 0;
		accY = 0;

	}

}

void readAccelXYZ() {

	int i = 0;
	signed char buff[6];

	if(useAccel == USE_MMAX7455L) {

		// values returned from the accelerometer are signed byte data (2’s complement)
		// reg 0x00: 10 bits output value X LSB
		// reg 0x01: 10 bits output value X MSB 
		// reg 0x02: 10 bits output value Y LSB
		// reg 0x03: 10 bits output value Y MSB
		// reg 0x04: 10 bits output value Z LSB
		// reg 0x05: 10 bits output value Z MSB

		i2c_start(accelAddress+I2C_WRITE);							// set device address and write mode
		i2c_write(0x00);											// sends address to read from (X LSB)
		i2c_rep_start(accelAddress+I2C_READ);						// set device address and read mode

		for(i=0; i<5; i++) {
			buff[i] = i2c_readAck();								// read one byte at a time
		}
		buff[i] = i2c_readNak();									// read last byte sending NACK
		i2c_stop();													// set stop conditon = release bus

		if(startCalibration) {										// if performing the calibration, then return the raw values
			accX = ((signed int)buff[1]<<8)|buff[0];    			// X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    			// Y axis
			accZ = ((signed int)buff[5]<<8)|buff[4];    			// Z axis
		} else {													// else return the calibrated values
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;	// X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;	// Y axis
			accZ = (((signed int)buff[5]<<8)|buff[4])-accOffsetZ;	// Z axis
		}

	} else if(useAccel == USE_ADXL345) {							

		// values returned from the accelerometer are signed byte data (2’s complement)
		// reg 0x32: 10 bits output value X LSB
		// reg 0x33: 10 bits output value X MSB 
		// reg 0x34: 10 bits output value Y LSB
		// reg 0x35: 10 bits output value Y MSB
		// reg 0x36: 10 bits output value Z LSB
		// reg 0x37: 10 bits output value Z MSB

		i2c_start(accelAddress+I2C_WRITE);							// set device address and write mode	
		i2c_write(0x32);											// sends address to read from (X LSB)
		i2c_rep_start(accelAddress+I2C_READ);						// set device address and read mode

		for(i=0; i<5; i++) {
			buff[i] = i2c_readAck();								// read one byte at a time
		}
		buff[i] = i2c_readNak();									// read last byte sending NACK
		i2c_stop();													// set stop conditon = release bus

		if(startCalibration) {										// if performing the calibration, then return the raw values
			accX = ((signed int)buff[1]<<8)|buff[0];    			// X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    			// Y axis
			accZ = ((signed int)buff[5]<<8)|buff[4];    			// Z axis
		} else {													// else return the calibrated values
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;	// X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;	// Y axis
			accZ = (((signed int)buff[5]<<8)|buff[4])-accOffsetZ;	// Z axis
		}

	} else {

		accX = 0;
		accY = 0;
		accZ = 0;

	}

}

void computeAngle() {

	unsigned int abs_acc_z=abs(accZ);

	// check the robot motion plane (horizontal or vertical) based on the Z axes;
	// this check (threshold) works only if the accelerometer is calibrated
	// leaving the robot flat on the ground
	if(abs_acc_z <= VERTICAL_THRESHOLD) {
		currPosition = HORIZONTAL_POS;
	} else {
		currPosition = VERTICAL_POS;
	}
	if(prevPosition == currPosition) {			
		timesInSamePos++;
		if(timesInSamePos >= SAME_POS_NUM) {	// if the robot maintains its position for a while, then update the robot position;
			timesInSamePos = 0;					// this check avoid to pass from one position to the other too fast when near the threshold
			robotPosition = currPosition;
		}
	} else {
		timesInSamePos = 0;
	}
	prevPosition = currPosition;

	// compute the angle using the X and Y axis
	currentAngle = (signed int)(atan2((float)accX, (float)accY)*RAD_2_DEG);

	if(currentAngle < 0) {
		currentAngle = currentAngle + (signed int)360;	// angles from 0 to 360
	}

}

