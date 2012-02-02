
#include "accelerometer.h"

void initAccelerometer() {

	unsigned char ret;
  
	// init I2C bus
	i2c_init();

	ret = initMMA7455L();

	if(ret) {	// MMA7455L doesn't respond, try with ADXL345
		accelAddress = ADXL345_ADDR;
		ret = initADXL345();
		if(ret) {
			useAccel = USE_NO_ACCEL;
		} else {
			useAccel = USE_ADXL345;
		}
	}

}

unsigned char initMMA7455L() {

	unsigned char ret = 0;

	// configure device
	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if (ret) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x16);	// power register
        i2c_write(0x45);	// measurement mode; 2g; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

	return 0;

}

unsigned char initADXL345() {
	
	unsigned char ret = 0;

	// configure device
	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if (ret) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x2D);	// power register
        i2c_write(0x08);	// measurement mode; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if (ret) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x31);	// Data format register
        i2c_write(0x00);	// set to 10-bits resolution; 2g sensitivity; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if (ret) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		return 1;
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x2C);	// Data format register
        i2c_write(0x09);	// set to full resolution; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

	return 0;

}

void readAccelXY() {

	int i = 0;
	signed char buff[4];


	if(useAccel == USE_MMAX7455L) {

		//i2c_start_wait(accelAddress+I2C_WRITE);		// set device address and write mode
		i2c_start(accelAddress+I2C_WRITE);	
		i2c_write(0x00);							// sends address to read from
		i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode

		for(i=0; i<3; i++) {
			buff[i] = i2c_readAck();				// read one byte
		}
		buff[i] = i2c_readNak();					// read last byte
		i2c_stop();									// set stop conditon = release bus

		if(startCalibration) {
			accX = ((signed int)buff[1]<<8)|buff[0];    // X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    // Y axis
		} else {
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;    // X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;    // Y axis
		}

/*
		if(accX & 0x02000) {	// test 10th bit
			accX |= 0xFC00;		// fill with ones (negative number in 2's complement)
		}
		if(accY & 0x02000) {
			accY |= 0xFC00;
		}
*/

	} else if(useAccel == USE_ADXL345) {

		//i2c_start_wait(accelAddress+I2C_WRITE);		// set device address and write mode
		i2c_start(accelAddress+I2C_WRITE);	
		i2c_write(0x32);							// sends address to read from
		i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode

		for(i=0; i<3; i++) {
			buff[i] = i2c_readAck();				// read one byte
		}
		buff[i] = i2c_readNak();					// read last byte
		i2c_stop();									// set stop conditon = release bus

		if(startCalibration) {
			accX = ((signed int)buff[1]<<8)|buff[0];    // X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    // Y axis
		} else {
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;    // X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;    // Y axis
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

		//i2c_start_wait(accelAddress+I2C_WRITE);		// set device address and write mode
		i2c_start(accelAddress+I2C_WRITE);	
		i2c_write(0x00);							// sends address to read from
		i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode

		for(i=0; i<5; i++) {
			buff[i] = i2c_readAck();				// read one byte
		}
		buff[i] = i2c_readNak();					// read last byte
		i2c_stop();									// set stop conditon = release bus


		if(startCalibration) {
			// 10 bits valus in 2's complement
			accX = ((signed int)buff[1]<<8)|buff[0];    // X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    // Y axis
			accZ = ((signed int)buff[5]<<8)|buff[4];    // Z axis
		} else {
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;    // X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;    // Y axis
			accZ = (((signed int)buff[5]<<8)|buff[4])-accOffsetZ;    // Z axis
		}

	} else if(useAccel == USE_ADXL345) {	

		//i2c_start_wait(accelAddress+I2C_WRITE);		// set device address and write mode
		i2c_start(accelAddress+I2C_WRITE);	
		i2c_write(0x32);							// sends address to read from
		i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode

		for(i=0; i<5; i++) {
			buff[i] = i2c_readAck();				// read one byte
		}
		buff[i] = i2c_readNak();					// read last byte
		i2c_stop();									// set stop conditon = release bus

		if(startCalibration) {
			// 10 bits valus in 2's complement
			accX = ((signed int)buff[1]<<8)|buff[0];    // X axis
			accY = ((signed int)buff[3]<<8)|buff[2];    // Y axis
			accZ = ((signed int)buff[5]<<8)|buff[4];    // Z axis
		} else {
			accX = (((signed int)buff[1]<<8)|buff[0])-accOffsetX;    // X axis
			accY = (((signed int)buff[3]<<8)|buff[2])-accOffsetY;    // Y axis
			accZ = (((signed int)buff[5]<<8)|buff[4])-accOffsetZ;    // Z axis
		}

	} else {

		accX = 0;
		accY = 0;
		accZ = 0;

	}

}

void computeAngle() {

	unsigned int abs_acc_z=abs(accZ);

	if(abs_acc_z <= NULL_ANGLE_THRESHOLD) { // && abs_acc_y <= NULL_ANGLE_THRESHOLD) {
		curr_position = ORIZZONTAL_POS;
	} else {
		curr_position = VERTICAL_POS;
	}

	if(prev_position == curr_position) {
		times_in_same_pos++;
		if(times_in_same_pos >= SAME_POS_NUM) {
			times_in_same_pos = 0;
			orizzontal_position = curr_position;	// 1 = orizzontal, 0 = vertical
		}
	} else {
		times_in_same_pos = 0;
	}

	prev_position = curr_position;

/*
	if(orizzontal_position == ORIZZONTAL_POS) {
		pwm_red = 0;
		pwm_green = 0;
		pwm_blue = 0;
		updateRedLed(pwm_red);
		updateGreenLed(pwm_green);
		updateBlueLed(pwm_blue);		
	} else {
		pwm_red = 255;
		pwm_green = 255;
		pwm_blue = 255;
		updateRedLed(pwm_red);
		updateGreenLed(pwm_green);
		updateBlueLed(pwm_blue);
	}
*/

	currentAngle = (signed int)(atan2f((float)accX, (float)accY)*RAD_2_DEG);	//180.0/PI;	//x/y

	if(currentAngle < 0) {
		currentAngle = currentAngle + (signed int)360;	// angles from 0 to 360
	}

}


