

#include <avr\io.h>
#include <avr\interrupt.h>
#include <math.h>
#include "constants.h"
#include "ports_io.h"
#include "spi.h"
#include "mirf.h"
#include "nRF24L01.h"
#include "e_agenda.h"
#include "e_remote_control.h"
#include "speed_control.h"
#include "usart.h"
#include "twimaster.h"

// adc
extern volatile unsigned char currentAdChannel;
extern unsigned char currentProx;
extern unsigned char currentMotLeftChannel;
extern unsigned char currentMotRightChannel;
extern unsigned char rightMotorPhase;
extern unsigned char leftMotorPhase;
extern volatile unsigned int proximityValue[24];
extern unsigned char adcSaveDataTo;
extern unsigned char adcSamplingState;
extern unsigned char rightChannelPhase;
extern unsigned char leftChannelPhase;

// consumption controller
extern unsigned int left_current_avg;
extern unsigned int right_current_avg;
extern unsigned int last_left_current;
extern unsigned int last_right_current;

// speed controller
extern unsigned int num_lvel_samples_avg;
extern volatile unsigned int last_num_lvel_samples_avg;
extern unsigned int num_rvel_samples_avg;
extern volatile unsigned int last_num_rvel_samples_avg;
extern unsigned int left_vel_sum;
extern volatile unsigned int last_left_vel_sum;
extern unsigned int right_vel_sum;
extern volatile unsigned int last_right_vel_sum;
extern signed long int pwm_right;
extern signed long int pwm_left;
extern unsigned char compute_left_vel;
extern unsigned char compute_right_vel;
extern unsigned char start_control;
extern signed long int pwm_right_desired;
extern signed long int pwm_left_desired;
extern unsigned char left_vel_changed;
extern unsigned char right_vel_changed;
extern unsigned int last_left_vel;
extern unsigned int last_right_vel;
extern signed long int pwm_right_working;
extern signed long int pwm_left_working;
extern unsigned char update_pwm;

// uart
extern unsigned char peripheralChoice;
extern unsigned char choosePeripheral;
extern unsigned char sendAdcValues;

// rgb leds
extern unsigned char pwm_red;
extern unsigned char pwm_green;
extern unsigned char pwm_blue;
extern unsigned char blinkState;

// nrf
extern unsigned int dataLED[3];
extern unsigned char speedl;
extern unsigned char speedr;
extern unsigned char rfData[PAYLOAD_SIZE];
extern unsigned char ackPayload[16];

// various
extern unsigned char myTimeout;

// ir remote control
extern unsigned char ir_move;
extern unsigned char command_received;
extern unsigned char colorState;

// accelerometer
extern int accelAddress;
extern int accX;
extern int accY;
extern int accZ;
extern unsigned int absAccX, absAccY, absAccZ;
extern unsigned int accOffsetX;							// values obtained during the calibration process; acc = raw_acc - offset
extern unsigned int accOffsetY;							// before calibration: values between -3g and +3g corresponds to values between 0 and 1024
extern unsigned int accOffsetZ;							// after calibration: values between -3g and +3g corresponds to values between -512 and 512
extern signed int currentAngle;							// current orientation of the robot extracted from both the x and y axes


/*
FUSES = {
	.low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),	// internal 8MHz clock, no divisor
	.high = (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_SPIEN),				// jtag disabled
	.extended = EFUSE_DEFAULT,
};
*/

void usartTransmit(unsigned char data);

// Data shared between the ISR and main program must be both volatile 
// and global in scope to avoid compiler optimizations that could lead to
// strange behaviors


// When using integer global variables (or >= 2 bytes variables) accessed by ISR and main
// use "atomic fetching" in order to avoid corruption of the variable:
//
//      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
//      {
//         MyValue_Local = MyValue_Global;
//      } 




void initAdc(void) {

	// ADCSRA -----> ADEN	ADSC	 ADATE	ADIF	ADIE	 ADPS2	ADPS1	ADPS0
	//				 0		0		 0		0		0		 0		0		0
	// ADMUX  -----> REFS1	REFS0	 ADLAR	MUX4	MUX3 	 MUX2	MUX1	MUX0
	//				 0		0		 0		0		0		 0		0		0
	// ADCSRB -----> -		ACME	 - 		- 		MUX5 	 ADTS2 	ADTS1 	ADTS0
	//				 0		0		 0		0		0		 0		0		0

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	// 1/64 prescaler => 8 MHz / 64 = 125 KHz => Tad;
											// one sample need 13.5 Tad in free running mode, so interrupt frequency is 125/13.5= 9.26 KHz (107 us)
	ADMUX |= (1 << REFS0); 	// voltage reference to AVCC (external)
	ADCSRA |= (1 << ADATE); // auto-trigger mode
	//ADRTS2:0 in ADCSRB  are already set to free running by default (0b000)
	ADCSRB &= 0xF8;
	ADCSRA |= (1 << ADIE);	// enable interrupt on conversion completion
	ADCSRA |= (1 << ADEN);	// enable ADC
	ADCSRA |= (1 << ADSC);	// start first conversion (start from channel 0)

}

// ISR_NOBLOCK enable the global interrupt flag when entering the interrupt service routine;
// this let us have nested interrupt, otherwise the interrupt generated during executing this ISR
// will not fire their corresponding ISR
//ISR(ADC_vect, ISR_NOBLOCK) {
ISR(ADC_vect) {	
	// ADIF is cleared by hardware when executing the corresponding interrupt handling vector
		
//	PORTB &= ~(1 << 7);

//	PORTA = 0x00;	// always turn off the pulses
//	PORTJ &= 0xF0;

	int value = ADCL;			// must be read first!!
	value = (ADCH<<8) | value;

	// save the last data

	switch(adcSaveDataTo) {

		case SAVE_TO_PROX:
			proximityValue[currentProx] = value;
			//currentProx = (currentProx+1)%24;
			currentProx++;
			if(currentProx > 23) {
				currentProx = 0;
			}
			PORTA = 0x00;	// always turn off the pulses
			PORTJ &= 0xF0;
			break;

		case SAVE_TO_RIGHT_MOTOR_CURRENT:
			right_current_avg += value;
			right_current_avg = right_current_avg >> 1;
			break;

		case SAVE_TO_RIGHT_MOTOR_VEL:
			right_vel_sum += value;
			num_rvel_samples_avg++;
			break;

		case SAVE_TO_LEFT_MOTOR_CURRENT:
			left_current_avg += value;
			left_current_avg = left_current_avg >> 1;
			break;

		case SAVE_TO_LEFT_MOTOR_VEL:
			left_vel_sum += value;
			num_lvel_samples_avg++;
			break;

	}

	//PORTA = 0x00;	// always turn off the pulses
	//PORTJ &= 0xF0;

	// complete sequence
	// select next channel
	// ...the sequence has to be changed in order to satisfy motors sampling requirements!
	switch(adcSamplingState) {

		case 0:
			currentAdChannel = currentProx>>1;
			//if((currentProx%2)==1) {	// active phase
/*
			if(currentProx & 0x01) {
				if(currentProx < 16) {
					//PORTA = 0x00;	// already done at the ISR beginning...
					//PORTA = (1 << (currentProx>>1));
					PORTA = (1 << currentAdChannel);
				} else {
					PORTJ = (1 << ((currentProx-16)>>1));
				}
			}
*/
			if(rightChannelPhase == ACTIVE_PHASE) {			// the first this isn't really correct
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_CURRENT;
			} else {
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_VEL;
			}
			adcSamplingState = 1;
			break;

		case 1:
			currentAdChannel = currentMotLeftChannel;
			leftChannelPhase = leftMotorPhase;
			adcSaveDataTo = SAVE_TO_PROX;
			adcSamplingState = 2;
			break;

		case 2:
			currentAdChannel = currentMotRightChannel;
			rightChannelPhase = rightMotorPhase;
			if(leftChannelPhase == ACTIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_CURRENT;
			} else {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_VEL;
			}
			adcSamplingState = 3;
			break;

		case 3:
			currentAdChannel = currentMotLeftChannel;
			leftChannelPhase = leftMotorPhase;
			if(rightChannelPhase == ACTIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_CURRENT;
			} else {
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_VEL;
			}
			adcSamplingState = 4;
			break;

		case 4:
			currentAdChannel = currentMotRightChannel;	
			rightChannelPhase = rightMotorPhase;	
			if(leftChannelPhase == ACTIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_CURRENT;
			} else {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_VEL;
			}
			adcSamplingState = 0;
			
			if(currentProx & 0x01) {
				if(currentProx < 16) {
					//PORTA = 0x00;	// already done at the ISR beginning...
					PORTA = (1 << (currentProx>>1));
					//PORTA = (1 << currentAdChannel);
				} else {
					PORTJ = (1 << ((currentProx-16)>>1));
				}
			}

			break;

	}

	// channel selection: continuously change the channel sampled in sequence
	if(currentAdChannel < 8) {
		ADCSRB &= ~(1 << MUX5);
		ADMUX = 0x40 + currentAdChannel;
	} else {
		ADCSRB |= (1 << MUX5);
		ADMUX = 0x40 + (currentAdChannel-8);
	}


//	PORTB |= (1 << 7);

}


void initPwm() {

/*
	// LEDs timer1/pwm
	// Timer1 clock input = Fosc = 8 MHz
	// Period freq = Fosc/TOP (max timer value) => TOP = Fosc/period freq
	// We need a frequency of about 30 KHz => 8000000/30000 = 266
	// The waveform generation mode let us chose the TOP value to be 256
	// thus we get period freq = 8000000/256 = 31250 Hz
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1) | (1 << WGM10); 	// enable OCA, OCB, OCC; clear on match, set at bottom
	TCCR1B |= (1 << WGM12) | (1 << CS10);		// mode 5 => fast-pwm 8 bit; no prescaler
	// the values for the leds pwm goes from 0 (max power on) to 255 (off)
	OCR1A = pwm_red;
	OCR1B = pwm_green;
	OCR1C = pwm_blue;
	//DDRB &= ~(1 << 7) & ~(1 << 6) & ~(1 << 5);	// leds as input to turn them off
	//TCCR1A &= ~(1 << COM1A1) & ~(1 << COM1B1) & ~(1 << COM1C1);	// disable OCA, OCB, OCC to turn them off
	//TIMSK1 |= (1 << OCIE1A); 	// Enable output compare match interrupt
	//TIMSK1 |= (1 << TOIE1);	// Enable timer overflow interrupt
*/

	// Motor right timer3/pwm
	// Timers clock input = Fosc = 8 MHz
	// Period freq = Fosc/TOP (max timer value) => TOP = Fosc/period freq
	// We need a period time of 10 ms (100 Hz)
	// Using 10-bit resolution (waveform generation mode 7) we have a period of: 8000000/1024 = 7812.5 Hz
	// We need to apply a prescaler to the timer in such a way to get the desired period:
	// 7812.5/100 = 78.125 => ideal prescaler, the nearest one is 1/64 and we get a period of:
	// 8000000/64/1024 = 122 Hz
	TCCR3A |= (1 << COM3A1) | (1 << WGM31) | (1 << WGM30); 	// enable OCA; clear on match, set at bottom
	TCCR3A |= (1 << WGM31) | (1 << WGM30);	
	TCCR3B |= (1 << WGM32) | (1 << CS31) | (1 << CS30);		// mode 7 => fast-pwm 10 bit; clock prescaler 1/64
	// the values for motors goes from 0 (stopped) to 1023 (max power)
	OCR3A = pwm_right;
	OCR3B = 0;
	TIMSK3 |= (1 << TOIE3);		// Enable timer overflow interrupt	

	// stop right motor
	TCCR3A  &= ~(1 << COM3A1) & ~(1 << COM3B1);	// disable OCA and OCB
	PORTE &= ~(1 << 4) & ~(1 << 3);				// output to 0

	// Motor left timer4/pwm
	// same configuration as timer3
	TCCR4A |= (1 << COM4A1) | (1 << WGM41) | (1 << WGM40); 	// enable OCA; clear on match, set at bottom
	TCCR4B |= (1 << WGM42) | (1 << CS41) | (1 << CS40);		// mode 7 => fast-pwm 10 bit; clock prescaler 1/64
	// the values for motors goes from 0 (stopped) to 1024 (max power)
	OCR4A = pwm_left;
	OCR4B = 0;
	TIMSK4 |= (1 << TOIE4);		// Enable timer overflow interrupt
	// stop left motor
	TCCR4A  &= ~(1 << COM4A1) & ~(1 << COM4B1);	// disable OCA and OCB
	PORTH &= ~(1 << 4) & ~(1 << 3);				// output to 0


}

// Motor left
ISR(TIMER4_OVF_vect) {

	leftMotorPhase = ACTIVE_PHASE;

	// copy sampling variables
	last_left_current = left_current_avg;
	last_num_lvel_samples_avg = num_lvel_samples_avg;
	last_left_vel_sum = left_vel_sum;

	// reset sampling variables
	left_current_avg = 0;
	num_lvel_samples_avg = 0;
	left_vel_sum = 0;

	// start control
	compute_left_vel = 1;
	start_control = 1;

	// PORTB ^= (1 << 7); // Toggle the LED

	if(pwm_left == 0) {
		// select channel 15 to sample left current
		currentMotLeftChannel = 15;
		TCCR4A  &= ~(1 << COM4A1) & ~(1 << COM4B1);	// disable OCA and OCB
		PORTH &= ~(1 << 4) & ~(1 << 3);				// output to 0
		TIMSK4 &= ~(1 << OCIE4B) & ~(1 << OCIE4A);	// disable OCA and OCB interrupt
		TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt => sampling of velocity is enabled even if 
										// the pwm is turned off...is it correct??
	} else if(pwm_left > 0) {   		// move forward
		// select channel 15 to sample left current
		currentMotLeftChannel = 15;
		TCCR4A  &= ~(1 << COM4B1);		// disable OCB
		TIMSK4 &= ~(1 << OCIE4B);		// disable OCB interrupt
		PORTH &= ~(1 << 4);				// output to 0
		TCCR4A |= (1 << COM4A1);		// enable OCA
		TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt
	} else if(pwm_left < 0) {      		// move backward
		// select channel 14 to sample left current
		currentMotLeftChannel = 14;
		TCCR4A  &= ~(1 << COM4A1);		// disable OCA
		TIMSK4 &= ~(1 << OCIE4A);		// disable OCA interrupt
		PORTH &= ~(1 << 3);				// output to 0
		TCCR4A |= (1 << COM4B1);		// enable OCB
		TIMSK4 |= (1 << OCIE4B);		// enable OCB interrupt
	}

}

// motor left forward
ISR(TIMER4_COMPA_vect) {
	leftMotorPhase = PASSIVE_PHASE;
	// select channel 14 to sample the left velocity
	currentMotLeftChannel = 14;
}

// motor left backward
ISR(TIMER4_COMPB_vect) {
	leftMotorPhase = PASSIVE_PHASE;
	// select channel 15 to sample the left velocity
	currentMotLeftChannel = 15;
}

// Motor right
ISR(TIMER3_OVF_vect) {

	rightMotorPhase = ACTIVE_PHASE;
	sendAdcValues = 1;

	// copy sampling variables
	last_right_current = right_current_avg;
	last_right_vel_sum = right_vel_sum;
	last_num_rvel_samples_avg = num_rvel_samples_avg;

	// reset sampling variables
	right_current_avg = 0;
	right_vel_sum = 0;
	num_rvel_samples_avg = 0;

	// start control
	compute_right_vel = 1;

  	// PORTB ^= (1 << 7); // Toggle the LED

	if(pwm_right == 0) {
		// select channel 13 to sample left current
		currentMotRightChannel = 13;
		TCCR3A  &= ~(1 << COM3A1) & ~(1 << COM3B1);	// disable OCA and OCB
		PORTE &= ~(1 << 4) & ~(1 << 3);				// output to 0
		TIMSK3 &= ~(1 << OCIE3B) & ~(1 << OCIE3A);	// disable OCA and OCB interrupt
		TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt => sampling of velocity is enabled even if 
										// the pwm is turned off...is it correct??
	}else if(pwm_right > 0) {   		// move forward
		// select channel 13 to sample left current
		currentMotRightChannel = 13;
		TCCR3A  &= ~(1 << COM3B1);		// disable OCB
		TIMSK3 &= ~(1 << OCIE3B);		// disable OCB interrupt
		PORTE &= ~(1 << 4);				// output to 0
		TCCR3A |= (1 << COM3A1);		// enable OCA
		TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt
	} else if(pwm_right < 0) {      	// move backward
		// select channel 12 to sample left current
		currentMotRightChannel = 12;
		TCCR3A  &= ~(1 << COM3A1);		// disable OCA
		TIMSK3 &= ~(1 << OCIE3A);		// disable OCA interrupt
		PORTE &= ~(1 << 3);				// output to 0
		TCCR3A |= (1 << COM3B1);		// enable OCB
		TIMSK3 |= (1 << OCIE3B);		// enable OCB interrupt
	}

}

// motor right forward
ISR(TIMER3_COMPA_vect) {

//	PORTB &= ~(1 << 5);

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 12 to sample the right velocity
	currentMotRightChannel = 12;

//	PORTB |= (1 << 5);
}

// motor right backward
ISR(TIMER3_COMPB_vect) {

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 13 to sample the right velocity
	currentMotRightChannel = 13;
}

/*
// LED RED
ISR(TIMER1_COMPA_vect) {

	// TIFR.OCF1A is cleared by hardware when executing the corresponding interrupt handling vector

   PORTB ^= (1 << 0); // Toggle the LED

}

// LED GREEN
ISR(TIMER1_COMPB_vect) {



}

// LED BLUE
ISR(TIMER1_COMPC_vect) {



}
*/

void readAccelXYZ() {

	int i = 0;
	unsigned char buff[6], ret;


#ifdef ACC_MMA7455L

#endif


#ifdef ACC_ADXL345

	//i2c_start_wait(accelAddress+I2C_WRITE);		// set device address and write mode
	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x32);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode

	for(i=0; i<5; i++) {
		buff[i] = i2c_readAck();				// read one byte
	}
	buff[i] = i2c_readNak();					// read last byte
	i2c_stop();									// set stop conditon = release bus

	accX = (((int)buff[1]) << 8) | buff[0];    // Y axis
	accY = (((int)buff[3]) << 8) | buff[2];    // Y axis
	accZ = (((int)buff[5]) << 8) | buff[4];    // Z axis


/*
	accX = i2c_readNak();
	i2c_stop();


	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x01);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode
	accX = accX + i2c_readNak()*256;
	i2c_stop();

	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x02);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode
	accY = i2c_readNak();
	i2c_stop();

	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x03);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode
	accY = accY + i2c_readNak()*256;
	i2c_stop();

	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x04);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode
	accZ = i2c_readNak();
	i2c_stop();

	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x05);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode
	accZ = accZ + i2c_readNak()*256;
	i2c_stop();
*/

#endif

}

void readAccelXY() {

	int i = 0;
	unsigned char buff[4], ret;


#ifdef ACC_MMA7455L

#endif


#ifdef ACC_ADXL345

	//i2c_start_wait(accelAddress+I2C_WRITE);		// set device address and write mode
	i2c_start(accelAddress+I2C_WRITE);	
	i2c_write(0x32);							// sends address to read from
	i2c_rep_start(accelAddress+I2C_READ);		// set device address and read mode

	for(i=0; i<3; i++) {
		buff[i] = i2c_readAck();				// read one byte
	}
	buff[i] = i2c_readNak();					// read last byte
	i2c_stop();									// set stop conditon = release bus

	accX = (((int)buff[1]) << 8) | buff[0];    // X axis
	accY = (((int)buff[3]) << 8) | buff[2];    // Y axis

#endif

}

void initI2C() {

	unsigned char ret;
  
	// init I2C bus
	i2c_init();

#ifdef ACC_MMA7455L

	// configure device
	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if ( ret ) {			// failed to issue start condition, possibly no device found
        i2c_stop();
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x16);	// power register
        i2c_write(0x01);	// measurement mode; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

#endif

#ifdef ACC_ADXL345

	// configure device
	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if ( ret ) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		PORTB &= ~(1 << 5);
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x2D);	// power register
        i2c_write(0x08);	// measurement mode; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if ( ret ) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		PORTB &= ~(1 << 5);
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x31);	// Data format register
        i2c_write(0x08);	// set to full resolution; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }

	ret = i2c_start(accelAddress+I2C_WRITE);       // set device address and write mode
    if ( ret ) {			// failed to issue start condition, possibly no device found
        i2c_stop();
		PORTB &= ~(1 << 5);
    }else {					// issuing start condition ok, device accessible
        i2c_write(0x2C);	// Data format register
        i2c_write(0x09);	// set to full resolution; ret=0 -> Ok, ret=1 -> no ACK 
        i2c_stop();			// set stop conditon = release bus
    }
	
#endif

}

void calibrateAccelerometer() {

	int j=0;
	accOffsetX = 0;
	accOffsetY = 0;
	accOffsetZ = 0;

	for(j=0; j<50; j++) {
		readAccelXYZ();
		accOffsetX += accX;
		accOffsetY += accY;
		accOffsetZ += accZ;
	}
	accOffsetX = accOffsetX/50;
	accOffsetY = accOffsetY/50;
	accOffsetZ = accOffsetZ/50;

}

void computeAngle() {

	readAccelXY();

	accX = accX-accOffsetX;
	accY = accY-accOffsetY;
	//accZ = accZ-accOffsetZ;

/*
	if(accX < 0) {
		absAccX = -accX;
	} else {
		absAccX = accX;
	}
	if(accY < 0) {
		absAccY = -accY;
	} else {
		absAccY = accY;
	}
	if(accZ < 0) {
		absAccZ = -accZ;
	} else {
		absAccZ = accZ;
	}

	if(abs_acc_z <= NULL_ANGLE_THRESHOLD && abs_acc_y <= NULL_ANGLE_THRESHOLD) {
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
*/

	currentAngle = (signed int)(atan2f((float)accX, (float)accY)*RAD_2_DEG);	//180.0/PI;	//x/y

	//current_angle = current_angle*RAD_2_DEG;
	if(currentAngle<0) {
		currentAngle = 360+currentAngle;	// angles from 0 to 360
	}

}

void initPeripherals(void) {

	cli();			// disable global interrupts (by default it should already be disabled)

	initPortsIO();
	initAdc();
	initPwm();
	initSPI();
	mirf_init();
	initUsart();
	initI2C();

	sei();			// enable global interrupts

	

}

void toggleBlueLed() {

	blinkState = 1 - blinkState;

	if(blinkState) {
		TCCR1A |= (1 << COM1C1);	// enable OCC
		OCR1C = 255;
	} else {
		TCCR1A &= ~(1 << COM1C1);
		PORTB &= ~(1 << 7);
	}

}

void updateRedLed(unsigned char value) {

	if(value == 0) {
		TCCR1A &= ~(1 << COM1A1);
		PORTB &= ~(1 << 5);
	} else {
		TCCR1A |= (1 << COM1A1);
		OCR1A = value;
	}

}

void updateGreenLed(unsigned char value) {

	if(value == 0) {
		TCCR1A &= ~(1 << COM1B1);
		PORTB &= ~(1 << 6);
	} else {
		TCCR1A |= (1 << COM1B1);
		OCR1B = value;
	}

}

void updateBlueLed(unsigned char value) {

	if(value == 0) {
		TCCR1A &= ~(1 << COM1C1);
		PORTB &= ~(1 << 7);
	} else {
		TCCR1A |= (1 << COM1C1);
		OCR1C = value;
	}

}

void sendValues() {
	myTimeout = 1;
}

unsigned char getSelector() {
   return (SEL0) + 2*(SEL1) + 4*(SEL2) + 8*(SEL3);
}


int main(void) {

	//unsigned char debugData = 0xAA;
	unsigned int i = 0;
	unsigned char packetId = 0;
	choosePeripheral = 1;

	initPeripherals();

PORTB &= ~(1 << 5);
	calibrateAccelerometer();
PORTB |= (1 << 5);

	e_start_agendas_processing();
	//e_activate_agenda(toggleBlueLed, 10000);		// every 1 seconds
	e_activate_agenda(sendValues, 20000);	// every 2 seconds
	e_init_remote_control();

	//usartTransmit(debugData);				

	while(1) {

		//PORTB ^= (1 << 6); // Toggle the green LED

		ir_move = e_get_data();

		if(command_received) {

			command_received = 0;

			//usartTransmit(ir_move);

			switch(ir_move) {

				case 5:	// stop motors
					pwm_right_desired = 0;
					pwm_left_desired = 0;
					break;

				case 2:	// both motors forward
					if(pwm_right_desired > pwm_left_desired) {
						pwm_left_desired = pwm_right_desired;
					} else {
						pwm_right_desired = pwm_left_desired;
					}
					pwm_right_desired += STEP_MOTORS;
					pwm_left_desired += STEP_MOTORS;
	                if (pwm_right_desired > MAX_MOTORS_PWM) pwm_right_desired = MAX_MOTORS_PWM;
    	            if (pwm_left_desired > MAX_MOTORS_PWM) pwm_left_desired = MAX_MOTORS_PWM;
               		break;

				case 8:	// both motors backward
					if(pwm_right_desired < pwm_left) {
						pwm_left_desired  = pwm_right_desired;
					} else {
						pwm_right_desired = pwm_left_desired;
					}
					pwm_right_desired -= STEP_MOTORS;
					pwm_left_desired -= STEP_MOTORS;
	                if (pwm_right_desired < -MAX_MOTORS_PWM) pwm_right_desired = -MAX_MOTORS_PWM;
    	            if (pwm_left_desired < -MAX_MOTORS_PWM) pwm_left_desired = -MAX_MOTORS_PWM;
                  	break;

				case 6:	// both motors right
					pwm_right_desired -= STEP_MOTORS;
					pwm_left_desired += STEP_MOTORS;
                	if (pwm_right_desired<-MAX_MOTORS_PWM) pwm_right_desired=-MAX_MOTORS_PWM;
                	if (pwm_left_desired>MAX_MOTORS_PWM) pwm_left_desired=MAX_MOTORS_PWM;
					break;

				case 4:	// both motors left
					pwm_right_desired += STEP_MOTORS;
					pwm_left_desired -= STEP_MOTORS;
	                if (pwm_right_desired>MAX_MOTORS_PWM) pwm_right_desired=MAX_MOTORS_PWM;
    	            if (pwm_left_desired<-MAX_MOTORS_PWM) pwm_left_desired=-MAX_MOTORS_PWM;
					break;

				case 3:	// left motor forward
					pwm_left_desired += STEP_MOTORS;
                	if (pwm_left_desired>MAX_MOTORS_PWM) pwm_left_desired=MAX_MOTORS_PWM;
					break;

				case 1:	// right motor forward
					pwm_right_desired += STEP_MOTORS;
	                if (pwm_right_desired>MAX_MOTORS_PWM) pwm_right_desired=MAX_MOTORS_PWM;
					break;

				case 9:	// left motor backward
					pwm_left_desired -= STEP_MOTORS;
            	    if (pwm_left_desired<-MAX_MOTORS_PWM) pwm_left_desired=-MAX_MOTORS_PWM;
					break;

				case 7:	// right motor backward
					pwm_right_desired -= STEP_MOTORS;
                	if (pwm_right_desired<-MAX_MOTORS_PWM) pwm_right_desired=-MAX_MOTORS_PWM;
					break;

               	case 0:	// colors
					colorState = (colorState+1)%5;

					if(colorState==0) {		// turn on blue
						pwm_blue = 0;
						pwm_green = MAX_LEDS_PWM;
						pwm_red = MAX_LEDS_PWM;					
					} else if(colorState==1) {	// turn on green
						pwm_blue = MAX_LEDS_PWM;
						pwm_green = 0;
						pwm_red = MAX_LEDS_PWM;
					} else if(colorState==2) {	// turn on red
						pwm_blue = MAX_LEDS_PWM;
						pwm_green = MAX_LEDS_PWM;
						pwm_red = 0;
					} else if(colorState==3) {	// turn on white
						pwm_blue = 0;
						pwm_green = 0;
						pwm_red = 0;
					} else if(colorState==4) {	// turn off
						pwm_blue = MAX_LEDS_PWM;
						pwm_green = MAX_LEDS_PWM;
						pwm_red = MAX_LEDS_PWM;
					}					

					updateRedLed(pwm_red);	
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);

					//LED_IR1 = !LED_IR1;
					//LED_IR2 = !LED_IR2;

                  	break;

               	default:
                 	break;

            }	// switch

		}	// ir command received


		//if(sendAdcValues && myTimeout) {
		//if(sendAdcValues) {
		if(myTimeout) {
			
			sendAdcValues = 0;
			myTimeout = 0;
			
			//last_right_vel_sum = (unsigned int)(last_right_vel_sum/last_num_rvel_samples_avg);

			//PORTB &= ~(1 << 6);
			//last_left_vel_sum = (unsigned int)(last_left_vel_sum/last_num_lvel_samples_avg);
			//PORTB |= (1 << 6);

			usartTransmit(0xAA);
			usartTransmit(0xAA);
			for(i=0; i<24; i++) {
				usartTransmit((unsigned char)(proximityValue[i]&0xFF));
				usartTransmit((unsigned char)(proximityValue[i]>>8));
			}
			//usartTransmit(getselector());
			//usartTransmit(getselector());
			usartTransmit(last_right_current&0xFF);
			usartTransmit(last_right_current>>8);
			usartTransmit(last_left_current&0xFF);
			usartTransmit(last_left_current>>8);
			

			// two possible cases cause the number of samples to be zero:
			// - when the pwm is at its maximum (thus no passive phase)
			// - a missing output compare match interrupt that indicates the start of the passive phase
			if(last_num_rvel_samples_avg != 0) {
				usartTransmit((unsigned char)((last_right_vel_sum/last_num_rvel_samples_avg)&0xFF));
				usartTransmit((unsigned char)((last_right_vel_sum/last_num_rvel_samples_avg)>>8));
			} else {
				usartTransmit((unsigned char)((1023)&0xFF));	// probably we don't use the pwm to its maximum, so
				usartTransmit((unsigned char)((1023)>>8));	// if the number of samples is 0 it means that the 
				//usartTransmit((unsigned char)((0)&0xFF));		// interrupt was missed (...it is really possible?)
				//usartTransmit((unsigned char)((0)>>8));
			}

			//usartTransmit((unsigned char)(last_num_rvel_samples_avg&0xFF));
			//usartTransmit((unsigned char)(last_num_rvel_samples_avg>>8));

			//usartTransmit((unsigned char)(last_num_lvel_samples_avg&0xFF));
			//usartTransmit((unsigned char)(last_num_lvel_samples_avg>>8));
						
			if(last_num_lvel_samples_avg != 0) {
				usartTransmit((unsigned char)((last_left_vel_sum/last_num_lvel_samples_avg)&0xFF));
				usartTransmit((unsigned char)((last_left_vel_sum/last_num_lvel_samples_avg)>>8));
			} else {
				usartTransmit((unsigned char)((1023)&0xFF));
				usartTransmit((unsigned char)((1023)>>8));
				//usartTransmit((unsigned char)((0)&0xFF));
				//usartTransmit((unsigned char)((0)>>8));
			}
			
			readAccelXYZ();
			usartTransmit(accX&0xFF);
			usartTransmit(accX>>8);
			usartTransmit(accY&0xFF);
			usartTransmit(accY>>8);
			usartTransmit(accZ&0xFF);
			usartTransmit(accZ>>8);	
			PORTB &= ~(1 << 6);
			computeAngle();
			PORTB |= (1 << 6);
			usartTransmit(currentAngle&0xFF);
			usartTransmit(currentAngle>>8);												

		}



		//if(!rx_fifo_is_empty()) {
		if(mirf_data_ready()) {

			// clear irq status
			mirf_config_register(STATUS, 0x70);

			mirf_get_data(rfData);
			flush_rx_fifo();

			//usartTransmit(rfData[0]);



#ifdef USE_REDUCED_PACKET

			//if((data[3]&0b00001000)==0b00001000) {	// check the 4th bit to sleep
			// it was noticed that some robots sometimes "think" to receive something and the data read are wrong,
			// this could lead to go to sleep involuntarily; in order to avoid this situation we define that the 
			// sleep message should be completely zero, but the flag bit
			if(rfData[0]==0 && rfData[1]==0 && rfData[2]==0 && (rfData[3]==0b00001000 || rfData[3]==0b00011000) && rfData[4]==0 && rfData[5]==0) {
				//sleep(ALARM_PAUSE_1_MIN);
			}

			speedr = (rfData[4]&0x7F);	// cast the speed to be at most 127, thus the received speed are in the range 0..127 (usually 0..100),
			speedl = (rfData[5]&0x7F);	// the received speed is then shifted by 3 (x8) in order to have a speed more or less
										// in the same range of the measured speed that is 0..800.
										// In order to have greater resolution at lower speed we shift the speed only by 2 (x4),
										// this means that the range is more or less 0..400.


			if((rfData[4]&0x80)==0x80) {			// motor right forward
				pwm_right_desired = speedr<<2;				// scale the speed received (0..100) to be in the range 0..400
			} else {								// backward
				pwm_right_desired = -(speedr<<2);
			}

			if((rfData[5]&0x80)==0x80) {			// motor left forward
				pwm_left_desired = speedl<<2;		
			} else {								// backward
				pwm_left_desired = -(speedl<<2);
			}
			
			if (pwm_right_desired>(MAX_MOTORS_PWM/2)) pwm_right_desired=(MAX_MOTORS_PWM/2);
            if (pwm_left_desired>(MAX_MOTORS_PWM/2)) pwm_left_desired=(MAX_MOTORS_PWM/2);
            if (pwm_right_desired<-(MAX_MOTORS_PWM/2)) pwm_right_desired=-(MAX_MOTORS_PWM/2);
            if (pwm_left_desired<-(MAX_MOTORS_PWM/2)) pwm_left_desired=-(MAX_MOTORS_PWM/2);
		

			for(i=0; i<3; i++) {
				dataLED[i]=rfData[i]&0xFF;
			}
			pwm_red = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[0]&0xFF)/100;
			pwm_blue = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[1]&0xFF)/100;
			pwm_green = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[2]&0xFF)/100;
			updateRedLed(pwm_red);	
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
			

			if(rfData[3]== 1) {			// turn on one IR
				//LED_IR1 = 0;
				//LED_IR2 = 1;
			} else if(rfData[3]==2) {	// turn on two IRs
				//LED_IR1 = 1;
				//LED_IR2 = 0;
			} else if(rfData[3]==3) {	// turn on all three IRs
				//LED_IR1 = 0;
				//LED_IR2 = 0;											
			} else {					// turn off IRs
				//LED_IR1 = 1;
				//LED_IR2 = 1;
			}

			if((rfData[3]&0b00000100)==0b00000100) {	// check the 3rd bit to enable/disable the IR receiving
				//ir_enabled = 1;
			} else {
				//ir_enabled = 0;
			}

			//desired_orientation = current_angle;
			

#else

			

#endif

#ifdef BIDIRECTIONAL
			packetId = (packetId+1)%256;
			//writeAckPayload(&packetId, 1);
			for(i=0; i<12; i++) {
				ackPayload[i] = (proximityValue[(i*2)+1]>>2);
			}
			writeAckPayload(ackPayload, 16);
#endif


		}

		if(getSelector() == 0) {	// no control

			if(start_control) {
				pwm_right_working = pwm_right_desired;	// pwm in the range 0..MAX_PWM_MOTORS
				pwm_left_working = pwm_left_desired;	
				start_control = 0;
				update_pwm = 1;
			}

		} else if(getSelector() == 2) {		// speed control

			if(start_control && left_vel_changed && right_vel_changed) {
				pwm_right_working = pwm_right_desired;
				pwm_left_working = pwm_left_desired;
				start_control = 0;
				left_vel_changed = 0;
				right_vel_changed = 0;
				//angle_changed = 0;
				//if(!orizzontal_position) {
				//	start_vertical_speed_control(&pwm_left_working, &pwm_right_working);
				//} else {
					PORTB &= ~(1 << 5);
					start_orizzontal_speed_control(&pwm_left_working, &pwm_right_working);
					PORTB |= (1 << 5);
				//}
				//start_power_control(&pwm_left_working, &pwm_right_working);		// the values for the new pwm must be current limited by the controller just before update them
				
				//pwm_right_working = pwm_right_desired;
				//pwm_left_working = pwm_left_desired;				
				
				update_pwm = 1;		
			}

		}

		if(compute_left_vel) {
			last_left_vel = (unsigned int)(last_left_vel_sum/last_num_lvel_samples_avg);
			left_vel_changed = 1;
			compute_left_vel = 0;
		}

		if(compute_right_vel) {
			last_right_vel = (unsigned int)(last_right_vel_sum/last_num_rvel_samples_avg);
			right_vel_changed = 1;
			compute_right_vel = 0;
		}

		if(update_pwm) {

			update_pwm = 0;
			pwm_left = pwm_left_working;
			pwm_right = pwm_right_working;

			if(pwm_right >= 0) {
				OCR3A = (int)pwm_right;
			} else {
				OCR3B = (int)(-pwm_right);
			}
			if(pwm_left >= 0) {
				OCR4A = pwm_left;
			} else {
				OCR4B = -pwm_left;
			}

		}

	} // while(1)

}
