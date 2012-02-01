

#include <avr\io.h>
#include <avr\interrupt.h>
#include <avr\sleep.h>
#include <math.h>
#include "constants.h"
#include "ports_io.h"
#include "spi.h"
#include "mirf.h"
#include "nRF24L01.h"
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
extern unsigned int batteryLevel;
extern unsigned char measBattery;
extern signed int currentProxValue;
extern int proximityResult[12];
extern unsigned int proximityOffset[12];
extern unsigned char updateProx;
extern unsigned long proximitySum[12];
extern unsigned char proxUpdated;

// consumption controller
extern unsigned int left_current_avg;
extern unsigned int right_current_avg;
extern unsigned int last_left_current;
extern unsigned int last_right_current;

// speed controller
extern unsigned int left_vel_sum;
extern unsigned int right_vel_sum;
extern signed int pwm_right;
extern signed int pwm_left;
extern unsigned char compute_left_vel;
extern unsigned char compute_right_vel;
extern signed int pwm_right_desired;
extern signed int pwm_left_desired;
extern unsigned char left_vel_changed;
extern unsigned char right_vel_changed;
extern signed int last_left_vel;
extern signed int last_right_vel;
extern signed int pwm_right_working;
extern signed int pwm_left_working;
extern unsigned char update_pwm;
extern unsigned char firstSampleRight;
extern unsigned char firstSampleLeft;

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
extern signed int speedl;
extern signed int speedr;
extern unsigned char rfData[PAYLOAD_SIZE];
extern unsigned char ackPayload[16];
extern unsigned char packetId;

// various
extern unsigned char myTimeout;
extern unsigned int delayCounter;
extern unsigned char currentSelector;
extern unsigned char startCalibration;
extern signed int calibrationCycle;

// ir remote control
extern unsigned char ir_move;
extern unsigned char command_received;
extern unsigned char colorState;
extern unsigned char irEnabled;
extern unsigned char behaviorState;

// accelerometer
extern int accelAddress;
extern signed int accX;
extern signed int accY;
extern signed int accZ;
extern unsigned int absAccX, absAccY, absAccZ;
extern signed int accOffsetX;							// values obtained during the calibration process; acc = raw_acc - offset
extern signed int accOffsetY;							// before calibration: values between -3g and +3g corresponds to values between 0 and 1024
extern signed int accOffsetZ;							// after calibration: values between -3g and +3g corresponds to values between -512 and 512
extern signed int currentAngle;							// current orientation of the robot extracted from both the x and y axes
extern unsigned char useAccel;
extern signed int accOffsetXSum;
extern signed int accOffsetYSum;
extern signed int accOffsetZSum;
extern unsigned char prev_position; 
extern unsigned char curr_position;
extern unsigned char times_in_same_pos;
extern unsigned char orizzontal_position;

// obstacle avoidance
extern unsigned char obstacleAvoidanceEnabled;
extern signed int rightProxSum;
extern signed int leftProxSum;

// cliff avoidance
extern unsigned char cliffAvoidanceEnabled;
extern unsigned int minGroundValue;
extern unsigned int minGround;
extern unsigned char prevRot;

/*
FUSES = {
	.low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),	// internal 8MHz clock, no divisor
	.high = (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_SPIEN),				// jtag disabled
	.extended = EFUSE_DEFAULT,
};
*/

void initAdc();
void initPwm();
void initI2C();


unsigned int myAbs(int i) {
	if(i < 0) {
		return i*(-1);
	} else {
		return -i;
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
	e_init_remote_control();

	sei();			// enable global interrupts

	
}

// used only for wake-up from sleep
ISR(TIMER2_OVF_vect) {

}

void sleep(unsigned char seconds) {

	unsigned int pause = seconds*30;	// the timer2 used to wake-up from sleep is configured to run at 30 Hz

	// disable external interrupt because it uses the timer2 to interpret the tv
	// remote signal and the timer2 must be free in order to be used for wake-up from sleep
	PCICR &= ~(1 << PCIE1);			// disable interrupt from falling edge
	PCMSK1 &= ~(1 << PCINT15);		

	// disable adc
	ADCSRA = 0x00;	// disable interrupt and turn off adc

	// disable motors pwm
	TCCR3A = 0x00;	// turn off timer
	TCCR3B = 0x00;
	TIMSK3 = 0x00;	// disable interrupt
	TCCR4A = 0x00;
	TCCR4B = 0x00;
	TIMSK4 = 0x00;

	// disable leds pwm
	TCCR1A = 0x00;	// turn off timer
	TCCR1B = 0x00;

	// close communication channels
	closeUsart();
	closeSPI();
	i2c_close();

	// set port pins
	initPortsIO();
	//PORTB &= ~(1 << 5) & ~(1 << 6) & ~(1 << 7);
	PORTC &= ~(1 << 7); // sleep pin
	//PORTB &= ~(1 << 4);	// radio CE pin
	PORTD = 0x00;	// I2C and uart pins to 0

	// set timer2 for wake-up: 
	// source clock = 8 MHz
	// prescaler = 1/1024 => 7812.5 Hz
	// max delay = 7812.5 / 256 = about 30 Hz (33 ms)
	TIMSK2 = 0x01; //(1 << TOIE2);
	TCCR2A &= ~(1 << WGM21); 	// mode 0 => normal mode
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);	// 1/1024 prescaler

	// set extendend standby mode and enable it
	//SMCR |= (1 << SM2) | (1 << SM1) | (1 << SM0) | (1 << SE);	// extended standby
	SMCR |= (1 << SM1) | (1 << SE);
	//SMCR |= (1 << SE);	// idle mode

	while(pause > 0) {	
		// enter extended standby mode
		//sleep_cpu();
		__asm__("sleep");
		pause--;
//		PORTB ^= (1 << 6);
	}

	// disable power mode
	//SMCR &= ~(1 << SE);
	SMCR = 0x00;

	// disable timer2 and its timer overflow interrupt
	TCCR2B &= ~(1 << CS22) &~(1 << CS21) &~(1 << CS20);	// disable timer2
	TIMSK2 = 0;					// disable all interrupt for timer2
	TCCR2A |= (1 << WGM21); 	// mode 2 => CTC mode

	pwm_red = 255;
	pwm_green = 255;
	pwm_blue = 255;
	pwm_right = 0;
	pwm_left = 0;
	initPeripherals();

}


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
	// default		 0		0		 0		0		0		 0		0		0
	// ADMUX  -----> REFS1	REFS0	 ADLAR	MUX4	MUX3 	 MUX2	MUX1	MUX0
	// default		 0		0		 0		0		0		 0		0		0
	// ADCSRB -----> -		ACME	 - 		- 		MUX5 	 ADTS2 	ADTS1 	ADTS0
	// default		 0		0		 0		0		0		 0		0		0

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	// 1/64 prescaler => 8 MHz / 64 = 125 KHz => Tad;
											// one sample need 13 Tad in free running mode, so interrupt frequency is 125/13 = 9.6 KHz (104 us)
	ADMUX |= (1 << REFS0); 	// voltage reference to AVCC (external)
	ADCSRA |= (1 << ADATE); // auto-trigger mode
	ADCSRB &= 0xF8;			// for safety...ADRTS2:0 in ADCSRB should be already set to free running by default (0b000)
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

	delayCounter++;		// this variable is used to have basic delays based on the adc interrupt timing (one interrupt every 104 us)

	int value = ADCL;			// must be read first!!
	value = (ADCH<<8) | value;

	// save the last data
	switch(adcSaveDataTo) {

		case SAVE_TO_PROX:
			if(currentProx==14 && measBattery==2) {
				batteryLevel = value;
				measBattery = 0;
				PORTC &= ~(1 << 6);
			} else {
				proximityValue[currentProx] = value;
			}
			currentProx++;
			if(currentProx > 23) {
				currentProx = 0;
				updateProx = 1;
			}
			break;

		case SAVE_TO_RIGHT_MOTOR_CURRENT:
			right_current_avg += value;
			right_current_avg = right_current_avg >> 1;
			break;

		case SAVE_TO_RIGHT_MOTOR_VEL:
			if(firstSampleRight > 0) {
				firstSampleRight++;
				if(firstSampleRight > 4) {		// to skip undesired samples (3 samples skipped)
					right_vel_sum += value;
					if(firstSampleRight==8) {	// number of samples to take for the speed computation
						firstSampleRight = 0;
						compute_right_vel = 1;
					}
				}
			}
			break;

		case SAVE_TO_LEFT_MOTOR_CURRENT:
			left_current_avg += value;
			left_current_avg = left_current_avg >> 1;
			break;

		case SAVE_TO_LEFT_MOTOR_VEL:
			//PORTB ^= (1 << 7);
			if(firstSampleLeft > 0) {
				firstSampleLeft++;
				if(firstSampleLeft > 4) {
					left_vel_sum += value;
					if(firstSampleLeft==8) {
						firstSampleLeft = 0;
						compute_left_vel = 1;
					}
				}
			}
			break;

		case SKIP_SAMPLE:
			break;
	}

	// select next channel
	switch(adcSamplingState) {

		case 0:	// proximity
			// currentProx goes from 0 to 23, currentAdChannel from 0 to 11
			currentAdChannel = currentProx>>1;	// when currentProx is odd it means it is the active phase (pulse on)
			if(rightChannelPhase == ACTIVE_PHASE) {			// the first time this isn't really correct
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_CURRENT;
			} else if(rightChannelPhase == PASSIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_VEL;
			} else {
				adcSaveDataTo = SKIP_SAMPLE;
			}
			adcSamplingState = 1;
			break;

		case 1:	// left motor
			currentAdChannel = currentMotLeftChannel;
			leftChannelPhase = leftMotorPhase;
			adcSaveDataTo = SAVE_TO_PROX;
			adcSamplingState = 2;
			break;

		case 2:	// right motor
			currentAdChannel = currentMotRightChannel;
			rightChannelPhase = rightMotorPhase;
			if(leftChannelPhase == ACTIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_CURRENT;
			} else if(leftChannelPhase == PASSIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_VEL;
			} else {
				adcSaveDataTo = SKIP_SAMPLE;
			}
			adcSamplingState = 3;
			break;

		case 3:	// left motor
			currentAdChannel = currentMotLeftChannel;
			leftChannelPhase = leftMotorPhase;
			if(rightChannelPhase == ACTIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_CURRENT;
			} else if(rightChannelPhase == PASSIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_RIGHT_MOTOR_VEL;
			} else {
				adcSaveDataTo = SKIP_SAMPLE;
			}
			adcSamplingState = 4;
			break;

		case 4:	// right motor
			currentAdChannel = currentMotRightChannel;	
			rightChannelPhase = rightMotorPhase;	
			if(leftChannelPhase == ACTIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_CURRENT;
			} else if(leftChannelPhase == PASSIVE_PHASE) {
				adcSaveDataTo = SAVE_TO_LEFT_MOTOR_VEL;
			} else {
				adcSaveDataTo = SKIP_SAMPLE;
			}
			adcSamplingState = 0;

			if(currentProx==14 && measBattery==1) {
				measBattery=2;
				PORTC |= (1 << 6);	// sense enable on
			}

			if(currentProx & 0x01) {	// if active phase
				if(currentProx < 16) {
					if(currentProx==14 && measBattery==1) {	// channel 7 is shared for both prox7 and battery sampling
						measBattery=2;
						PORTC |= (1 << 6);	// sense enable on
					} else {
						PORTA = (1 << (currentProx>>1));	// pulse on
					}
				} else {
					#ifdef HW_REV_3_0
					PORTJ = (1 << ((currentProx-16)>>1));	// pulse on
					#endif
					
					#ifdef HW_REV_3_0_1
					PORTJ &= ~(1 << ((currentProx-16)>>1));
/*
					if(currentProx==17) {
						PORTJ = 0x0E;
					} else if(currentProx==19) {
						PORTJ &= 0x0D;
					} else if(currentProx==21) {
						PORTJ = 0x0B;
					} else if(currentProx==23) {
						PORTJ = 0x07;
					} 
*/
					#endif

					#ifdef HW_REV_3_1
					PORTJ &= ~(1 << ((currentProx-16)>>1));
					#endif

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

	// turn off the pulses in order to have 200 us of pulse
	if(adcSamplingState == 2) {

		#ifdef HW_REV_3_0
		PORTJ &= 0xF0;
		PORTA = 0x00;
		#endif

		#ifdef HW_REV_3_0_1
		PORTJ = 0xFF;
		PORTA = 0x00;
		#endif

		#ifdef HE_REV_3_1
		PORTJ = 0xFF;
		PORTA = 0x00;
		#endif

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

//	PORTB &= ~(1 << 6);

	if(pwm_left == 0) {
		//firstSampleLeft = 0;
		left_vel_sum = 0;
		last_left_vel = 0;
		left_current_avg = 0;

		//leftMotorPhase = NO_PHASE;
		//compute_left_vel = 1;

		if(prev_pwm_left < 0) {
			leftMotorPhase = PASSIVE_PHASE;
			// select channel 14 to sample the left velocity
			currentMotLeftChannel = 15;
		} else {
			leftMotorPhase = PASSIVE_PHASE;
			// select channel 14 to sample the left velocity
			currentMotLeftChannel = 14;		
		}
		firstSampleLeft = 1;

		// select channel 15 to sample left current
		//currentMotLeftChannel = 15;
		TCCR4A  &= ~(1 << COM4A1) & ~(1 << COM4B1);	// disable OCA and OCB
		PORTH &= ~(1 << 4) & ~(1 << 3);				// output to 0
		TIMSK4 &= ~(1 << OCIE4B) & ~(1 << OCIE4A);	// disable OCA and OCB interrupt
		//TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt => sampling of velocity is enabled even if 
										// the pwm is turned off...is it correct??
		TIFR4 |= (1 << OCF4A) | (1 << OCF4B);
	} else if(pwm_left > 0) {   		// move forward
		leftMotorPhase = ACTIVE_PHASE;
		// select channel 15 to sample left current
		currentMotLeftChannel = 15;
		TCCR4A  &= ~(1 << COM4B1);		// disable OCB
		TIMSK4 &= ~(1 << OCIE4B);		// disable OCB interrupt
		PORTH &= ~(1 << 4);				// output to 0
		TCCR4A |= (1 << COM4A1);		// enable OCA
		TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt
	} else if(pwm_left < 0) {      		// move backward
		leftMotorPhase = ACTIVE_PHASE;
		// select channel 14 to sample left current
		currentMotLeftChannel = 14;
		TCCR4A  &= ~(1 << COM4A1);		// disable OCA
		TIMSK4 &= ~(1 << OCIE4A);		// disable OCA interrupt
		PORTH &= ~(1 << 3);				// output to 0
		TCCR4A |= (1 << COM4B1);		// enable OCB
		TIMSK4 |= (1 << OCIE4B);		// enable OCB interrupt
	}

//	PORTB |= (1 << 6);

}

// motor left forward
ISR(TIMER4_COMPA_vect) {

//	PORTB &= ~(1 << 6);

//	if(pwm_left == 0) {
//		return;
//	}

	leftMotorPhase = PASSIVE_PHASE;
	// select channel 14 to sample the left velocity
	currentMotLeftChannel = 14;

	firstSampleLeft = 1;

//	PORTB |= (1 << 6);

}

// motor left backward
ISR(TIMER4_COMPB_vect) {

//	PORTB &= ~(1 << 6);

//	if(pwm_left == 0) {
//		return;
//	}

	leftMotorPhase = PASSIVE_PHASE;
	// select channel 15 to sample the left velocity
	currentMotLeftChannel = 15;

	firstSampleLeft = 1;

//	PORTB |= (1 << 6);

}

// Motor right
ISR(TIMER3_OVF_vect) {

//	PORTB &= ~(1 << 6);

  	// PORTB ^= (1 << 7); // Toggle the LED

	if(pwm_right == 0) {
		//firstSampleRight = 0;
		right_vel_sum = 0;
		last_right_vel = 0;
		right_current_avg = 0;
		//rightMotorPhase = NO_PHASE;
		//compute_right_vel = 1;

		if(prev_pwm_right < 0) {
			rightMotorPhase = PASSIVE_PHASE;
			// select channel 12 to sample the right velocity
			currentMotRightChannel = 13;
		} else {
			rightMotorPhase = PASSIVE_PHASE;
			// select channel 12 to sample the right velocity
			currentMotRightChannel = 12;
		}
		firstSampleRight = 1;

		// select channel 13 to sample left current
		//currentMotRightChannel = 13;
		TCCR3A  &= ~(1 << COM3A1) & ~(1 << COM3B1);	// disable OCA and OCB
		PORTE &= ~(1 << 4) & ~(1 << 3);				// output to 0
		TIMSK3 &= ~(1 << OCIE3B) & ~(1 << OCIE3A);	// disable OCA and OCB interrupt
		//TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt => sampling of velocity is enabled even if 
										// the pwm is turned off...is it correct??
		TIFR3 |= (1 << OCF3A) | (1 << OCF3B);
	}else if(pwm_right > 0) {   		// move forward
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 13 to sample left current
		currentMotRightChannel = 13;
		TCCR3A  &= ~(1 << COM3B1);		// disable OCB
		TIMSK3 &= ~(1 << OCIE3B);		// disable OCB interrupt
		PORTE &= ~(1 << 4);				// output to 0
		TCCR3A |= (1 << COM3A1);		// enable OCA
		TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt
	} else if(pwm_right < 0) {      	// move backward
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 12 to sample left current
		currentMotRightChannel = 12;
		TCCR3A  &= ~(1 << COM3A1);		// disable OCA
		TIMSK3 &= ~(1 << OCIE3A);		// disable OCA interrupt
		PORTE &= ~(1 << 3);				// output to 0
		TCCR3A |= (1 << COM3B1);		// enable OCB
		TIMSK3 |= (1 << OCIE3B);		// enable OCB interrupt
	}

//	PORTB |= (1 << 6);

}

// motor right forward
ISR(TIMER3_COMPA_vect) {

//	PORTB &= ~(1 << 5);

//	if(pwm_right == 0) {
//		return;
//	}

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 12 to sample the right velocity
	currentMotRightChannel = 12;

	firstSampleRight = 1;

//	PORTB |= (1 << 5);

}

// motor right backward
ISR(TIMER3_COMPB_vect) {

//	PORTB &= ~(1 << 5);

	if(pwm_right == 0) {
		return;
	}

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 13 to sample the right velocity
	currentMotRightChannel = 13;

	firstSampleRight = 1;

//	PORTB |= (1 << 5);
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

void initI2C() {

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


void sendValues() {
	myTimeout = 1;
}

unsigned char getSelector() {
   return (SEL0) + 2*(SEL1) + 4*(SEL2) + 8*(SEL3);
}

void obstacleAvoidance() {
	// obstacle avoidance using the 3 front proximity sensors

	//		forward
	//
	//			0
	//		7		1
	//	velL	x	 velR
	//	  |		|	  |
	//	  6	  y_0	  2
	//
	//
	//		5		3
	//			4
	//
	// The follwoing tables shows the weights (simplified respect to the trigonometry)
	// of all the proximity sensors for the resulting force:
	//
	//		0		1		2		3		4		5		6		7
	//	x	-1		-0.5	0		0.5		1		0.5		0		-0.5
	//	y	0		0.5		1		0.5		0		-0.5	-1		-0.5

	signed int velX=0, velY=0;
	unsigned int sumSensorsX=0, sumSensorsY=0;
	signed int speedL=0, speedR=0;


	if(pwm_right_desired < 0) {
		speedr = -speedr;
	}
	if(pwm_left_desired < 0) {
		speedl = - speedl;
	}

	velX = (speedr + speedl)/2;
	velY = (speedr - speedl)/2;

	sumSensorsX = -proximityResult[0] - proximityResult[1]/2 + proximityResult[3]/2 + proximityResult[4] + proximityResult[5]/2 - proximityResult[7]/2;
	sumSensorsY = proximityResult[1]/2 + proximityResult[2] + proximityResult[3]/2 - proximityResult[5]/2 - proximityResult[6] - proximityResult[7]/2;

	velX += sumSensorsX/4;
	velY += sumSensorsY/4;

	speedR = (velX + velY);
	speedL = (velX - velY);

	if(speedL < 0) {
		speedL = -speedL;
		pwm_left_working = -(speedL<<2);
	} else {
		pwm_left_working = speedL<<2;
	}

	if(speedR < 0) {
		speedR = -speedR;
		pwm_right_working = -(speedR<<2);
	} else {
		pwm_right_working = speedR<<2;
	}


	if (pwm_right_working>(MAX_MOTORS_PWM/2)) pwm_right_working=(MAX_MOTORS_PWM/2);
	if (pwm_left_working>(MAX_MOTORS_PWM/2)) pwm_left_working=(MAX_MOTORS_PWM/2);
	if (pwm_right_working<-(MAX_MOTORS_PWM/2)) pwm_right_working=-(MAX_MOTORS_PWM/2);
	if (pwm_left_working<-(MAX_MOTORS_PWM/2)) pwm_left_working=-(MAX_MOTORS_PWM/2);



/*
	signed int currentProxValue1=0, currentProxValue2=0, speedL=0, speedR=0;

	if(speedr==0 || speedl==0) {
		pwm_right_working = 0;
		pwm_left_working = 0;
		return;
	}

	// max speed tested = 40
	if(speedr > 40) {
		speedr = 40;
	}
	if(speedl > 40) {
		speedl = 40;
	}

	currentProxValue1 = proximityValue[0] - proximityValue[1];	// ambient - (ambient+reflected)
	if(currentProxValue1 < 0) {
		currentProxValue1 = 0;
	}
	currentProxValue2 = proximityValue[2] - proximityValue[3];	// ambient - (ambient+reflected)
	if(currentProxValue2 < 0) {
		currentProxValue2 = 0;
	}
	rightProxSum = currentProxValue1/2 + currentProxValue2;
	currentProxValue2 = proximityValue[14] - proximityValue[15];	// ambient - (ambient+reflected)
	if(currentProxValue2 < 0) {
		currentProxValue2 = 0;
	}
	leftProxSum = currentProxValue1/4 + currentProxValue2;


	rightProxSum = rightProxSum/(int)(300/speedr);	// scale the sum to have a moderate impact on the velocity
	leftProxSum = leftProxSum/(int)(300/speedl);

	if(rightProxSum > (int)(speedr*2)) {		// velocity of the motors goes from -30 to +30
		rightProxSum = (int)(speedr*2);
	}
	if(leftProxSum > (int)(speedl*2)) {
		leftProxSum = (int)(speedl*2);
	}


	speedL = (int)speedl - rightProxSum;
	speedR = (int)speedr - leftProxSum;

	if(speedL < 0) {
		speedL = -speedL;
		pwm_left_working = -(speedL<<2);
	} else {
		pwm_left_working = speedL<<2;
	}

	if(speedR < 0) {
		speedR = -speedR;
		pwm_right_working = -(speedR<<2);
	} else {
		pwm_right_working = speedR<<2;
	}


	if (pwm_right_working>(MAX_MOTORS_PWM/2)) pwm_right_working=(MAX_MOTORS_PWM/2);
	if (pwm_left_working>(MAX_MOTORS_PWM/2)) pwm_left_working=(MAX_MOTORS_PWM/2);
	if (pwm_right_working<-(MAX_MOTORS_PWM/2)) pwm_right_working=-(MAX_MOTORS_PWM/2);
	if (pwm_left_working<-(MAX_MOTORS_PWM/2)) pwm_left_working=-(MAX_MOTORS_PWM/2);
*/

}

void cliffAvoidance() {

	signed int g0=0, g1=0, g2=0, g3=0;

	g0 = proximityValue[16] - proximityValue[17];	// ambient - (ambient+reflected)
	if(g0 < 0) {
		g0 = 0;
	}

	g1 = proximityValue[18] - proximityValue[19];	// ambient - (ambient+reflected)
	if(g1 < 0) {
		g1 = 0;
	}

	g2 = proximityValue[20] - proximityValue[21];	// ambient - (ambient+reflected)
	if(g2 < 0) {
		g2 = 0;
	}

	g3 = proximityValue[22] - proximityValue[23];	// ambient - (ambient+reflected)
	if(g3 < 0) {
		g3 = 0;
	}

	minGroundValue = g0;
	minGround = GROUND_LEFT;
	if(g1 < minGroundValue) {
		minGroundValue = g1;
		minGround = GROUND_CENTER_LEFT;
	}
	if(g2 < minGroundValue) {
		minGroundValue = g2;
		minGround = GROUND_CENTER_RIGHT;
	}
	if(g3 < minGroundValue) {
		minGroundValue = g3;
		minGround = GROUND_RIGHT;
	}

	if(minGroundValue <= CLIFF_THR) {
		pwm_right_working = 0;
		pwm_left_working = 0;
	}

}

int main(void) {

	unsigned int i = 0;
	choosePeripheral = 1;

	initPeripherals();

	startCalibration = 1;
	calibrationCycle = 0;
/*
	pwm_red = 0;
	pwm_green = 0;
	pwm_blue = 0;
	updateRedLed(pwm_red);
	updateGreenLed(pwm_green);
	updateBlueLed(pwm_blue);				
*/				

	while(1) {

		PORTB ^= (1 << 6); // Toggle the green LED

		currentSelector = getSelector();
		//PORTB &= ~(1 << 6);
		readAccelXYZ();
		//PORTB |= (1 << 6);
		computeAngle();

		if(updateProx) {

			updateProx = 0;
			if(startCalibration) {

				proximityResult[0] = proximityValue[0] - proximityValue[1];	// ambient - (ambient+reflected)
				if(proximityResult[0] < 0) {
					proximityResult[0] = 0;
				}

				proximityResult[1] = proximityValue[2] - proximityValue[3];	// ambient - (ambient+reflected)
				if(proximityResult[1] < 0) {
					proximityResult[1] = 0;
				}					

				proximityResult[2] = proximityValue[4] - proximityValue[5];	// ambient - (ambient+reflected)
				if(proximityResult[2] < 0) {
					proximityResult[2] = 0;
				}

				proximityResult[3] = proximityValue[6] - proximityValue[7];	// ambient - (ambient+reflected)
				if(proximityResult[3] < 0) {
					proximityResult[3] = 0;
				}

				proximityResult[4] = proximityValue[8] - proximityValue[9];	// ambient - (ambient+reflected)
				if(proximityResult[4] < 0) {
					proximityResult[4] = 0;
				}

				proximityResult[5] = proximityValue[10] - proximityValue[11];	// ambient - (ambient+reflected)
				if(proximityResult[5] < 0) {
					proximityResult[5] = 0;
				}

				proximityResult[6] = proximityValue[12] - proximityValue[13];	// ambient - (ambient+reflected)
				if(proximityResult[6] < 0) {
					proximityResult[6] = 0;
				}

				proximityResult[7] = proximityValue[14] - proximityValue[15];	// ambient - (ambient+reflected)
				if(proximityResult[7] < 0) {
					proximityResult[7] = 0;
				}	

				proximityResult[8] = proximityValue[16] - proximityValue[17];	// ambient - (ambient+reflected)
				if(proximityResult[8] < 0) {
					proximityResult[8] = 0;
				}

				proximityResult[9] = proximityValue[18] - proximityValue[19];	// ambient - (ambient+reflected)
				if(proximityResult[9] < 0) {
					proximityResult[9] = 0;
				}

				proximityResult[10] = proximityValue[20] - proximityValue[21];	// ambient - (ambient+reflected)
				if(proximityResult[10] < 0) {
					proximityResult[10] = 0;
				}

				proximityResult[11] = proximityValue[22] - proximityValue[23];	// ambient - (ambient+reflected)
				if(proximityResult[11] < 0) {
					proximityResult[11] = 0;
				}

			} else { 

				proximityResult[0] = proximityValue[0] - proximityValue[1] - proximityOffset[0];	// ambient - (ambient+reflected) - offset
				if(proximityResult[0] < 0) {
					proximityResult[0] = 0;
				}

				proximityResult[1] = proximityValue[2] - proximityValue[3] - proximityOffset[1];	// ambient - (ambient+reflected) - offset
				if(proximityResult[1] < 0) {
					proximityResult[1] = 0;
				}					

				proximityResult[2] = proximityValue[4] - proximityValue[5] - proximityOffset[2];	// ambient - (ambient+reflected) - offset
				if(proximityResult[2] < 0) {
					proximityResult[2] = 0;
				}

				proximityResult[3] = proximityValue[6] - proximityValue[7] - proximityOffset[3];	// ambient - (ambient+reflected) - offset
				if(proximityResult[3] < 0) {
					proximityResult[3] = 0;
				}

				proximityResult[4] = proximityValue[8] - proximityValue[9] - proximityOffset[4];	// ambient - (ambient+reflected) - offset
				if(proximityResult[4] < 0) {
					proximityResult[4] = 0;
				}

				proximityResult[5] = proximityValue[10] - proximityValue[11] - proximityOffset[5];	// ambient - (ambient+reflected) - offset
				if(proximityResult[5] < 0) {
					proximityResult[5] = 0;
				}

				proximityResult[6] = proximityValue[12] - proximityValue[13] - proximityOffset[6];	// ambient - (ambient+reflected) - offset
				if(proximityResult[6] < 0) {
					proximityResult[6] = 0;
				}

				proximityResult[7] = proximityValue[14] - proximityValue[15] - proximityOffset[7];	// ambient - (ambient+reflected) - offset
				if(proximityResult[7] < 0) {
					proximityResult[7] = 0;
				}	

				proximityResult[8] = proximityValue[16] - proximityValue[17] - proximityOffset[8];	// ambient - (ambient+reflected) - offset
				if(proximityResult[8] < 0) {
					proximityResult[8] = 0;
				}

				proximityResult[9] = proximityValue[18] - proximityValue[19] - proximityOffset[9];	// ambient - (ambient+reflected) - offset
				if(proximityResult[9] < 0) {
					proximityResult[9] = 0;
				}

				proximityResult[10] = proximityValue[20] - proximityValue[21] - proximityOffset[10];	// ambient - (ambient+reflected) - offset
				if(proximityResult[10] < 0) {
					proximityResult[10] = 0;
				}

				proximityResult[11] = proximityValue[22] - proximityValue[23] - proximityOffset[11];	// ambient - (ambient+reflected) - offset
				if(proximityResult[11] < 0) {
					proximityResult[11] = 0;
				}

			}
			proxUpdated = 1;
		}
/*
		if(delayCounter%78 == 0) {
			start_control = 1;
		}
*/
/*
		if(delayCounter >= 10000) {
			pwm_right_desired = 40;
			pwm_left_desired = 40;
		} else {
			pwm_right_desired = 0;
			pwm_left_desired = 0;
		}
*/
		if(delayCounter >= 20000) {
			measBattery = 1;
			//sleep(60);
			/*
			motorSpeed = 1 - motorSpeed;
			if(motorSpeed) {
				pwm_right_desired = 40;
				pwm_left_desired = 40;
			} else {
				pwm_right_desired = 0;
				pwm_left_desired = 0;
			}
			*/
		}


		if(startCalibration && calibrationCycle<CALIBRATION_CYCLES) {

			if(proxUpdated) {

				proxUpdated = 0;
				
				if(calibrationCycle==0) {
					for(i=0; i<12; i++) {
						proximitySum[i] = 0;
					}
					accOffsetXSum = 0;
					accOffsetYSum = 0;
					accOffsetZSum = 0;
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

		} else if(calibrationCycle == CALIBRATION_CYCLES) {

			for (i=0;i<12;i++) {
				//proximityOffset[i]=(unsigned int)((float)proximitySum[i]/(float)calibrationCycle);
				proximityOffset[i] = proximitySum[i]>>4;
			}

			accOffsetX = accOffsetXSum>>4;				
			accOffsetY = accOffsetYSum>>4;
			accOffsetZ = accOffsetZSum>>4;

			startCalibration = 0;
			calibrationCycle = 0;
/*
			pwm_red = 255;
			pwm_green = 255;
			pwm_blue = 255;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
*/

		}

		if(irEnabled) {

			ir_move = e_get_data();

			if(command_received) {

				command_received = 0;

				switch(ir_move) {

					case 5:	// stop motors
					case 51:
						pwm_right_desired = 0;
						pwm_left_desired = 0;
						break;

					case 2:	// both motors forward
					case 31:
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
					case 30:
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
					case 47:
						pwm_right_desired -= STEP_MOTORS;
						pwm_left_desired += STEP_MOTORS;
	                	if (pwm_right_desired<-MAX_MOTORS_PWM) pwm_right_desired=-MAX_MOTORS_PWM;
	                	if (pwm_left_desired>MAX_MOTORS_PWM) pwm_left_desired=MAX_MOTORS_PWM;
						break;

					case 4:	// both motors left
					case 46:
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
					case 50:
						colorState = (colorState+1)%5;

						if(colorState==0) {		// turn on blue
							LED_IR1_HIGH;
							LED_IR2_HIGH;
							pwm_blue = 0;
							pwm_green = MAX_LEDS_PWM;
							pwm_red = MAX_LEDS_PWM;					
						} else if(colorState==1) {	// turn on green
							pwm_blue = MAX_LEDS_PWM;
							pwm_green = 0;
							pwm_red = MAX_LEDS_PWM;
						} else if(colorState==2) {	// turn on red
							LED_IR1_LOW;
							LED_IR2_LOW;
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

	                  	break;

					case 16:	// volume +
						obstacleAvoidanceEnabled = 1;
						break;
					
					case 17:	// volume -
						obstacleAvoidanceEnabled = 0;
						break;

					case 32:	// program +
						cliffAvoidanceEnabled = 1;
						break;

					case 33:	// program -
						cliffAvoidanceEnabled = 0;
						break;
					
					case 52:
						behaviorState = (behaviorState+1)%4;
						switch(behaviorState) {
							case 0: 
								obstacleAvoidanceEnabled = 0;
								cliffAvoidanceEnabled = 0;
								break;
							case 1:
								obstacleAvoidanceEnabled = 1;
								cliffAvoidanceEnabled = 0;
								break;
							case 2:
								obstacleAvoidanceEnabled = 0;
								cliffAvoidanceEnabled = 1;
								break;
							case 3:
								obstacleAvoidanceEnabled = 1;
								cliffAvoidanceEnabled = 1;
								break;
									
						}
						break;		

	               	default:
	                 	break;

	            }	// switch
				
				if(pwm_right_desired >= 0) {
					speedr = pwm_right_desired >> 2;
				} else {
					speedr = (-pwm_right_desired) >> 2;
				}
				if(pwm_left_desired >= 0) {
					speedl = pwm_left_desired >> 2;
				} else {
					speedl = (-pwm_left_desired) >> 2;
				}

			}	// ir command received

		}	// ir enabled check


		//if(sendAdcValues && myTimeout) {
		//if(sendAdcValues) {
		//if(myTimeout) {
		if(delayCounter >= 20000) {
			delayCounter = 0;
/*
			sendAdcValues = 0;
			myTimeout = 0;
			
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
			usartTransmit((unsigned char)(last_right_vel&0xFF));
			usartTransmit((unsigned char)(last_right_vel>>8));
			usartTransmit((unsigned char)(last_left_vel&0xFF));
			usartTransmit((unsigned char)(last_left_vel>>8));

			
			readAccelXYZ();
			usartTransmit(accX&0xFF);
			usartTransmit(accX>>8);
			usartTransmit(accY&0xFF);
			usartTransmit(accY>>8);
			usartTransmit(accZ&0xFF);
			usartTransmit(accZ>>8);	
			//PORTB &= ~(1 << 6);
			//computeAngle();
			//PORTB |= (1 << 6);
			usartTransmit(currentAngle&0xFF);
			usartTransmit(currentAngle>>8);		
			
			usartTransmit(batteryLevel&0xFF);
			usartTransmit(batteryLevel>>8);	
			
			usartTransmit(ir_move);	
			
			usartTransmit(BUTTON0);
			
			usartTransmit(CHARGE_ON);								
*/
		}

//PORTB &= ~(1 << 7);
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
			if(rfData[0]==0 && rfData[1]==0 && rfData[2]==0 && rfData[3]==0b00001000 && rfData[4]==0 && rfData[5]==0) {

				sleep(60);

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
			

			if((rfData[3]&0b00000001)==0b00000001) {	// turn on back IR
				LED_IR1_LOW;
			} else {
				LED_IR1_HIGH; 
			} 
			
			if((rfData[3]&0b00000010)==0b00000010) {	// turn on front IRs
				LED_IR2_LOW;
			} else {
				LED_IR2_HIGH;
			}

			if((rfData[3]&0b00000100)==0b00000100) {	// check the 3rd bit to enable/disable the IR receiving
				irEnabled = 1;
			} else {
				irEnabled = 0;
			}

			if((rfData[3]&0b00010000)==0b00010000) {	// check the 5th bit to start calibration of all sensors
				startCalibration = 1;
				calibrationCycle = 0;
				pwm_red = 0;
				pwm_green = 0;
				pwm_blue = 0;
				updateRedLed(pwm_red);
				updateGreenLed(pwm_green);
				updateBlueLed(pwm_blue);
			}

			if((rfData[3]&0b01000000)==0b01000000) {	// check the seventh bit to enable/disable obstacle avoidance
				obstacleAvoidanceEnabled = 1;
			} else {
				obstacleAvoidanceEnabled = 0;
			}

			if((rfData[3]&0b10000000)==0b10000000) {	// check the eight bit to enable/disable obstacle avoidance
				cliffAvoidanceEnabled = 1;
			} else {
				cliffAvoidanceEnabled = 0;
			}

			//desired_orientation = current_angle;
			

#else

			

#endif

#ifdef BIDIRECTIONAL
			//packetId = (packetId+1)%256;
			//writeAckPayload(&packetId, 1);
			//for(i=0; i<12; i++) {
			//	ackPayload[i] = (proximityValue[(i*2)+1]>>2);
			//}

			ackPayload[0] = packetId&0xFF;

			switch(packetId) {
				case 3:
					ackPayload[1] = proximityResult[0]&0xFF;
					ackPayload[2] = proximityResult[0]>>8;					
					ackPayload[3] = proximityResult[1]&0xFF;
					ackPayload[4] = proximityResult[1]>>8;
					ackPayload[5] = proximityResult[2]&0xFF;
					ackPayload[6] = proximityResult[2]>>8;
					ackPayload[7] = proximityResult[3]&0xFF;
					ackPayload[8] = proximityResult[3]>>8;
					ackPayload[9] = proximityResult[5]&0xFF;
					ackPayload[10] = proximityResult[5]>>8;
					ackPayload[11] = proximityResult[6]&0xFF;
					ackPayload[12] = proximityResult[6]>>8;
					ackPayload[13] = proximityResult[7]&0xFF;
					ackPayload[14] = proximityResult[7]>>8;	
					ackPayload[15] = CHARGE_ON | (BUTTON0 << 1);		
					packetId = 4;
					break;

				case 4:
					ackPayload[1] = proximityResult[4]&0xFF;
					ackPayload[2] = proximityResult[4]>>8;
					ackPayload[3] = proximityResult[8]&0xFF;
					ackPayload[4] = proximityResult[8]>>8;
					ackPayload[5] = proximityResult[9]&0xFF;
					ackPayload[6] = proximityResult[9]>>8;
					ackPayload[7] = proximityResult[10]&0xFF;
					ackPayload[8] = proximityResult[10]>>8;
					ackPayload[9] = proximityResult[11]&0xFF;
					ackPayload[10] = proximityResult[11]>>8;
					ackPayload[11] = accX&0xFF;	//((-accOffsetY)&0x03FF)
					ackPayload[12] = accX>>8;
					ackPayload[13] = accY&0xFF;
					ackPayload[14] = accY>>8;
					ackPayload[15] = ir_move;			
					packetId = 5;
					break;
				
				case 5:
					ackPayload[1] = proximityValue[0]&0xFF;
					ackPayload[2] = proximityValue[0]>>8;
					ackPayload[3] = proximityValue[2]&0xFF;
					ackPayload[4] = proximityValue[2]>>8;
					ackPayload[5] = proximityValue[4]&0xFF;
					ackPayload[6] = proximityValue[4]>>8;
					ackPayload[7] = proximityValue[6]&0xFF;
					ackPayload[8] = proximityValue[6]>>8;
					ackPayload[9] = proximityValue[10]&0xFF;
					ackPayload[10] = proximityValue[10]>>8;
					ackPayload[11] = proximityValue[12]&0xFF;
					ackPayload[12] = proximityValue[12]>>8;
					ackPayload[13] = proximityValue[14]&0xFF;
					ackPayload[14] = proximityValue[14]>>8;
					ackPayload[15] = currentSelector;
					packetId = 6;
					break;	
					
				case 6:
					ackPayload[1] = proximityValue[8]&0xFF;
					ackPayload[2] = proximityValue[8]>>8;
					ackPayload[3] = proximityValue[16]&0xFF;
					ackPayload[4] = proximityValue[16]>>8;
					ackPayload[5] = proximityValue[18]&0xFF;
					ackPayload[6] = proximityValue[18]>>8;
					ackPayload[7] = proximityValue[20]&0xFF;
					ackPayload[8] = proximityValue[20]>>8;
					ackPayload[9] = proximityValue[22]&0xFF;
					ackPayload[10] = proximityValue[22]>>8;
					ackPayload[11] = accZ&0xFF;
					ackPayload[12] = accZ>>8;
					ackPayload[13] = batteryLevel&0xFF;
					ackPayload[14] = batteryLevel>>8;
					ackPayload[15] = 0;				
					packetId = 3;
					break;											
								

			}

			writeAckPayload(ackPayload, 16);
#endif


		}
//PORTB |= (1 << 7);


		if(currentSelector == 0) {	// no control

			// compute velocities even if they aren't used...
			if(compute_left_vel) {
				last_left_vel = left_vel_sum>>2;
				left_vel_changed = 1;
				compute_left_vel = 0;
				left_vel_sum = 0;
			}

			if(compute_right_vel) {
				last_right_vel = right_vel_sum>>2;
				right_vel_changed = 1;
				compute_right_vel = 0;
				right_vel_sum = 0;
			}

			pwm_right_working = pwm_right_desired;	// pwm in the range 0..MAX_PWM_MOTORS
			pwm_left_working = pwm_left_desired;
				
			if(obstacleAvoidanceEnabled) {
				//PORTB &= ~(1 << 7);
				obstacleAvoidance();
				//PORTB |= (1 << 7);				
			}
				
			if(cliffAvoidanceEnabled) {
				cliffAvoidance();
			}
			
			update_pwm = 1;

		} else if(currentSelector == 2) {		// speed control

/*
			if(pwm_left==0 || pwm_right==0) {
				pwm_right_working = pwm_right_desired;
				pwm_left_working = pwm_left_desired;
				update_pwm = 1;
			}
*/

/*
			if(pwm_left==0) {
				pwm_left_working = pwm_left_desired;
				pwm_left = pwm_left_working;
				if(pwm_left >= 0) {
					OCR4A = (unsigned int)pwm_left;
				} else {
					OCR4B =(unsigned int)( -pwm_left);
				}
			}
			
			if(pwm_right==0) {
				pwm_right_working = pwm_right_desired;
				pwm_right = pwm_right_working;

				if(pwm_right >= 0) {
					OCR3A = (unsigned int)pwm_right;
				} else {
					OCR3B = (unsigned int)(-pwm_right);
				}
			}
*/

			if(compute_left_vel) {
				last_left_vel = left_vel_sum>>2;
				left_vel_changed = 1;
				compute_left_vel = 0;
				left_vel_sum = 0;
			
				pwm_left_working = pwm_left_desired;

				//if(orizzontal_position == ORIZZONTAL_POS) {
					//PORTB &= ~(1 << 5);
					start_orizzontal_speed_control_left(&pwm_left_working);
					//PORTB |= (1 << 5);
				//} else {
				//	PORTB &= ~(1 << 6);
				//	start_vertical_speed_control_left(&pwm_left_working);
				//	PORTB |= (1 << 6);
				//}

				pwm_left = pwm_left_working;

				if(pwm_left >= 0) {
					OCR4A = (unsigned int)pwm_left;
				} else {
					OCR4B =(unsigned int)( -pwm_left);
				}

				//if(pwm_left_working==0) {
				//	PORTB &= ~(1 << 6);
				//	pwm_left_working = 1;
				//}

				//update_pwm = 1;												
			}

			if(compute_right_vel) {
				last_right_vel = right_vel_sum>>2;
				right_vel_changed = 1;
				compute_right_vel = 0;
				right_vel_sum = 0;
			
				pwm_right_working = pwm_right_desired;

				//if(orizzontal_position == ORIZZONTAL_POS) {
					//PORTB &= ~(1 << 5);
					start_orizzontal_speed_control_right(&pwm_right_working);
					//PORTB |= (1 << 5);
				//} else {
				//	PORTB &= ~(1 << 6);
				//	start_vertical_speed_control_right(&pwm_right_working);
				//	PORTB |= (1 << 6);
				//}
	
				pwm_right = pwm_right_working;
	
				if(pwm_right >= 0) {
					OCR3A = (unsigned int)pwm_right;
				} else {
					OCR3B = (unsigned int)(-pwm_right);
				}

				//if(pwm_right_working==0) {
				//	PORTB &= ~(1 << 5);
				//	pwm_right_working = 1;
				//}

				//update_pwm = 1;
			}

/*
			if(left_vel_changed && right_vel_changed) {
				pwm_right_working = pwm_right_desired;
				pwm_left_working = pwm_left_desired;
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
*/

		}

/*
		if(compute_left_vel) {
			last_left_vel = left_vel_sum>>2;
			left_vel_changed = 1;
			compute_left_vel = 0;
			left_vel_sum = 0;
		}

		if(compute_right_vel) {
			last_right_vel = right_vel_sum>>2;
			right_vel_changed = 1;
			compute_right_vel = 0;
			right_vel_sum = 0;
		}
*/

		if(update_pwm) {

			update_pwm = 0;
			pwm_left = pwm_left_working;
			pwm_right = pwm_right_working;

			if(pwm_right >= 0) {
				OCR3A = (unsigned int)pwm_right;
			} else {
				OCR3B = (unsigned int)(-pwm_right);
			}
			if(pwm_left >= 0) {
				OCR4A = (unsigned int)pwm_left;
			} else {
				OCR4B =(unsigned int)( -pwm_left);
			}
/*
			if(pwm_left == 0) {
				// select channel 15 to sample left current
				//currentMotLeftChannel = 15;
				TCCR4A  &= ~(1 << COM4A1) & ~(1 << COM4B1);	// disable OCA and OCB
				PORTH &= ~(1 << 4) & ~(1 << 3);				// output to 0
				TIMSK4 &= ~(1 << OCIE4B) & ~(1 << OCIE4A);	// disable OCA and OCB interrupt
				TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt => sampling of velocity is enabled even if 
												// the pwm is turned off...is it correct??
			} else if(pwm_left > 0) {   		// move forward
				// select channel 15 to sample left current
				//currentMotLeftChannel = 15;
				TCCR4A  &= ~(1 << COM4B1);		// disable OCB
				TIMSK4 &= ~(1 << OCIE4B);		// disable OCB interrupt
				PORTH &= ~(1 << 4);				// output to 0
				TCCR4A |= (1 << COM4A1);		// enable OCA
				TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt
			} else if(pwm_left < 0) {      		// move backward
				// select channel 14 to sample left current
				//currentMotLeftChannel = 14;
				TCCR4A  &= ~(1 << COM4A1);		// disable OCA
				TIMSK4 &= ~(1 << OCIE4A);		// disable OCA interrupt
				PORTH &= ~(1 << 3);				// output to 0
				TCCR4A |= (1 << COM4B1);		// enable OCB
				TIMSK4 |= (1 << OCIE4B);		// enable OCB interrupt
			}

			if(pwm_right == 0) {
				// select channel 13 to sample left current
				//currentMotRightChannel = 13;
				TCCR3A  &= ~(1 << COM3A1) & ~(1 << COM3B1);	// disable OCA and OCB
				PORTE &= ~(1 << 4) & ~(1 << 3);				// output to 0
				TIMSK3 &= ~(1 << OCIE3B) & ~(1 << OCIE3A);	// disable OCA and OCB interrupt
				TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt => sampling of velocity is enabled even if 
												// the pwm is turned off...is it correct??
			}else if(pwm_right > 0) {   		// move forward
				// select channel 13 to sample left current
				//currentMotRightChannel = 13;
				TCCR3A  &= ~(1 << COM3B1);		// disable OCB
				TIMSK3 &= ~(1 << OCIE3B);		// disable OCB interrupt
				PORTE &= ~(1 << 4);				// output to 0
				TCCR3A |= (1 << COM3A1);		// enable OCA
				TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt
			} else if(pwm_right < 0) {      	// move backward
				// select channel 12 to sample left current
				//currentMotRightChannel = 12;
				TCCR3A  &= ~(1 << COM3A1);		// disable OCA
				TIMSK3 &= ~(1 << OCIE3A);		// disable OCA interrupt
				PORTE &= ~(1 << 3);				// output to 0
				TCCR3A |= (1 << COM3B1);		// enable OCB
				TIMSK3 |= (1 << OCIE3B);		// enable OCB interrupt
			}
*/
		}

	} // while(1)

}
