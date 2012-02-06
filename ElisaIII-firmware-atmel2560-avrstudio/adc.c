
#include "adc.h"


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
			    // sometimes it was noticed that the velocity is sampled even if the pwm
			    // is in its active phase; as a workaround simply skip the samples in these
			    // cases
				if(((PINE & _BV(PE3))>>3) || ((PINE & _BV(PE4))>>4)) {  // if active phase for either forward or backward direction
					//PORTB &= ~(1 << 5);
					break;
				}
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
				if(((PINH & _BV(PH3))>>3) || ((PINH & _BV(PH4))>>4)) {  // if active phase for either forward or backward direction
					//PORTB &= ~(1 << 5);
					break;
				}
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
		//#warning "turn off pulse with 0 (hw rev 3.0)"
		#endif

		#ifdef HW_REV_3_0_1
		PORTJ = 0xFF;
		PORTA = 0x00;
		//#warning "turn off pulse with 0 (hw rev 3.0.1)"
		#endif

		#ifdef HW_REV_3_1
		PORTJ = 0xFF;
		PORTA = 0x00;
		//#warning "turn off pulse with 1 (hw rev 3.1)"
		#endif

	}

//	PORTB |= (1 << 7);

}


