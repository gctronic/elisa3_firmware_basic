
#include "adc.h"


void initAdc(void) {

	// ADCSRA -----> ADEN	ADSC	 ADATE	ADIF	ADIE	 ADPS2	ADPS1	ADPS0
	// default		 0		0		 0		0		0		 0		0		0
	// ADMUX  -----> REFS1	REFS0	 ADLAR	MUX4	MUX3 	 MUX2	MUX1	MUX0
	// default		 0		0		 0		0		0		 0		0		0
	// ADCSRB -----> -		ACME	 - 		- 		MUX5 	 ADTS2 	ADTS1 	ADTS0
	// default		 0		0		 0		0		0		 0		0		0

	ADCSRA = 0;
	ADCSRB = 0;
	ADMUX = 0;

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	// 1/64 prescaler => 8 MHz/64=125 KHz => Tad (adc clock)
											// one sample need 13 Tad in free running mode, so interrupt 
											// frequency is 125/13=9.6 KHz (104 us between adc interrupts)
	ADMUX |= (1 << REFS0); 	// voltage reference to AVCC (external)
	ADCSRA |= (1 << ADATE); // auto-trigger mode: the new sampling is started just after the last one is completed
	ADCSRB &= 0xF8;			// for safety...ADTS2:0 in ADCSRB should be already set to free running by default (0b000)
	ADCSRA |= (1 << ADIE);	// enable interrupt on conversion completion
	ADCSRA |= (1 << ADEN);	// enable ADC
	ADCSRA |= (1 << ADSC);	// start first conversion (start from channel 0)

}

ISR(ADC_vect) {

	// ADIF is cleared by hardware when executing the corresponding interrupt handling vector

	// channel 0..6:  prox0..6
	// channel 7:	  prox7/battery
	// channel 8..11: cliff0..3
	// channel 12:	  active phase when going backward: motor right current; passive phase when going forward: motor right velocity 
	// channel 13.	  active phase when going forward: motor right current; passive phase when going backward: motor right velocity
	// channel 14:    active phase when going backward: motor left current; passive phase when going forward: motor left velocity 
	// channel 15:    active phase when going forward: motor left current; passive phase when going backward: motor left velocity


	//LED_BLUE_ON;

	clockTick++;				// this variable is used as base time for timed processes/functions (e,g, delay); 
								// resolution of 104 us based on adc interrupts

	int value = ADCL;			// get the sample; low byte must be read first!!
	value = (ADCH<<8) | value;

	// save the last sampled data in the correct position; the sequence is:
	// prox0 passive phase | motor left | motor right | motor left | motor right | 
	// prox0 active phase  | motor left | motor right | motor left | motor right | 
	// prox1 passive phase | ...
	// motor left and motor right indicate either the sampling of the current consumption or 
	// the velocity respectively for the left and right motor; discrimination between the 
	// current and velocity is done in the motors timers interrupts in which is flagged the pwm 
	// phase (active=>current or passive=>velocity) of the motors
	switch(adcSaveDataTo) {

		case SAVE_TO_PROX:
			if(currentProx==14 && measBattery==2) {		// about every 2 seconds the battery level is sampled; both
				batteryLevel = value;					// the proximity 7 and battery are connected to the same adc channel
				measBattery = 0;
				SENS_ENABLE_OFF;						// the adc channel is connected to proximity
			} else {
				proximityValue[currentProx] = value;	// even indexes contain ambient values; odd indexes contains "reflected" values
			}

			if(currentProx & 0x01) {
				proximityResult[currentProx>>1] = proximityValue[currentProx-1] - proximityValue[currentProx] - proximityOffset[currentProx>>1];	// ambient - (ambient+reflected) - offset
				if(proximityResult[currentProx>>1] < 0) {
					proximityResult[currentProx>>1] = 0;
				}
				if(proximityResult[currentProx>>1] > 1024) {
					proximityResult[currentProx>>1] = 1024;
				}

				// linearization of the proximity values: the values of the proximity will range from 
				// 0 to 255 after linearization and decrease linearly with distance.
				// The linearization of the proximity values is done using four linear functions:
				// 1) from 0 to PHASE1: y = x (where x = proximity value9
				// 2) from PHASE1 to PHASE2: y = x/2 + 30
				// 3) from PHASE2 to PHASE3: y = x/4 + 75
				// 4) from PHASE3 upwards: y = x/8 + 127.5
				// The linearized values are used for the obstacles avoidance.
				if(currentProx < 16) {	// only for proximity (not ground sensors)
					
					if(proximityResult[currentProx>>1] < PHASE1) {

						proximityResultLinear[currentProx>>1] = proximityResult[currentProx>>1];

					} else if(((proximityResult[currentProx>>1]+60)>>1) < PHASE2) {
				
						proximityResultLinear[currentProx>>1] = ((proximityResult[currentProx>>1]-60)>>1) + PHASE1;

					} else if(((proximityResult[currentProx>>1]+300)>>2) < PHASE3) {

						proximityResultLinear[currentProx>>1] = ((proximityResult[currentProx>>1]-180)>>2) + PHASE2;

					} else {

						proximityResultLinear[currentProx>>1] = ((proximityResult[currentProx>>1]-420)>>3) + PHASE3;
						
					}

				}

				// the cliff avoidance behavior is inserted within this interrupt service routine in order to react
				// as fast as possible; the maximum speed usable with cliff avoidance is 30 in all kind of surface 
				// (apart from black ones) after calibration.
				if(cliffAvoidanceEnabled) {
					if(proximityResult[8]<CLIFF_THR || proximityResult[9]<CLIFF_THR || proximityResult[10]<CLIFF_THR || proximityResult[11]<CLIFF_THR) {
						cliffDetectedFlag = 1;
						//LED_RED_ON;			
						// set resulting velocity to 0 and change the pwm registers directly to be able
						// to stop as fast as possible (the next pwm cycle)
						// left motor
						pwm_left = 0;
						OCR4A = 0;
						OCR4B = 0;
						// right motor
						pwm_right = 0;
						OCR3A = 0;
						OCR3B = 0;
					} else {
						cliffDetectedFlag = 0;
						//LED_RED_OFF;
					}
				} else {
					cliffDetectedFlag = 0;
				}

			}

			currentProx++;
			if(currentProx > 23) {						// in total there are 8 proximity sensors and 4 ground sensors => 12 sensors
				currentProx = 0;						// for each one there is a passive phase in which the ambient light is sampled,
				proxUpdated = 1;							// and an active phase in which an IR pulse is turned on and the reflected light 
			}											// is sampled; thus 12 sensors x 2 phases = 24 samples
			break;

		case SAVE_TO_RIGHT_MOTOR_CURRENT:
			right_current_avg += value;
			right_current_avg = right_current_avg >> 1;	// the current consumption is an estimate, not really an average of the samples
			break;

		case SAVE_TO_RIGHT_MOTOR_VEL:
			if(firstSampleRight > 0) {
			    // sometimes it was noticed that the velocity is sampled even if the pwm
			    // is in its active phase; as a workaround simply skip the samples in these
			    // cases
				if(((PINE & _BV(PE3))>>3) || ((PINE & _BV(PE4))>>4)) {  // if active phase for either forward or backward direction
					//LED_RED_ON;
					break;
				}
				firstSampleRight++;
				if(firstSampleRight > 4) {				// to skip undesired samples (3 samples skipped) in which there could be glitches
					right_vel_sum += value;
					if(firstSampleRight==8) {			// number of samples to take for the speed computation (average of 4 samples)
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
			if(firstSampleLeft > 0) {
				if(((PINH & _BV(PH3))>>3) || ((PINH & _BV(PH4))>>4)) {
					//LED_RED_ON;
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

		case SKIP_SAMPLE:								// this case isn't used anymore; it was used to avoid sampling the motors velocity
			break;										// when the desired speed was zero. Now the speed is always sampled independently 
														// of the desired velocity.
	}			

	// select next channel to sample based on the previous sequence and actual motors pwm phase
	switch(adcSamplingState) {

		case 0:	// proximity
			currentAdChannel = currentProx>>1;				// select the channel to sample after next interrupt (in which the adc register is updated with the new channel)
															// currentProx goes from 0 to 23, currentAdChannel from 0 to 11
			if(rightChannelPhase == ACTIVE_PHASE) {			// select where to save the data that we're sampling
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
				SENS_ENABLE_ON;			// next time measure battery instead of proximity 7
			}

			// turn on the IR pulses for the proximities only in their active phases
			if(currentProx & 0x01) {
				if(currentProx < 16) {	// pulse for proximity and ground sensors are placed in different ports;
										// PORTA for proximity sensors, PORTJ for ground sensors
					PORTA = (1 << (currentProx>>1));	// pulse on
				} else {
					if(hardwareRevision == HW_REV_3_0) {
						PORTJ = (1 << ((currentProx-16)>>1));	// pulse on
					}

					if(hardwareRevision == HW_REV_3_0_1) {
						PORTJ &= ~(1 << ((currentProx-16)>>1));	// pulse on (inverse logic)
					}

					if(hardwareRevision == HW_REV_3_1) {
						PORTJ &= ~(1 << ((currentProx-16)>>1));	// pulse on (inverse logic)
					}

				}
			}
			break;

	}

	// channel selection in the adc register; continuously manually change the channel 
	// sampled since there is no automatic way of doing it
	if(currentAdChannel < 8) {		// MUX5=0 + ADMUX=0..7 => adc channel=0..7
		ADCSRB &= ~(1 << MUX5);
		ADMUX = 0x40 + currentAdChannel;
	} else {						// MUX5=1 + ADMUX=0..7 => adc channel=8..15
		ADCSRB |= (1 << MUX5);
		ADMUX = 0x40 + (currentAdChannel-8);
	}

	// turn off the proximity IR pulses in order to have 200 us of pulse
	if(adcSamplingState == 2) {

		if(hardwareRevision == HW_REV_3_0) {
			PORTJ &= 0xF0;
			PORTA = 0x00;
			//#warning "turn off pulse with 0 (hw rev 3.0)"
		}

		if(hardwareRevision == HW_REV_3_0_1) {
			PORTJ = 0xFF;
			PORTA = 0x00;
			//#warning "turn off pulse with 0 (hw rev 3.0.1)"
		}

		if(hardwareRevision == HW_REV_3_1) {
			PORTJ = 0xFF;
			PORTA = 0x00;
			//#warning "turn off pulse with 1 (hw rev 3.1)"
		}

	}

	//LED_BLUE_OFF;

}


