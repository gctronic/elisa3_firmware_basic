
#include "motors.h"

void initMotors() {

	// Motor right timer3/pwm
	// Timers clock input = Fosc = 8 MHz
	// Period freq = Fosc/TOP (max timer value) => TOP = Fosc/period freq
	// We need a period time of 10 ms (100 Hz)
	// Using 10-bit resolution (waveform generation mode 7) we have a period of: 8000000/1024 = 7812.5 Hz
	// We need to apply a prescaler to the timer in such a way to get the desired period:
	// 7812.5/100 = 78.125 => ideal prescaler, the nearest one is 1/64 and we get a period of:
	// 8000000/64/1024 = 122 Hz

	TCCR3A = 0;
	TCCR3B = 0;
	TIMSK3 = 0;
	TCCR4A = 0;
	TCCR4B = 0;
	TIMSK4 = 0;

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

void handleMotorsWithNoController() {

	// compute velocities even if they aren't used
	if(compute_left_vel) {
		last_left_vel = left_vel_sum>>2;
		compute_left_vel = 0;
		left_vel_sum = 0;

		if(pwm_left_desired >= 0) {
			leftMotSteps += (last_left_vel>>3);
		} else {
			leftMotSteps -= (last_left_vel>>3);
		}
	}

	if(compute_right_vel) {
		last_right_vel = right_vel_sum>>2;
		compute_right_vel = 0;
		right_vel_sum = 0;

		if(pwm_right_desired >= 0) {
			rightMotSteps += (last_right_vel>>3);
		} else {
			rightMotSteps -= (last_right_vel>>3);
		}
	}


	pwm_right_working = pwm_right_desired;	// pwm in the range 0..MAX_PWM_MOTORS
	pwm_left_working = pwm_left_desired;
	if(obstacleAvoidanceEnabled) {
		obstacleAvoidance(&pwm_left_working, &pwm_right_working);
	}
	pwm_left_desired_to_control = pwm_left_working;
	pwm_right_desired_to_control = pwm_right_working;

	pwm_left = pwm_left_working;
	pwm_right = pwm_right_working;

	if(pwm_right > 0) {
		OCR3A = (unsigned int)pwm_right;
	} else if(pwm_right < 0) {
		OCR3B = (unsigned int)(-pwm_right);
	} else {
		OCR3A = 0;
		OCR3B = 0;
	}
	if(pwm_left > 0) {
		OCR4A = (unsigned int)pwm_left;
	} else if(pwm_left < 0) {
		OCR4B =(unsigned int)( -pwm_left);
	} else {
		OCR4A = 0;
		OCR4B = 0;
	}

}

void handleMotorsWithSpeedController() {

	pwm_left_working = pwm_left_desired;
	pwm_right_working = pwm_right_desired;
	if(obstacleAvoidanceEnabled) {
		obstacleAvoidance(&pwm_left_working, &pwm_right_working);
	}
	pwm_left_desired_to_control = pwm_left_working;
	pwm_right_desired_to_control = pwm_right_working;

	if(compute_left_vel) {

		last_left_vel = left_vel_sum>>2;
		compute_left_vel = 0;
		left_vel_sum = 0;

		if(pwm_left_desired_to_control >= 0) {
			leftMotSteps += (last_left_vel>>3);
		} else {
			leftMotSteps -= (last_left_vel>>3);
		}

		if(robotPosition == HORIZONTAL_POS) {
			//PORTB &= ~(1 << 5);
			start_horizontal_speed_control_left(&pwm_left_working);
			//PORTB |= (1 << 5);
		} else {
			//PORTB &= ~(1 << 6);
			start_vertical_speed_control_left(&pwm_left_working);
			//PORTB |= (1 << 6);
		}

		pwm_left = pwm_left_working;

		if(pwm_left > 0) {
			OCR4A = (unsigned int)pwm_left;
		} else if(pwm_left < 0) {
			OCR4B =(unsigned int)( -pwm_left);
		} else {
			OCR4A = 0;
			OCR4B = 0;
		}

	}

	if(compute_right_vel) {

		last_right_vel = right_vel_sum>>2;
		compute_right_vel = 0;
		right_vel_sum = 0;

		if(pwm_right_desired_to_control >= 0) {
			rightMotSteps += (last_right_vel>>3);
		} else {
			rightMotSteps -= (last_right_vel>>3);
		}

		if(robotPosition == HORIZONTAL_POS) {
			//PORTB &= ~(1 << 5);
			start_horizontal_speed_control_right(&pwm_right_working);
			//PORTB |= (1 << 5);
		} else {
			//PORTB &= ~(1 << 6);
			start_vertical_speed_control_right(&pwm_right_working);
			//PORTB |= (1 << 6);
		}

		pwm_right = pwm_right_working;

		if(pwm_right > 0) {
			OCR3A = (unsigned int)pwm_right;
		} else if(pwm_right < 0) {
			OCR3B = (unsigned int)(-pwm_right);
		} else {
			OCR3A = 0;
			OCR3B = 0;
		}

	}

}

void setLeftSpeed(signed char vel) {

	speedl = abs(vel);

    if(vel >= 0) {
        pwm_left_desired = speedl<<2;
    } else {
        pwm_left_desired = -(speedl<<2);
    }

	if (pwm_left_desired>(MAX_MOTORS_PWM/2)) pwm_left_desired=(MAX_MOTORS_PWM/2);
	if (pwm_left_desired<-(MAX_MOTORS_PWM/2)) pwm_left_desired=-(MAX_MOTORS_PWM/2);

}

void setRightSpeed(signed char vel) {

	speedr = abs(vel);

    if(vel >= 0) {
        pwm_right_desired = speedr<<2;
    } else {
        pwm_right_desired = -(speedr<<2);
    }

	if (pwm_right_desired>(MAX_MOTORS_PWM/2)) pwm_right_desired=(MAX_MOTORS_PWM/2);
	if (pwm_right_desired<-(MAX_MOTORS_PWM/2)) pwm_right_desired=-(MAX_MOTORS_PWM/2);

}

// Motor left
ISR(TIMER4_OVF_vect) {

//	LED_GREEN_ON;

	if(cliffDetectedFlag) {
		pwm_left = 0;
		OCR4A = 0;
		OCR4B = 0;
	}

	left_current_avg = 0;

	// set pins mode based on controller output
	if(pwm_left == 0) {
		//firstSampleLeft = 0;


		//leftMotorPhase = NO_PHASE;
		//compute_left_vel = 1;

		if(pwm_left_desired_to_control >= 0) {
			leftMotorPhase = PASSIVE_PHASE;
			currentMotLeftChannel = 14;
		} else {
			leftMotorPhase = PASSIVE_PHASE;
			currentMotLeftChannel = 15;
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

/*
	// set channel to sample based on desired velocity (direction);
	// we cannot rely on the sole controller output value to decide which
	// channel to sample due to the "zero" special case: there are basically
	// two zeros, one for the forward direction and one for the backward.
	// The controller thus can output zero both when running forward and backward
	// but the correct channel to sample is different in each of the two cases.
	// Thus it's better to check the desired velocity that gives us the direction
	// of the robot (the controller cannot change the direction) and consequently
	// the correct channel to sample.
	if(pwm_left_desired >= 0) {
		leftMotorPhase = ACTIVE_PHASE;
		currentMotLeftChannel = 15;
	} else {
		leftMotorPhase = ACTIVE_PHASE;
		currentMotLeftChannel = 14;
	}
*/

//	LED_GREEN_OFF;

}

// motor left forward
ISR(TIMER4_COMPA_vect) {

//	LED_GREEN_ON;

//	if(pwm_left == 0) {
//		return;
//	}

	leftMotorPhase = PASSIVE_PHASE;
	// select channel 14 to sample the left velocity
	currentMotLeftChannel = 14;

	firstSampleLeft = 1;

//	LED_GREEN_OFF;

}

// motor left backward
ISR(TIMER4_COMPB_vect) {

//	LED_GREEN_ON;

//	if(pwm_left == 0) {
//		return;
//	}

	leftMotorPhase = PASSIVE_PHASE;
	// select channel 15 to sample the left velocity
	currentMotLeftChannel = 15;

	firstSampleLeft = 1;

//	LED_GREEN_OFF;

}

// Motor right
ISR(TIMER3_OVF_vect) {

//	LED_GREEN_ON;

  	// PORTB ^= (1 << 7); // Toggle the LED

	if(cliffDetectedFlag) {
		pwm_right = 0;
		OCR3A = 0;
		OCR3B = 0;
	}

	right_current_avg = 0;


	if(pwm_right == 0) {
		//firstSampleRight = 0;

		//rightMotorPhase = NO_PHASE;
		//compute_right_vel = 1;

		if(pwm_right_desired_to_control >= 0) {
			rightMotorPhase = PASSIVE_PHASE;
			// select channel 13 to sample left current
			currentMotRightChannel = 12;
		} else {
			rightMotorPhase = PASSIVE_PHASE;
			// select channel 12 to sample left current
			currentMotRightChannel = 13;
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

/*
	if(pwm_right_desired >= 0) {
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 13 to sample left current
		currentMotRightChannel = 13;
	} else {
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 12 to sample left current
		currentMotRightChannel = 12;
	}
*/
//	LED_GREEN_OFF;

}

// motor right forward
ISR(TIMER3_COMPA_vect) {

//	LED_RED_ON;

//	if(pwm_right == 0) {
//		return;
//	}

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 12 to sample the right velocity
	currentMotRightChannel = 12;

	firstSampleRight = 1;

//	LED_RED_OFF;

}

// motor right backward
ISR(TIMER3_COMPB_vect) {

//	LED_RED_ON;

//	if(pwm_right == 0) {
//		return;
//	}

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 13 to sample the right velocity
	currentMotRightChannel = 13;

	firstSampleRight = 1;

//	LED_RED_OFF;
}
