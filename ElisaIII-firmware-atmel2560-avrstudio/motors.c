
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
