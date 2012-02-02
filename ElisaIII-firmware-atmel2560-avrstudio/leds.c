
#include "leds.h"


void initRGBleds() {
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
