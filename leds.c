
#include "leds.h"


void initRGBleds() {

	// LEDs timer1/pwm
	// Timer1 clock input = Fosc = 8 MHz
	// Period freq = Fosc/TOP (max timer value) => TOP = Fosc/period freq
	// We need a frequency of about 30 KHz => 8000000/30000 = 266
	// The waveform generation mode let us chose the TOP value to be 256
	// thus we get period freq = 8000000/256 = 31250 Hz

	TCCR1A = 0;
	TCCR1B = 0;

	// enable OCA, OCB, OCC; clear on match, set at bottom
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1) | (1 << WGM10); 	
	// mode 5 => fast-pwm 8 bit; no prescaler
	TCCR1B |= (1 << WGM12) | (1 << CS10);										
	// the values for the leds pwm goes from 0 (max power on) to 255 (off)
	OCR1A = pwm_red;
	OCR1B = pwm_green;
	OCR1C = pwm_blue;

}

void toggleBlueLed() {

	blinkState = 1 - blinkState;

	if(blinkState) {
		TCCR1A |= (1 << COM1C1);	// always enable OCC in case it was disabled
		OCR1C = 255;
	} else {
		TCCR1A &= ~(1 << COM1C1);	// disable OCC to get the maximum output power; this is due to the fact 
		PORTB &= ~(1 << 7);			// that the minimum duty cycle when the output compare is enable is 1 
									// (duty cycle = 0 is not possible); thus the peripheral is disabled and 
									// the pin is configured accordingly (low state).
	}

}

void updateRedLed(unsigned char value) {

	if(value == 0) {
		TCCR1A &= ~(1 << COM1A1);	// disabel OCA
		PORTB &= ~(1 << 5);			// set pin state to turn on the led
	} else {
		TCCR1A |= (1 << COM1A1);	// always enable OCA in case it was disabled
		OCR1A = value;
	}

}

void updateGreenLed(unsigned char value) {

	if(value == 0) {
		TCCR1A &= ~(1 << COM1B1);	// disable OCB
		PORTB &= ~(1 << 6);			// set pin state to turn on the led
	} else {	
		TCCR1A |= (1 << COM1B1);	// always enable OCA in case it was disabled
		OCR1B = value;
	}

}

void updateBlueLed(unsigned char value) {

	if(value == 0) {
		TCCR1A &= ~(1 << COM1C1);	// disable OCC
		PORTB &= ~(1 << 7);			// set pin state to turn on the led
	} else {
		TCCR1A |= (1 << COM1C1);	// always enable OCA in case it was disabled
		OCR1C = value;
	}

}

void setGreenLed(unsigned char ledNum, unsigned char isOn) {

	switch(ledNum) {

		case 0:	isOn?GREEN_LED0_ON:GREEN_LED0_OFF;
				break;

		case 1:	isOn?GREEN_LED1_ON:GREEN_LED1_OFF;
				break;

		case 2:	isOn?GREEN_LED2_ON:GREEN_LED2_OFF;
				break;

		case 3:	isOn?GREEN_LED3_ON:GREEN_LED3_OFF;
				break;

		case 4:	isOn?GREEN_LED4_ON:GREEN_LED4_OFF;
				break;

		case 5:	isOn?GREEN_LED5_ON:GREEN_LED5_OFF;
				break;

		case 6:	isOn?GREEN_LED6_ON:GREEN_LED6_OFF;
				break;

		case 7:	isOn?GREEN_LED7_ON:GREEN_LED7_OFF;
				break;

		default: break;

	}

}

void turnOffGreenLeds() {

	GREEN_LED0_OFF;
	GREEN_LED1_OFF;
	GREEN_LED2_OFF;
	GREEN_LED3_OFF;
	GREEN_LED4_OFF;
	GREEN_LED5_OFF;
	GREEN_LED6_OFF;
	GREEN_LED7_OFF;

}


void turnOnGreenLeds() {

	GREEN_LED0_ON;
	GREEN_LED1_ON;
	GREEN_LED2_ON;
	GREEN_LED3_ON;
	GREEN_LED4_ON;
	GREEN_LED5_ON;
	GREEN_LED6_ON;
	GREEN_LED7_ON;

}


