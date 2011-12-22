
#include <avr\io.h>
#include <avr\interrupt.h>
#include "usart.h"

// speed controller
extern signed long int pwm_right;
extern signed long int pwm_left;

// uart
extern unsigned char peripheralChoice;
extern unsigned char choosePeripheral;
extern unsigned char sendAdcValues;

// rgb leds
extern unsigned char pwm_red;
extern unsigned char pwm_green;
extern unsigned char pwm_blue;

void initUsart() {

	// clock is 8 MHz, thus:
	// Normal mode:
	// @9600 baud: 8000000/16/9600-1 = 51 => 8000000/16/52 = 9615 => 100-(9600/9615*100)=0.15% of error
	// @19200 baud: 8000000/16/19200-1 = 25 => 8000000/16/26 = 19230 => 100-(19200/19230*100)=0.15% of error
	// @38400 baud: 8000000/16/38400-1 = 12 => 8000000/16/13 = 38461 => 100-(38400/38461*100)=0.15% of error
	// Double speed mode:
	// @57600 baud: 8000000/8/57600-1 = 16 => 8000000/8/17 = 58823 => 100-(57600/58823*100)=2.08% of error	

	// set baudrate
	UBRR0H = 0;
	UBRR0L = 16;

	//UCSR0A  &= ~(1 << U2X0);								// disable double transmission speed
	UCSR0A  |= (1 << U2X0);									// enable double speed

	UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);	// enable uart0 transmitter and receiver; enable rx interrupt
	
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);					// set frame format: 8data, no parity, 1 stop bit

}

void usartTransmit(unsigned char data) {

	// wait for empty transmit buffer
	while (!(UCSR0A & (1<<UDRE0)));

	// put data into buffer, sends the data
	UDR0 = data;

}

ISR(USART0_RX_vect) {

	char receivedByte = UDR0;

	if(choosePeripheral) {
		switch(receivedByte) {
			case '0': // red led
				peripheralChoice = 0;
				choosePeripheral = 0;
				break;
			case '1': // green led
				peripheralChoice = 1;
				choosePeripheral = 0;
				break;
			case '2': // blue led
				peripheralChoice = 2;
				choosePeripheral = 0;
				break;
			case '3': // right motor
				peripheralChoice = 3;
				choosePeripheral = 0;
				break;
			case '4': // left motor
				peripheralChoice = 4;
				choosePeripheral = 0;
				break;
			case '5':
				peripheralChoice = 5;
				choosePeripheral = 0;
				sendAdcValues = 1;
				break;
			default:
				break;				 
		}

	} else {	// apply values to chosen peripheral

		int current_pwm=0;

		switch(peripheralChoice) {
			case 0:	// red led
				if(receivedByte == '-') {
					TCCR1A |= (1 << COM1A1);	// enable OCA
					current_pwm = pwm_red+10;
					if(current_pwm > 255) {
						current_pwm = 255;
					}
					pwm_red = current_pwm;
					OCR1A = pwm_red;
				} else if(receivedByte == '+') {
					current_pwm = pwm_red-10;
					if(current_pwm < 0) {
						current_pwm = 0;
					}
					pwm_red = current_pwm;
					if(pwm_red == 0) {
						TCCR1A &= ~(1 << COM1A1);
						PORTB &= ~(1 << 5);
					} else {
						OCR1A = pwm_red;
					}

				} else {
					choosePeripheral = 1;
				}
				break;
			case 1:	// green led
				if(receivedByte == '-') {
					TCCR1A |= (1 << COM1B1);	// enable OCB
					current_pwm = pwm_green+10;
					if(current_pwm > 255) {
						current_pwm = 255;
					}
					pwm_green = current_pwm;
					OCR1B = pwm_green;
				} else if(receivedByte == '+') {
					current_pwm = pwm_green-10;
					if(current_pwm < 0) {
						current_pwm = 0;
					}
					pwm_green = current_pwm;
					if(pwm_green == 0) {
						TCCR1A &= ~(1 << COM1B1);
						PORTB &= ~(1 << 6);
					} else {
						OCR1B = pwm_green;
					}
				} else {
					choosePeripheral = 1;
				}
				break;
			case 2: // blue led
				if(receivedByte == '-') {
					TCCR1A |= (1 << COM1C1);	// enable OCC
					current_pwm = pwm_blue+10;
					if(current_pwm > 255) {
						current_pwm = 255;
					}
					pwm_blue = current_pwm;
					OCR1C = pwm_blue;
				} else if(receivedByte == '+') {
					current_pwm = pwm_blue-10;
					if(current_pwm < 0) {
						current_pwm = 0;
					}
					pwm_blue = current_pwm;
					if(pwm_blue == 0) {
						TCCR1A &= ~(1 << COM1C1);
						PORTB &= ~(1 << 7);
					} else {
						OCR1C = pwm_blue;
					}
				} else {
					choosePeripheral = 1;
				}
				break;
			case 3: // right motor
				if(receivedByte == '+') {
					pwm_right += STEP_MOTORS;
					if(pwm_right > MAX_MOTORS_PWM) {
						pwm_right = MAX_MOTORS_PWM;
					}
					if(pwm_right >= 0) {
						OCR3A = (int)pwm_right;
					} else {
						OCR3B = (int)(-pwm_right);
					}
				} else if(receivedByte == '-') {
					pwm_right -= STEP_MOTORS;
					if(pwm_right < -MAX_MOTORS_PWM) {
						pwm_right = -MAX_MOTORS_PWM;
					}
					if(pwm_right >= 0) {
						OCR3A = (int)pwm_right;		// I set the new value for the output compares here
					} else {						// so the next timer interrupt the values are immediately
						OCR3B = (int)(-pwm_right);	// updated
					}
				} else if(receivedByte == 's') {
					pwm_right = 0;
					OCR3A = 0;
					OCR3B = 0;
				} else {
					choosePeripheral = 1;
				}
				break;
			case 4: // left motor
				if(receivedByte == '+') {
					pwm_left += STEP_MOTORS;
					if(pwm_left > MAX_MOTORS_PWM) {
						pwm_left = MAX_MOTORS_PWM;
					}
					if(pwm_left >= 0) {
						OCR4A = pwm_left;
					} else {
						OCR4B = -pwm_left;
					}
				} else if(receivedByte == '-') {
					pwm_left -= STEP_MOTORS;
					if(pwm_left < -MAX_MOTORS_PWM) {
						pwm_left = -MAX_MOTORS_PWM;
					}
					if(pwm_left >= 0) {
						OCR4A = pwm_left;
					} else {
						OCR4B = -pwm_left;
					}
				} else if(receivedByte == 's') {
					pwm_left = 0;
					OCR4A = 0;
					OCR4B = 0;
				} else {
					choosePeripheral = 1;
				}
				break;
			case 5: // adc
				if(receivedByte == 's') {
					sendAdcValues = 0;
					choosePeripheral = 1;
				}

		}		

	}



}
