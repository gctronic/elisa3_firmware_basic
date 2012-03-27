
#include "usart.h"
#include "leds.h"

void initUsart0() {

	// clock is 8 MHz, thus:
	// Normal mode:
	// @9600 baud: 8000000/16/9600-1 = 51 => 8000000/16/52 = 9615 => 100-(9600/9615*100)=0.15% of error
	// @19200 baud: 8000000/16/19200-1 = 25 => 8000000/16/26 = 19230 => 100-(19200/19230*100)=0.15% of error
	// @38400 baud: 8000000/16/38400-1 = 12 => 8000000/16/13 = 38461 => 100-(38400/38461*100)=0.15% of error
	// Double speed mode:
	// @57600 baud: 8000000/8/57600-1 = 16 => 8000000/8/17 = 58823 => 100-(57600/58823*100)=2.08% of error	


	UBRR0H = 0;												// set baudrate
	UBRR0L = 16;
	UCSR0A  |= (1 << U2X0);									// enable double speed
	//UCSR0A &= ~(1 << U2X0);
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);	// enable uart0 transmitter and receiver; enable rx interrupt for use with aseba
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);					// set frame format: 8-bit data, no parity, 1 stop bit



}

void initUsart1() {

	UBRR1H = 0;												// set baudrate
	UBRR1L = 16;
	UCSR1A  |= (1 << U2X1);									// enable double speed
	//UCSR0A &= ~(1 << U2X1);
	UCSR1B |= (1 << TXEN1) | (1 << RXEN1);					// enable uart0 transmitter and receiver
	UCSR1C |= (1<<UCSZ11) | (1<<UCSZ10);					// set frame format: 8-bit data, no parity, 1 stop bit

}

void closeUsart() {

	UCSR0A = 0x00;	// clear all usart registers
	UCSR0B = 0x00;
	UCSR0C = 0x00;

}

void usart0Transmit(unsigned char data, unsigned char isBlocking) {

	while (!(UCSR0A & (1<<UDRE0)));		// wait for empty transmit buffer
	UDR0 = data;						// put data into buffer, sends the data
	if(isBlocking) {
		while (!(UCSR0A & (1<<TXC0)));	// wait transmission complete
	}
}

void usart1Transmit(unsigned char data, unsigned char isBlocking) {

	while (!(UCSR1A & (1<<UDRE1)));		// wait for empty transmit buffer
	UDR1 = data;						// put data into buffer, sends the data
	if(isBlocking) {
		while (!(UCSR1A & (1<<TXC1)));	// wait transmission complete
	}

}

char usart0InputBufferEmpty() {

	if(UCSR0A & (1<<RXC0)) {	// something received
		return 0;
	} else {
		return 1;
	}

}

unsigned char usart0Receive() {

	unsigned int i=0;

	while(usart0InputBufferEmpty()) {
		i++;
		if(i>150) {
			/*
			if(UCSR0A & (1<<3)) {	// overflow flag
			}
			*/
			commError = 1;
			return 0;				// timeout
		}
	}								// wait for data to be received

	return UDR0;					// get and return received data from buffer

}

// The following usart0 rx isr has to be used with aseba.
ISR(USART0_RX_vect) {
	byteCount++;
	if(byteCount <= UART_BUFF_SIZE) {
		uartBuff[nextByteIndex] = UDR0;
		nextByteIndex++;
		if(nextByteIndex==UART_BUFF_SIZE) {
			nextByteIndex=0;
		}
	}
}


// The following usart0 rx isr was used to control the robo during prototyping; now the radio 
// substitutes completely the usart for this purpose.
// Commands available at the moment through usart:
//
// Menu:
// 0 => red led
// 1 => green led
// 2 => blue led
// 3 => right motor
// 4 => left motor
// 5 => send adc values
//
// Once one of this options is choosen it's possible to control the related peripheral:
// for options 0,1,2 there are the commands "+" and "-" that increase or decrease the luminosity of the leds
// for options 3,4 there are the commmands "+", "-" and "s" to increase, decrease and reset to zero the speed respectively
// for options 5 there is the command "s" that stop the sending of the adc values
// if a key is pressed that do not correspond to any commands for that option, then the initial menu is entered.
/*
ISR(USART0_RX_vect) {

	char receivedByte = UDR0;

	if(choosePeripheral) {						// menu
		switch(receivedByte) {
			case '0': 							// red led
				peripheralChoice = 0;
				choosePeripheral = 0;
				break;
			case '1': 							// green led
				peripheralChoice = 1;
				choosePeripheral = 0;
				break;
			case '2': 							// blue led
				peripheralChoice = 2;
				choosePeripheral = 0;
				break;
			case '3': 							// right motor
				peripheralChoice = 3;
				choosePeripheral = 0;
				break;
			case '4': 							// left motor
				peripheralChoice = 4;
				choosePeripheral = 0;
				break;
			case '5':							// send adc values
				peripheralChoice = 5;
				choosePeripheral = 0;
				sendAdcValues = 1;
				break;
			default:
				break;				 
		}

	} else {									// commands availables for the menu option
	
		int current_pwm=0;

		switch(peripheralChoice) {
			case 0:								// red led
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
			case 1:								// green led
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
			case 2: 							// blue led
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
			case 3: 								// right motor
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
						OCR3A = (int)pwm_right;		// set the new value for the output compares here
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
			case 4: 								// left motor
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
			case 5: 								// adc
				if(receivedByte == 's') {
					sendAdcValues = 0;
					choosePeripheral = 1;
				}

		}		

	}



}
*/
