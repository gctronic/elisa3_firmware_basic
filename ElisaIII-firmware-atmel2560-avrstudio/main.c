

#include <avr\io.h>
#include <avr\interrupt.h>

#include "spi.h"
#include "mirf.h"
#include "nRF24L01.h"
#include "e_agenda.h"
#include "e_remote_control.h"

volatile unsigned char currentAdChannel = 0;
// channel 0..6:  prox0..6
// channel 7:	  prox7/battery
// channel 8..11: cliff0..3
// channel 12:	  active phase when going backward: motor right current; passive phase when going forward: motor right velocity 
// channel 13.	  active phase when going forward: motor right current; passive phase when going backward: motor right velocity
// channel 14:    active phase when going backward: motor left current; passive phase when going forward: motor left velocity 
// channel 15:    active phase when going forward: motor left current; passive phase when going backward: motor left velocity
// The channel selection sequence is:
unsigned char channelSequence[120] = 
{0, 12, 14, 13, 15, 0, 12, 14, 13, 15, 1, 12, 14, 13, 15, 1, 12, 14, 13, 15, 2, 12, 14, 13, 15, 2, 12, 14, 13, 15, 3, 12, 14, 13, 15,
3, 12, 14, 13, 15, 4, 12, 14, 13, 15, 4, 12, 14, 13, 15, 5, 12, 14, 13, 15, 5, 12, 14, 13, 15, 6, 12, 14, 13, 15, 6, 12, 14, 13, 15, 
7, 12, 14, 13, 15, 7, 12, 14, 13, 15, 8, 12, 14, 13, 15, 8, 12, 14, 13, 15, 9, 12, 14, 13, 15, 9, 12, 14, 13, 15, 10, 12, 14, 13, 15, 
10, 12, 14, 13, 15, 11, 12, 14, 13, 15, 11, 12, 14, 13, 15};
unsigned char channelIndex = 0; 

unsigned char currentProx = 0;
unsigned char currentMotLeftChannel = 0;
unsigned char currentMotRightChannel = 0;
unsigned char rightMotorPhase = 0;
unsigned char leftMotorPhase = 0;
#define activePhase 0
#define passivePhase 1
volatile unsigned int proximityValue[24] = {0};
volatile unsigned int cliffValue[8] = {0};
unsigned char adcSaveDataTo = 0;
#define saveToProx 				0
#define saveToCliff 			1
#define saveToRightMotorCurrent 2
#define saveToRightMotorVel		3
#define saveToLeftMotorCurrent	4
#define saveToLeftMotorVel		5
unsigned char adcSamplingState = 0;

//--Current consumption controller--//
char start_left_current_sampling = 0;				// set at the beginning of the motors period
char start_right_current_sampling = 0;
unsigned int left_current_avg = 0;					// the left motor current consumption is sampled from 0 to duty cycle at each motor period; 
unsigned int right_current_avg = 0;					// the average of all these samples is given as the new current consumption
unsigned int last_left_current = 0;					// current consumption of the previous/last period
unsigned int last_right_current = 0;

//--Speed controller--//
char start_left_vel_sampling = 0;						// set at the end of duty cycle
char start_right_vel_sampling = 0;
volatile unsigned int num_lvel_samples_avg = 0;			// current number of samples received for filtering (average) the left velocity value
volatile unsigned int last_num_lvel_samples_avg = 0;
volatile unsigned int num_rvel_samples_avg = 0;			// current number of samples received for filtering (average) the right velocity value
volatile unsigned int last_num_rvel_samples_avg = 0;
unsigned int left_vel_sum = 0;							// sum of all the adc values (at the end they will be divided by the total number of samples received)
unsigned int last_left_vel_sum = 0;
unsigned int right_vel_sum = 0;
unsigned int last_right_vel_sum = 0;
signed long int pwm_right = 0;							// last pwm values before update; this pwm is expressed in the range 0..PERIOD MOTORS
signed long int pwm_left = 0;


//--nRF--//
#define CHANGE_STATE 0x0
#define CHANGE_RF 0x1
unsigned int dataLED[3];
unsigned char speedl=0, speedr=0;
uint8_t rfData[PAYLOAD_SIZE];

unsigned char peripheralChoice = 5;	// red led=0, green led=1, blue led=2, right motor=3, left motor=4
unsigned char choosePeripheral = 1;
unsigned char pwm_red = 255, pwm_green = 255, pwm_blue = 255;
unsigned char sendAdcValues = 0;

unsigned char blinkState = 0;

//--IR Remote control--//
unsigned char ir_move = 0;
extern unsigned char data_ir;
extern unsigned char command_received;

unsigned char colorState = 0;

#define STEP_MOTORS 100
#define MAX_MOTORS_PWM 1023
#define MAX_LEDS_PWM 255

FUSES = {
	.low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),	// internal 8MHz clock, no divisor
	.high = (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_SPIEN),				// jtag disabled
	.extended = EFUSE_DEFAULT,
};


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


void initPorts(void) {

	DDRA = 0xFF;	// proximity pulses as output
	PORTA = 0x00;	// proximity pulses turned off
	
	DDRB = 0xF7;	// pwm for led r/g/b as output; CE, MOSI, SCK, SS as output (master) 
	PORTB |= (1 << 5) | (1 << 6) | (1 << 7); // leds off	
	//PORTB &= ~(1 << 5) & ~(1 << 6) & ~(1 << 7); // leds off
	//PORTB &= ~(1 << 7);

	DDRC = 0xF0;	// selector as input; IR leds as output; sens-enable, sleep as output
	PORTC = 0xB0;	// sleep = 1, IR leds = 1

	DDRD = 0x00;	// all pins to input; when usart and i2c peripherals are activated they change the pins direction accordingly

	DDRE = 0x18;	// pwm and dir for motor right as output; when usart is activated it changes the pins direction accordingly

	DDRF = 0x00;	// adc channel pins as input
	
	DDRG = 0x00;	// unused pins as input
	
	DDRH = 0x58;	// pwm and dir for motor left as output; when usart is activated it changes the pins direction accordingly
		
	DDRJ = 0x0F;	// cliff pulses as output; charge-on, button0, remote as input

	DDRK = 0x00;	// adc channel pins as input

	DDRL = 0x08;	// output compare for pwm as output
		
}

void initAdc(void) {

	// ADCSRA -----> ADEN	ADSC	 ADATE	ADIF	ADIE	 ADPS2	ADPS1	ADPS0
	//				 0		0		 0		0		0		 0		0		0
	// ADMUX  -----> REFS1	REFS0	 ADLAR	MUX4	MUX3 	 MUX2	MUX1	MUX0
	//				 0		0		 0		0		0		 0		0		0
	// ADCSRB -----> -		ACME	 - 		- 		MUX5 	 ADTS2 	ADTS1 	ADTS0
	//				 0		0		 0		0		0		 0		0		0

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	// 1/64 prescaler => 8 MHz / 64 = 125 KHz
	ADMUX |= (1 << REFS0); 	// voltage reference to AVCC (external)
	ADCSRA |= (1 << ADATE); // auto-trigger mode
	//ADRTS2:0 in ADCSRB  are already set to free running by default (0b000)
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
		
	//PORTB |= (1 << 7);

	int value = 0;
	value = ADCL;			// must be read first!!
	value = (ADCH<<8) | value;

	//proximityValue[currentProx] = value;
	/*
	if(rightMotorPhase == passivePhase) {
		right_vel_sum += value;
		num_rvel_samples_avg++;
	}
	*/


	// save the last data
	switch(adcSaveDataTo) {

		case saveToProx:	
			proximityValue[currentProx] = value;
			currentProx = (currentProx+1)%24;
			break;

		//case saveToCliff:	
		//	
		//	break;

		case saveToRightMotorCurrent:
			right_current_avg = value;
			right_current_avg = right_current_avg >> 1;
			break;

		case saveToRightMotorVel:

			break;

		case saveToLeftMotorCurrent:
			left_current_avg = value;
			left_current_avg = left_current_avg >> 1;
			break;

		case saveToLeftMotorVel:

			break;

	}

	// select next channel
	// ...the sequence has to be changed in order to satisfy motors sampling requirements!
	switch(adcSamplingState) {

		case 0:
			currentAdChannel = currentProx;
			if(rightMotorPhase == activePhase) {			// the first this isn't really correct
				adcSaveDataTo = saveToRightMotorCurrent;
			} else {
				adcSaveDataTo = saveToRightMotorVel;
			}
			break;

		case 1:
			currentAdChannel = currentMotLeftChannel;
			adcSaveDataTo = saveToProx;
			break;

		case 2:
			currentAdChannel = currentMotRightChannel;
			if(leftMotorPhase == activePhase) {
				adcSaveDataTo = saveToLeftMotorCurrent;
			} else {
				adcSaveDataTo = saveToLeftMotorVel;
			}
			break;

		case 3:
			currentAdChannel = currentMotLeftChannel;
			if(rightMotorPhase == activePhase) {
				adcSaveDataTo = saveToRightMotorCurrent;
			} else {
				adcSaveDataTo = saveToRightMotorVel;
			}
			break;

		case 4:
			currentAdChannel = currentMotRightChannel;		
			if(leftMotorPhase == activePhase) {
				adcSaveDataTo = saveToLeftMotorCurrent;
			} else {
				adcSaveDataTo = saveToLeftMotorVel;
			}
			break;

	}
	adcSamplingState = (adcSamplingState+1)%5;

	// channel selection: continuously change the channel sampled in sequence
	if(currentAdChannel < 8) {
		ADCSRB &= ~(1 << MUX5);
		ADMUX |= currentAdChannel;
	} else {
		ADCSRB |= (1 << MUX5);
		ADMUX |= currentAdChannel-8;
	}


	/*
	switch(currentAdChannel) {

		case 0:	 
		case 1:	
		case 2: 
		case 3:	
		case 4:	
		case 5: 
		case 6:		// proximity 0..6
					proximityValue[currentAdChannel] = value;
					break;

		case 7:		// proximity 7/battery
					proximityValue[currentAdChannel] = value;
					break;

		case 8: 
		case 9:	 
		case 10:	
		case 11:	// cliff 0..3 
					cliffValue[currentAdChannel-8] = value;
					break;

		case 12:	// passive phase + forward => motor right velocity; active + backward => motor right current
					if(OCF3A || OCF3B) {	// passive phase (timer counter and output compares matches)
						TIFR3 |= (1 << OCF3B) | (1 << OCF3A);
						start_right_current_sampling = 0;	// clear flags
					}
					if(start_right_current_sampling) {
						if(pwm_right < 0) {	// backward
							right_current_avg = value;
						}
						right_current_avg = right_current_avg >> 1;
					} else {	// velocity sampling
						
					}
					break;

		case 13:	// active phase + forward => motor right current; passive + backward => motor right velocity
					break;

		case 14: 
					break;

		case 15:	 
					break;

	}
	*/


	//PORTB &= ~(1 << 7);

}


void initPwm() {


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

	// copy sampling variables
	// reset sampling variables
	leftMotorPhase = activePhase;
	last_left_current = left_current_avg;
	left_current_avg = 0;
	// start control

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
	leftMotorPhase = passivePhase;
	// select channel 14 to sample the left velocity
	currentMotLeftChannel = 14;
}

// motor left backward
ISR(TIMER4_COMPB_vect) {
	leftMotorPhase = passivePhase;
	// select channel 15 to sample the left velocity
	currentMotLeftChannel = 15;
}

// Motor right
ISR(TIMER3_OVF_vect) {

	// copy sampling variables
	// reset sampling variables
	rightMotorPhase = activePhase;
	last_right_current = right_current_avg;
	right_current_avg = 0;

	last_right_vel_sum = right_vel_sum;
	last_num_rvel_samples_avg = num_rvel_samples_avg;
	right_vel_sum = 0;
	num_rvel_samples_avg = 0;

	sendAdcValues = 1;

	// start control

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
	rightMotorPhase = passivePhase;
	// select channel 12 to sample the right velocity
	currentMotRightChannel = 12;
}

// motor right backward
ISR(TIMER3_COMPB_vect) {
	rightMotorPhase = passivePhase;
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

void initUsart() {

	// clock is 8 MHz, thus:
	// @9600 baud: 8000000/16/9600-1 = 51 => 8000000/16/52 = 9615 => 100-(9600/9615*100)=0.15% of error

	// set baudrate
	UBRR0H = 0;
	UBRR0L = 51;

	UCSR0A  &= ~(1 << U2X0);								// disable double transmission speed

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

void initExternalInterrupt() {
	
	
	
}


void initPeripherals(void) {

	cli();			// disable global interrupts (by default it should already be disabled)

	initPorts();
	initAdc();
	initPwm();
	initSPI();
	mirf_init();
	initUsart();
	initExternalInterrupt();

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

int main(void) {

	unsigned char debugData = 0xAA;
	unsigned int i = 0;
	unsigned char packetId = 0;
	choosePeripheral = 1;
	channelIndex = 0;

	initPeripherals();

	e_start_agendas_processing();
	//e_activate_agenda(toggleBlueLed, 10000);		// every 1 seconds
	e_init_remote_control();

/*
	currentAdChannel = 12;
	// channel selection: continuously change the channel sampled in sequence
	if(currentAdChannel < 8) {
		ADCSRB &= ~(1 << MUX5);
		ADMUX |= currentAdChannel;
	} else {
		ADCSRB |= (1 << MUX5);
		ADMUX |= currentAdChannel-8;
	}
*/

	//usartTransmit(debugData);				

	while(1) {

		//PORTB ^= (1 << 6); // Toggle the green LED

		// test ok
		//if(TCNT3 >= 2000) {
		//	PORTB |= 0x80;
		//} else {
		//	PORTB &= 0x7F;	
		//}

		// test ok
		//if((TIFR3&0x01) == 1) {
		//	PORTB |= 0x80;
		//} else {
		//	PORTB &= 0x7F;	
		//}


		ir_move = e_get_data();

		if(command_received) {

			command_received = 0;

			//usartTransmit(ir_move);


			switch(ir_move) {

				case 5:	// stop motors
					pwm_right = 0;
					pwm_left = 0;
					break;

				case 2:	// both motors forward
					if(pwm_right > pwm_left) {
						pwm_left = pwm_right;
					} else {
						pwm_right = pwm_left;
					}
					pwm_right += STEP_MOTORS;
					pwm_left += STEP_MOTORS;
	                if (pwm_right > MAX_MOTORS_PWM) pwm_right = MAX_MOTORS_PWM;
    	            if (pwm_left > MAX_MOTORS_PWM) pwm_left = MAX_MOTORS_PWM;
               		break;

				case 8:	// both motors backward
					if(pwm_right < pwm_left) {
						pwm_left  = pwm_right;
					} else {
						pwm_right = pwm_left;
					}
					pwm_right -= STEP_MOTORS;
					pwm_left -= STEP_MOTORS;
	                if (pwm_right < -MAX_MOTORS_PWM) pwm_right = -MAX_MOTORS_PWM;
    	            if (pwm_left < -MAX_MOTORS_PWM) pwm_left = -MAX_MOTORS_PWM;
                  	break;

				case 6:	// both motors right
					pwm_right -= STEP_MOTORS;
					pwm_left += STEP_MOTORS;
                	if (pwm_right<-MAX_MOTORS_PWM) pwm_right=-MAX_MOTORS_PWM;
                	if (pwm_left>MAX_MOTORS_PWM) pwm_left=MAX_MOTORS_PWM;
					break;

				case 4:	// both motors left
					pwm_right += STEP_MOTORS;
					pwm_left -= STEP_MOTORS;
	                if (pwm_right>MAX_MOTORS_PWM) pwm_right=MAX_MOTORS_PWM;
    	            if (pwm_left<-MAX_MOTORS_PWM) pwm_left=-MAX_MOTORS_PWM;
					break;

				case 3:	// left motor forward
					pwm_left += STEP_MOTORS;
                	if (pwm_left>MAX_MOTORS_PWM) pwm_left=MAX_MOTORS_PWM;
					break;

				case 1:	// right motor forward
					pwm_right += STEP_MOTORS;
	                if (pwm_right>MAX_MOTORS_PWM) pwm_right=MAX_MOTORS_PWM;
					break;

				case 9:	// left motor backward
					pwm_left -= STEP_MOTORS;
            	    if (pwm_left<-MAX_MOTORS_PWM) pwm_left=-MAX_MOTORS_PWM;
					break;

				case 7:	// right motor backward
					pwm_right -= STEP_MOTORS;
                	if (pwm_right<-MAX_MOTORS_PWM) pwm_right=-MAX_MOTORS_PWM;
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

		}	// ir command received

/*
		if(sendAdcValues) {
		
			sendAdcValues = 0;
			
			last_right_vel_sum = (unsigned int)(last_right_vel_sum/last_num_rvel_samples_avg);

			usartTransmit(0xAA);
			usartTransmit(0xAA);
			//usartTransmit(last_left_current&0xFF);
			//usartTransmit(last_left_current>>8);
			//usartTransmit(last_right_current&0xFF);
			//usartTransmit(last_right_current>>8);
			//usartTransmit((unsigned char)(proximityValue[currentProx]&0xFF));
			//usartTransmit((unsigned char)(proximityValue[currentProx]>>8));
			usartTransmit((unsigned char)(last_right_vel_sum&0xFF));
			usartTransmit((unsigned char)(last_right_vel_sum>>8));
			usartTransmit((unsigned char)(last_num_rvel_samples_avg&0xFF));
			usartTransmit((unsigned char)(last_num_rvel_samples_avg>>8));

		}
*/


		if(!rx_fifo_is_empty()) {

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
				pwm_right = speedr<<2;				// scale the speed received (0..100) to be in the range 0..400
			} else {								// backward
				pwm_right = -(speedr<<2);
			}

			if((rfData[5]&0x80)==0x80) {			// motor left forward
				pwm_left = speedl<<2;		
			} else {								// backward
				pwm_left = -(speedl<<2);
			}
			
			if (pwm_right>(MAX_MOTORS_PWM)) pwm_right=(MAX_MOTORS_PWM);
            if (pwm_left>(MAX_MOTORS_PWM)) pwm_left=(MAX_MOTORS_PWM);
            if (pwm_right<-(MAX_MOTORS_PWM)) pwm_right=-(MAX_MOTORS_PWM);
            if (pwm_left<-(MAX_MOTORS_PWM)) pwm_left=-(MAX_MOTORS_PWM);
		

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
			writeAckPayload(&packetId, 1);
#endif


		}


	}

}
