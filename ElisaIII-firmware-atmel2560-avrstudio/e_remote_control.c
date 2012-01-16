/********************************************************************************

			Control IR receiver module							
			December 2005: first version							
			Valentin Longchamp 


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Valentin Longchamp

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch
**********************************************************************************/
/*
Taken from the e-puck library and adapted to work with Atmel microprocessor and 
without "Agenda"; it uses Timer2 for timing the reading of the signal
*/


#include "e_remote_control.h"
#include <avr\io.h>
#include <avr\interrupt.h>


/*------ internal variables ------*/
static unsigned char address_temp = 0;
static unsigned char data_temp = 0;
static unsigned char check_temp = 0;
unsigned char address = 0;
unsigned char data_ir = 0;
unsigned char check = 2;

/* external variables */
extern unsigned char command_received;
extern unsigned char checkGlitch;


/*! \brief Initialise the IR receiver ports */
void e_init_remote_control(void) { 	// initialisation for IR interruptions on PCINT1 (external interrupt)

	PCICR |= (1 << PCIE1);		// enable interrupt on change of PCINT15:8 pins
	PCMSK1 |= (1 << PCINT15);	// enable PCINT15
	TCCR2A |= (1 << WGM01); 	// mode 2 => CTC mode

}

ISR(PCINT1_vect) {


	if(bit_is_clear(PINJ, 6)) {

		//PORTB ^= (1 << 5);
		//PORTB &= ~(1 << 6);
   			
		PCICR &= ~(1 << PCIE1);			// disable interrupt from falling edge
		PCMSK1 &= ~(1 << PCINT15);
		
		// check the pin change isn't due to a glitch; to check this verify that
		// the pin remain low for at least 400 us (the giltches last about 200 us)
		// 0.4 / 0.032 = 13 => 0.416 us
		checkGlitch = 1;
		OCR2A = 13;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);

		check_temp = address_temp = data_temp = 0;
		return;

	}
	
}


/*------ user calls ------*/

/** \brief Read the check bit
 * \return	check	check bit of the signal
 */
unsigned char e_get_check(void) {
	return check;
}

/** \brief Read the adress of the commande
 * \return	adress	adress part of the signal
 */
unsigned char e_get_address(void) {
	return address;
}

/** \brief Read the data of the command
 * \return	data	data part of the signal
 */
unsigned char e_get_data(void) {
	return data_ir;
}

ISR(TIMER2_COMPA_vect) {

	static int i = -1;

	//PORTB ^= (1 << 5);

	TCCR2B &= ~(1 << CS22) &~(1 << CS21) &~(1 << CS20);	// disable timer2
	
	if(checkGlitch) {
		if(REMOTE) {	// if high it is a glitch
			PCICR |= (1 << PCIE1);		// enable external interrupt
			PCMSK1 |= (1 << PCINT15);	// clear interrupt flag
			i = -1;			
		} else {
			checkGlitch = 0;
			// activate the IR Receiver with a 2.1[ms] cycle value
			// we set the resolution of the timer to be:
			// 0.128 ms (prescaler 1/1024): 1/(8000000/1024) = 0.000128
			// 0.032 ms (prescaler 1/256): 1/(8000000/256) = 0.000032
			// we need 2 ms of delay:
			// 2/0.128 = 15.6
			//OCR2A = 16;
			//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
			// 2.1/0.032 = 64 => 2.048 ms
			// but we already wait 0.416 us => 13, so 64-13=51
			OCR2A = 51;
			TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
			TIMSK2 |= (1 << OCIE2A);

		}
	} else {


		if (i == -1) { // start bit confirm  change timer period

			if(REMOTE) {	//if high it is only a noise

				PCICR |= (1 << PCIE1);		// enable external interrupt
				PCMSK1 |= (1 << PCINT15);	// clear interrupt flag
				i = -1;
			
				//PORTB |= (1 << 6);

			} else {	   // read the check bit
			
				//cycle value is 0.6 to go to check bit[ms]
				// we need a delay of 0.6 ms: 0.6 / 0.128 = 4.6
				//OCR2A = 5;
				//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
				// 0.9/0.032 = 28 => 0.896
				OCR2A = 28;
				TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
				TIMSK2 |= (1 << OCIE2A);									

				check_temp = address_temp = data_temp = 0;
				i=0;
			}
		} else if (i == 1)	{ // check bit read and change timer period

			check_temp = REMOTE;	   // read the check bit
			//cycle value is 1.778[ms]
			// we need a delay of 0.6 ms: 1.8 / 0.128 = 14
			//OCR2A = 14;
			//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
			// 1.778/0.032 = 54 => 1.728
			OCR2A = 54;
			TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
			TIMSK2 |= (1 << OCIE2A);

		} else if ((i > 1) && (i < 7)) { // we read address
		
			//OCR2A = 14;
			//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
			// 1.778/0.032 = 54
			OCR2A = 54;
			TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
			TIMSK2 |= (1 << OCIE2A);

			unsigned char temp = REMOTE;
			temp <<= 6-i;
			address_temp += temp;

		} else if ((i > 6) && (i < 13 )) { // we read data

			//OCR2A = 14;
			//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
			// 1.778/0.032 = 54
			OCR2A = 54;
			TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
			TIMSK2 |= (1 << OCIE2A);

			unsigned char temp = REMOTE;
			temp <<= 6+6-i;
			data_temp += temp;

		} else if (i == 13) { // last bit read

			PCICR |= (1 << PCIE1);		// enable interrupt
			PCMSK1 |= (1 << PCINT15);	// clear interrupt flag

			i = -1;
			check = check_temp;
			address = address_temp;
			data_ir = data_temp;
			command_received=1;

			//PORTB |= (1 << 6);
		} 

	}
	
	if(i!=-1)
		i++;


}
