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

/*! \file
 * \ingroup motor_LED
 * \brief Manage the IR receiver module (timer2)
 *
 * This module manage the IR receiver with the agenda solution (timer2).
 *
 * Alittle exemple to manage the IR remote (the body LED change his state when 
 * you press a button of the IR controller).
 * \code
 * #include <p30f6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/advance_one_timer/e_remote_control.h>
 * #include <motor_led/advance_one_timer/e_agenda.h>
 * 
 * int main(void)
 * {
 * 	int ir_check;
 * 	int previous_check = 0;
 * 	e_init_port();
 * 	e_init_remote_control();
 * 	e_start_agendas_processing();
 * 	while(1)
 * 	{
 * 		ir_check = e_get_check();
 * 		if(ir_check != previous_check)
 * 			BODY_LED = BODY_LED^1;
 * 		previous_check = ir_check;
 * 	}
 * }
 * \endcode
 * \sa e_agenda.h
 * \author Code: Francesco Mondada, Lucas Meier \n Doc: Jonathan Besuchet
 */

//#include <p24FJ64GA104.h>
#include "e_remote_control.h"
//#include "e_agenda.h"
//#include <ports.h>
//#include <outcompare.h>
#include <avr\io.h>
#include <avr\interrupt.h>


/*------ internal variables ------*/

static unsigned char address_temp = 0;
static unsigned char data_temp = 0;
static unsigned char check_temp = 0;

unsigned char address = 0;
unsigned char data_ir = 0;
unsigned char check = 2;

unsigned char my_toggle_state3 = 0;
extern unsigned char command_received;
extern unsigned char blinkState;


/*! \brief Initialise the IR receiver ports */
void e_init_remote_control(void) // initialisation for IR interruptions on INT0
{

	PCICR |= (1 << PCIE1);		// enable interrupt on change of PCINT15:8 pins
	PCMSK1 |= (1 << PCINT15);	// enable PCINT15
/*
	//CloseINT0();
	//ConfigINT0(FALLING_EDGE_INT | INT_ENABLE | INT_PRI_6);	// high priority
	INTCON2bits.INT0EP = 1;      //set interrupt polarity to falling edge
	IFS0bits.INT0IF = 0;      //clear to enable interrupt
	IEC0bits.INT0IE = 1;      //enable interrupt on INT0  
	return;
*/

	TCCR2A |= (1 << WGM01); 	// mode 2 => CTC mode

}

ISR(PCINT1_vect) {


//void __attribute__((__interrupt__, auto_psv))
// _INT0Interrupt(void) // interrupt for IR receiver
//{

//		my_toggle_state3 = 1-my_toggle_state3;
//		
//		if(my_toggle_state3) {
//			SetDCOC3PWM_GB(0,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		} else {
//			SetDCOC3PWM_GB(64,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		}


	if(bit_is_clear(PINJ, 6)) {

	PORTB ^= (1 << 5);

//		PORTB &= ~(1 << 5);

/*
		blinkState = 1 - blinkState;

		if(blinkState) {
			TCCR1A |= (1 << COM1C1);	// enable OCC
			OCR1C = 255;
		} else {
			TCCR1A &= ~(1 << COM1C1);
			PORTB &= ~(1 << 7);
		}
*/
		//IEC0bits.INT0IE = 0;   			//disable interrup from falling edge
		PCICR &= ~(1 << PCIE1);
		PCMSK1 &= ~(1 << PCINT15);
	//	e_set_led(1,1);
		
		//e_activate_agenda(e_read_remote_control, 20); //activate the IR Receiver agenda with a 2.1[ms] cycle value
		// we set the resolution of the timer to be:
		// 0.128 ms (prescaler 1/1024): 1/(8000000/1024) = 0.000128
		// 0.032 ms (prescaler 1/256): 1/(8000000/256) = 0.000032
		// we need 2 ms of delay:
		// 2/0.128 = 15.6
		//OCR2A = 16;
		//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
		// 2/0.032 = 63 => 2.016 ms
		OCR2A = 64;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);		
				
//		PORTB |= (1 << 5);

		check_temp = address_temp = data_temp = 0;
		return;
	}
}

/*! \brief Read the signal and stock the information */
void e_read_remote_control(void) // interrupt from timer for next bits
{
	static int i = -1;
		
//	PORTB ^= (1 << 6);

	if (i == -1)	// start bit confirm  change timer period
	{

//		my_toggle_state3 = 1-my_toggle_state3;
//		
//		if(my_toggle_state3) {
//			SetDCOC3PWM_GB(0,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		} else {
//			SetDCOC3PWM_GB(64,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		}

		if(REMOTE){
		//if(bit_is_set(PINJ,6)) {
			//if high it is only a noise
				//IEC0bits.INT0IE = 1;   	//enable interrupt from falling edge
				//IFS0bits.INT0IF = 0;    //clear interrupt flag from first receive !
				PCICR |= (1 << PCIE1);
				PCMSK1 |= (1 << PCINT15);
				//e_destroy_agenda(e_read_remote_control);
				i = -1;
			}
		else			   // read the check bit
			{
				//e_set_agenda_cycle(e_read_remote_control, 6); //cycle value is 0.6 to go to check bit[ms]
				// we need a delay of 0.6 ms: 0.6 / 0.128 = 4.6
				//OCR2A = 5;
				//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
				// 0.6/0.032 = 19 => 0.608
				OCR2A = 28;
				TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
				TIMSK2 |= (1 << OCIE2A);									

				check_temp = address_temp = data_temp = 0;
				//e_set_led(1,1);
				i=0;
			}
	} 	
//	e_set_led(2,1);
	
	else if (i == 1)	// check bit read and change timer period
	{

//		my_toggle_state3 = 1-my_toggle_state3;
//		
//		if(my_toggle_state3) {
//			SetDCOC3PWM_GB(0,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		} else {
//			SetDCOC3PWM_GB(64,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		}

//	e_set_led(3,1);
		check_temp = REMOTE;	   // read the check bit
		//e_set_agenda_cycle(e_read_remote_control, 18); //cycle value is 1.778[ms]
		// we need a delay of 0.6 ms: 1.8 / 0.128 = 14
		//OCR2A = 14;
		//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
		// 1.8/0.032 = 56 => 1.792
		OCR2A = 53;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);

		//e_set_led(1,1);
	} 
	else if ((i > 1) && (i < 7)) // we read address
	{
//		my_toggle_state3 = 1-my_toggle_state3;
//		
//		if(my_toggle_state3) {
//			SetDCOC3PWM_GB(0,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		} else {
//			SetDCOC3PWM_GB(64,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		}

//	e_set_led(4,1);
		
		//OCR2A = 14;
		//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
		// 1.8/0.032 = 56
		OCR2A = 55;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);

		unsigned char temp = REMOTE;
		temp <<= 6-i;
		address_temp += temp;
	}
	else if ((i > 6) && (i < 13 )) // we read data
	{
//		my_toggle_state3 = 1-my_toggle_state3;
//		
//		if(my_toggle_state3) {
//			SetDCOC3PWM_GB(0,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		} else {
//			SetDCOC3PWM_GB(64,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		}

//			e_set_led(5,1);

		//OCR2A = 14;
		//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
		// 1.8/0.032 = 56
		OCR2A = 54;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);

		unsigned char temp = REMOTE;
		temp <<= 6+6-i;
		data_temp += temp;
	}
	
	else if (i == 13) // last bit read
	{
//		my_toggle_state3 = 1-my_toggle_state3;
//		
//		if(my_toggle_state3) {
//			SetDCOC3PWM_GB(0,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		} else {
//			SetDCOC3PWM_GB(64,64);
//			SetDCOC4PWM_GB(64,64);
//			SetDCOC5PWM_GB(64,64);
//		}

		//e_set_led(1,0);
		//IEC0bits.INT0IE = 1;   	//enable interrupt from falling edge
		//IFS0bits.INT0IF = 0;    //clear interrupt flag from first receive !
		PCICR |= (1 << PCIE1);
		PCMSK1 |= (1 << PCINT15);
		//e_destroy_agenda(e_read_remote_control);
		i = -1;
		check = check_temp;
		address = address_temp;
		data_ir = data_temp;
		command_received=1;
	} 
	
	if(i!=-1)
		i++;	
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
//void __attribute__((interrupt, auto_psv))
// _T4Interrupt(void)
//{

	static int i = -1;

//	PORTB ^= (1 << 5);

	PORTB &= ~(1 << 5);

	TCCR2B &= ~(1 << CS22) &~(1 << CS21) &~(1 << CS20);

	//e_read_remote_control();
		
//	PORTB ^= (1 << 6);

	if (i == -1)	// start bit confirm  change timer period
	{

		if(REMOTE) {	//if high it is only a noise

			PCICR |= (1 << PCIE1);		// enable interrupt
			PCMSK1 |= (1 << PCINT15);	// clear interrupt flag
			i = -1;
		} else {	   // read the check bit
			
			//cycle value is 0.6 to go to check bit[ms]
			// we need a delay of 0.6 ms: 0.6 / 0.128 = 4.6
			//OCR2A = 5;
			//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
			// 0.6/0.032 = 19 => 0.608
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
		// 1.8/0.032 = 56 => 1.792
		OCR2A = 53;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);

	} else if ((i > 1) && (i < 7)) { // we read address
		
		//OCR2A = 14;
		//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
		// 1.8/0.032 = 56
		OCR2A = 55;
		TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler
		TIMSK2 |= (1 << OCIE2A);

		unsigned char temp = REMOTE;
		temp <<= 6-i;
		address_temp += temp;

	} else if ((i > 6) && (i < 13 )) { // we read data

		//OCR2A = 14;
		//TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);		// 1/1024 prescaler
		// 1.8/0.032 = 56
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
	} 
	
	if(i!=-1)
		i++;

	PORTB |= (1 << 5);

}
