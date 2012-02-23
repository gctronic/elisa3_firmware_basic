
#include "ir_remote_control.h"

// internal variables
static unsigned char address_temp = 0;
static unsigned char data_temp = 0;
static unsigned char check_temp = 0;
unsigned char address = 0;
unsigned char data_ir = 0;
unsigned char check = 2;

void init_ir_remote_control(void) { 	

	PCICR |= (1 << PCIE1);			// enable interrupt on change of PCINT15:8 pins
	PCMSK1 |= (1 << PCINT15);		// enable PCINT15
	TCCR2A |= (1 << WGM21); 		// mode 2 => CTC mode

}

// external interrupt service routine
ISR(PCINT1_vect) {

	if(irEnabled) {						// if the robot is configured to accept TV remote commands

		if(bit_is_clear(PINJ, 6)) {		// the interrupt is generated at every pin state change; we only look
										// for the falling edge
			PCICR &= ~(1 << PCIE1);		// disable external interrupt
			PCMSK1 &= ~(1 << PCINT15);
		
			// check the pin change isn't due to a glitch; to check this verify that
			// the pin remain low for at least 400 us (the giltches last about 200 us)
			// 0.4 / 0.032 = 13 => 0.416 us
			checkGlitch = 1;							// we're checking if this is a glitch
			OCR2A = 13;									// output compare register
			TCCR2B |= (1 << CS22) | (1 << CS21);		// 1/256 prescaler => 8 MHz / 256 = 31.25 KHz (32 us resolution)
			TIMSK2 |= (1 << OCIE2A);					// enable output compare interrupt

			check_temp = address_temp = data_temp = 0;
			return;

		}

	}
	
}

ISR(TIMER2_COMPA_vect) {

	static int i = -1;

	//PORTB ^= (1 << 5);	// toggle red led

	TCCR2B &= ~(1 << CS22) &~(1 << CS21) &~(1 << CS20);		// stop timer2
	
		if(checkGlitch) {					// if checking this is a glitch

			if(REMOTE) {					// if high it is a glitch

				PCICR |= (1 << PCIE1);		// re-enable external interrupt to receive the next command
				PCMSK1 |= (1 << PCINT15);	// clear interrupt flag
				i = -1;			

			} else {						// not a glitch => real command received

				checkGlitch = 0;

				// activate the IR Receiver with a 2.1 ms cycle value
				// we set the resolution of the timer to be 0.032 ms (prescaler 1/256) so:
				// 2.1 / 0.032 = 64 to be set in the output compare register (=> 2.048 ms)
				// but we already wait 0.416 us => 13, so 64-13=51
				OCR2A = 51;								// output compare register
				TCCR2B |= (1 << CS22) | (1 << CS21);	// 1/256 prescaler => 8 MHz / 256 = 31.25 KHz (32 us resolution)
				TIMSK2 |= (1 << OCIE2A);				// enable output compare interrupt

			}

		} else {


			if (i == -1) { 						// start bit confirmed

				if(REMOTE) {					// double check => if high it is only a noise

					PCICR |= (1 << PCIE1);		// re-enable external interrupt to receive the next command
					PCMSK1 |= (1 << PCINT15);	// clear interrupt flag
					i = -1;

				} else {	// read the check bit
			
					//cycle value is 0.9 ms to go to check bit so:
					// 0.9 / 0.032 = 28 => 0.896
					OCR2A = 28;								// output compare register
					TCCR2B |= (1 << CS22) | (1 << CS21);	// 1/256 prescaler
					TIMSK2 |= (1 << OCIE2A);				// enable output compare interrupt					

					check_temp = address_temp = data_temp = 0;
					i=0;

				}

			} else if (i == 1)	{ 						// check bit read and change timer period

				check_temp = REMOTE;	   				// read the check bit
				//cycle value is 1.778 ms => 1.778 / 0.032 = 54 (=> 1.728 ms)
				OCR2A = 54;								// output compare register
				TCCR2B |= (1 << CS22) | (1 << CS21);	// 1/256 prescaler
				TIMSK2 |= (1 << OCIE2A);				// enable output compare interrupt

			} else if ((i > 1) && (i < 7)) {			// we read address
		
				OCR2A = 54;
				TCCR2B |= (1 << CS22) | (1 << CS21);
				TIMSK2 |= (1 << OCIE2A);

				unsigned char temp = REMOTE;
				temp <<= 6-i;
				address_temp += temp;

			} else if ((i > 6) && (i < 13 )) { 			// we read data

				OCR2A = 54;
				TCCR2B |= (1 << CS22) | (1 << CS21);
				TIMSK2 |= (1 << OCIE2A);

				unsigned char temp = REMOTE;
				temp <<= 6+6-i;
				data_temp += temp;

			} else if (i == 13) { 						// last bit read
				
				TIMSK2 = 0;								// disable all interrupt for timer2
				PCICR |= (1 << PCIE1);					// enable external interrupt to receive next command
				PCMSK1 |= (1 << PCINT15);				// clear interrupt flag

				i = -1;
				check = check_temp;
				address = address_temp;
				data_ir = data_temp;
				command_received=1;

			} 

		}
	
		if(i!=-1) {

			i++;

		}

}

unsigned char ir_remote_get_check(void) {
	return check;
}

unsigned char ir_remote_get_address(void) {
	return address;
}

unsigned char ir_remote_get_data(void) {
	return data_ir;
}

void handleIRRemoteCommands() {

	if(irEnabled) {

		irCommand = ir_remote_get_data();

		if(command_received) {

			command_received = 0;

			switch(irCommand) {
				// sometimes there are two cases for the same command because two different
				// remote controls are used; one of this do not contain "numbers"
				case 5:	// stop motors
				case 51:
					pwm_right_desired = 0;
					pwm_left_desired = 0;
					break;

				case 2:	// both motors forward
				case 31:
					if(pwm_right_desired > pwm_left_desired) {
						pwm_left_desired = pwm_right_desired;
					} else {
						pwm_right_desired = pwm_left_desired;
					}
					pwm_right_desired += STEP_MOTORS;
					pwm_left_desired += STEP_MOTORS;
	                if (pwm_right_desired > (MAX_MOTORS_PWM/2)) pwm_right_desired = (MAX_MOTORS_PWM/2);
    	            if (pwm_left_desired > (MAX_MOTORS_PWM/2)) pwm_left_desired = (MAX_MOTORS_PWM/2);
               		break;

				case 8:	// both motors backward
				case 30:
					if(pwm_right_desired < pwm_left) {
						pwm_left_desired  = pwm_right_desired;
					} else {
						pwm_right_desired = pwm_left_desired;
					}
					pwm_right_desired -= STEP_MOTORS;
					pwm_left_desired -= STEP_MOTORS;
	                if (pwm_right_desired < -(MAX_MOTORS_PWM/2)) pwm_right_desired = -(MAX_MOTORS_PWM/2);
    	            if (pwm_left_desired < -(MAX_MOTORS_PWM/2)) pwm_left_desired = -(MAX_MOTORS_PWM/2);
                  	break;

				case 6:	// both motors right
				case 47:
					pwm_right_desired -= STEP_MOTORS;
					pwm_left_desired += STEP_MOTORS;
                	if (pwm_right_desired<-(MAX_MOTORS_PWM/2)) pwm_right_desired=-(MAX_MOTORS_PWM/2);
                	if (pwm_left_desired>(MAX_MOTORS_PWM/2)) pwm_left_desired=(MAX_MOTORS_PWM/2);
					break;

				case 4:	// both motors left
				case 46:
					pwm_right_desired += STEP_MOTORS;
					pwm_left_desired -= STEP_MOTORS;
	                if (pwm_right_desired>(MAX_MOTORS_PWM/2)) pwm_right_desired=(MAX_MOTORS_PWM/2);
	   	            if (pwm_left_desired<-(MAX_MOTORS_PWM/2)) pwm_left_desired=-(MAX_MOTORS_PWM/2);
					break;

				case 3:	// left motor forward
					pwm_left_desired += STEP_MOTORS;
	               	if (pwm_left_desired>(MAX_MOTORS_PWM/2)) pwm_left_desired=(MAX_MOTORS_PWM/2);
					break;

				case 1:	// right motor forward
					pwm_right_desired += STEP_MOTORS;
	                if (pwm_right_desired>(MAX_MOTORS_PWM/2)) pwm_right_desired=(MAX_MOTORS_PWM/2);
					break;

				case 9:	// left motor backward
					pwm_left_desired -= STEP_MOTORS;
	           	    if (pwm_left_desired<-(MAX_MOTORS_PWM/2)) pwm_left_desired=-(MAX_MOTORS_PWM/2);
					break;

				case 7:	// right motor backward
					pwm_right_desired -= STEP_MOTORS;
	               	if (pwm_right_desired<-(MAX_MOTORS_PWM/2)) pwm_right_desired=-(MAX_MOTORS_PWM/2);
					break;

	           	case 0:	// colors
				case 50:
					colorState = (colorState+1)%5;

					if(colorState==0) {			// turn on blue and off all IRs
						LED_IR1_HIGH;
						LED_IR2_HIGH;
						pwm_blue = 0;
						pwm_green = MAX_LEDS_PWM;
						pwm_red = MAX_LEDS_PWM;
					} else if(colorState==1) {	// turn on green
						pwm_blue = MAX_LEDS_PWM;
						pwm_green = 0;
						pwm_red = MAX_LEDS_PWM;
					} else if(colorState==2) {	// turn on red and on all IRs
						LED_IR1_LOW;
						LED_IR2_LOW;
						pwm_blue = MAX_LEDS_PWM;
						pwm_green = MAX_LEDS_PWM;
						pwm_red = 0;
					} else if(colorState==3) {	// turn on white
						pwm_blue = 0;
						pwm_green = 0;
						pwm_red = 0;
					} else if(colorState==4) {	// turn off all leds
						pwm_blue = MAX_LEDS_PWM;
						pwm_green = MAX_LEDS_PWM;
						pwm_red = MAX_LEDS_PWM;
					}

					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);

	               	break;

				case 16:	// volume +
					obstacleAvoidanceEnabled = 1;
					break;

				case 17:	// volume -
					obstacleAvoidanceEnabled = 0;
					break;

				case 32:	// program +
					cliffAvoidanceEnabled = 1;
					break;

				case 33:	// program -
					cliffAvoidanceEnabled = 0;
					break;

				case 52:	// av/tv button
					behaviorState = (behaviorState+1)%4;
					switch(behaviorState) {
						case 0:
							obstacleAvoidanceEnabled = 0;
							cliffAvoidanceEnabled = 0;
							break;
						case 1:
							obstacleAvoidanceEnabled = 1;
							cliffAvoidanceEnabled = 0;
							break;
						case 2:
							obstacleAvoidanceEnabled = 0;
							cliffAvoidanceEnabled = 1;
							break;
						case 3:
							obstacleAvoidanceEnabled = 1;
							cliffAvoidanceEnabled = 1;
							break;

					}
					break;
	    
		       	default:
	               	break;

			}	// switch

			// convert pwm deisred in absolute speed (0 to 100)
			if(pwm_right_desired >= 0) {
				speedr = pwm_right_desired >> 2;
			} else {
				speedr = (-pwm_right_desired) >> 2;
			}
			if(pwm_left_desired >= 0) {
				speedl = pwm_left_desired >> 2;
			} else {
				speedl = (-pwm_left_desired) >> 2;
			}

		}	// ir command received

	}	// ir enabled check


}


