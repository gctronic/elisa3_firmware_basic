
#include <avr\io.h>
#include <avr\interrupt.h>
#include <math.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"


int main(void) {

	choosePeripheral = 1;					// usart menu

	initPeripherals();

	calibrateSensors();

	while(1) {

		//PORTB ^= (1 << 6); 				// toggle the green led

		currentSelector = getSelector();	// update selector position

		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();

/*
		if(delayCounter%(PAUSE_10_MSEC) == 0) {
			// do something every 10 milliseconds
		}
*/

		if(delayCounter >= (PAUSE_2_SEC)) {
			
			delayCounter = 0;
			
			measBattery = 1;
			
/*
			sendAdcValues = 0;

			usartTransmit(0xAA);
			usartTransmit(0xAA);
			for(i=0; i<24; i++) {
				usartTransmit((unsigned char)(proximityValue[i]&0xFF));
				usartTransmit((unsigned char)(proximityValue[i]>>8));
			}
			//usartTransmit(getselector());
			//usartTransmit(getselector());
			usartTransmit(last_right_current&0xFF);
			usartTransmit(last_right_current>>8);
			usartTransmit(last_left_current&0xFF);
			usartTransmit(last_left_current>>8);


			// two possible cases cause the number of samples to be zero:
			// - when the pwm is at its maximum (thus no passive phase)
			// - a missing output compare match interrupt that indicates the start of the passive phase
			usartTransmit((unsigned char)(last_right_vel&0xFF));
			usartTransmit((unsigned char)(last_right_vel>>8));
			usartTransmit((unsigned char)(last_left_vel&0xFF));
			usartTransmit((unsigned char)(last_left_vel>>8));


			readAccelXYZ();
			usartTransmit(accX&0xFF);
			usartTransmit(accX>>8);
			usartTransmit(accY&0xFF);
			usartTransmit(accY>>8);
			usartTransmit(accZ&0xFF);
			usartTransmit(accZ>>8);
			//PORTB &= ~(1 << 6);
			//computeAngle();
			//PORTB |= (1 << 6);
			usartTransmit(currentAngle&0xFF);
			usartTransmit(currentAngle>>8);

			usartTransmit(batteryLevel&0xFF);
			usartTransmit(batteryLevel>>8);

			usartTransmit(irCommand);

			usartTransmit(BUTTON0);

			usartTransmit(CHARGE_ON);
*/

			//sleep(20);

		}


		handleIRRemoteCommands();


		handleRFCommands();


		if(currentSelector == 0) {	// no control

			// compute velocities even if they aren't used
			if(compute_left_vel) {
				last_left_vel = left_vel_sum>>2;
				compute_left_vel = 0;
				left_vel_sum = 0;
			}

			if(compute_right_vel) {
				last_right_vel = right_vel_sum>>2;
				compute_right_vel = 0;
				right_vel_sum = 0;
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


		} else if(currentSelector == 1) {		// only horizontal speed control

			pwm_left_working = pwm_left_desired;
			pwm_right_working = pwm_right_desired;
			if(obstacleAvoidanceEnabled) {
				obstacleAvoidance(&pwm_left_working, &pwm_right_working);
			}
			pwm_left_desired_to_control = pwm_left_working;
			pwm_right_desired_to_control = pwm_right_working;

			if(compute_left_vel) {

				last_left_vel = left_vel_sum>>2;	// 4 samples taken for measuring velocity
				compute_left_vel = 0;
				left_vel_sum = 0;

				start_horizontal_speed_control_left(&pwm_left_working);

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

				start_horizontal_speed_control_right(&pwm_right_working);

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

		} else if(currentSelector == 2) {		// both horizontal and vertical speed control

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


	} // while(1)

}
