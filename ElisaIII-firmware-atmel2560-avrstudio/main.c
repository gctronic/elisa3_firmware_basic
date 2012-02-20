
#include <avr\io.h>
#include <avr\interrupt.h>
#include <math.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"


int main(void) {

	unsigned int i = 0;
	choosePeripheral = 1;

	initPeripherals();

	startCalibration = 1;
	calibrationCycle = 0;
/*
	pwm_red = 0;
	pwm_green = 0;
	pwm_blue = 0;
	updateRedLed(pwm_red);
	updateGreenLed(pwm_green);
	updateBlueLed(pwm_blue);
*/

	//PORTL = 0x00;
	//PORTG &= ~(1 << 3);

	while(1) {

		//PORTB ^= (1 << 6); 	// toggle the green led

		currentSelector = getSelector();

		readAccelXYZ();

		computeAngle();
/*
		if(delayCounter%78 == 0) {
			start_control = 1;
		}
*/
/*
		if(delayCounter >= 10000) {
			pwm_right_desired = 40;
			pwm_left_desired = 40;
		} else {
			pwm_right_desired = 0;
			pwm_left_desired = 0;
		}
*/
		if(delayCounter >= 20000) {
			measBattery = 1;
			//sleep(20);
			//delayCounter = 0;
		}


		if(startCalibration && calibrationCycle<=CALIBRATION_CYCLES) {

			if(proxUpdated) {

				proxUpdated = 0;

				if(calibrationCycle==0) {		// reset all variables
					for(i=0; i<12; i++) {
						proximitySum[i] = 0;
						proximityOffset[i] = 0;
					}
					accOffsetXSum = 0;
					accOffsetYSum = 0;
					accOffsetZSum = 0;

					calibrationCycle++;

					continue;					// the first time "proxUpdated" is set, all the proximity values saved in the array 
												// "proximityResult" hasn't the offset reset to 0. so we start the actual calibration
												// the next time
				}

				for (i=0;i<12;i++) {
					proximitySum[i] += proximityResult[i];
				}

				accOffsetXSum += accX;
				accOffsetYSum += accY;
				accOffsetZSum += accZ;

				calibrationCycle++;

			}

			continue;

		} else if(calibrationCycle > CALIBRATION_CYCLES) {

			for(i=0;i<12;i++) {
				proximityOffset[i] = proximitySum[i]>>4;
			}

			for(i=8; i<12; i++) {
				proximityOffset[i] -= 512;
			}

			accOffsetX = accOffsetXSum>>4;
			accOffsetY = accOffsetYSum>>4;
			accOffsetZ = accOffsetZSum>>4;

			startCalibration = 0;
			calibrationCycle = 0;
/*
			pwm_red = 255;
			pwm_green = 255;
			pwm_blue = 255;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
*/

		}

		if(irEnabled) {

			ir_move = ir_remote_get_data();

			if(command_received) {

				command_received = 0;

				switch(ir_move) {

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

						if(colorState==0) {		// turn on blue
							LED_IR1_HIGH;
							LED_IR2_HIGH;
							pwm_blue = 0;
							pwm_green = MAX_LEDS_PWM;
							pwm_red = MAX_LEDS_PWM;
						} else if(colorState==1) {	// turn on green
							pwm_blue = MAX_LEDS_PWM;
							pwm_green = 0;
							pwm_red = MAX_LEDS_PWM;
						} else if(colorState==2) {	// turn on red
							LED_IR1_LOW;
							LED_IR2_LOW;
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

					case 52:
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


		//if(sendAdcValues && myTimeout) {
		//if(sendAdcValues) {
		//if(myTimeout) {
		if(delayCounter >= 20000) {
			delayCounter = 0;
/*
			sendAdcValues = 0;
			myTimeout = 0;

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

			usartTransmit(ir_move);

			usartTransmit(BUTTON0);

			usartTransmit(CHARGE_ON);
*/
		}

//PORTB &= ~(1 << 7);
		if(mirf_data_ready()) {

			// clear irq status
			mirf_config_register(STATUS, 0x70);

			mirf_get_data(rfData);
			flush_rx_fifo();

			//usartTransmit(rfData[0]);


			//if((data[3]&0b00001000)==0b00001000) {	// check the 4th bit to sleep
			// it was noticed that some robots sometimes "think" to receive something and the data read are wrong,
			// this could lead to go to sleep involuntarily; in order to avoid this situation we define that the
			// sleep message should be completely zero, but the flag bit
			if(rfData[0]==0 && rfData[1]==0 && rfData[2]==0 && rfData[3]==0b00001000 && rfData[4]==0 && rfData[5]==0) {

				sleep(60);

			}

			speedr = (rfData[4]&0x7F);	// cast the speed to be at most 127, thus the received speed are in the range 0..127 (usually 0..100),
			speedl = (rfData[5]&0x7F);	// the received speed is then shifted by 3 (x8) in order to have a speed more or less
										// in the same range of the measured speed that is 0..800.
										// In order to have greater resolution at lower speed we shift the speed only by 2 (x4),
										// this means that the range is more or less 0..400.


			if((rfData[4]&0x80)==0x80) {			// motor right forward
				pwm_right_desired = speedr<<2;				// scale the speed received (0..100) to be in the range 0..400
			} else {								// backward
				pwm_right_desired = -(speedr<<2);
			}

			if((rfData[5]&0x80)==0x80) {			// motor left forward
				pwm_left_desired = speedl<<2;
			} else {								// backward
				pwm_left_desired = -(speedl<<2);
			}

			if (pwm_right_desired>(MAX_MOTORS_PWM/2)) pwm_right_desired=(MAX_MOTORS_PWM/2);
            if (pwm_left_desired>(MAX_MOTORS_PWM/2)) pwm_left_desired=(MAX_MOTORS_PWM/2);
            if (pwm_right_desired<-(MAX_MOTORS_PWM/2)) pwm_right_desired=-(MAX_MOTORS_PWM/2);
            if (pwm_left_desired<-(MAX_MOTORS_PWM/2)) pwm_left_desired=-(MAX_MOTORS_PWM/2);


			for(i=0; i<3; i++) {
				dataLED[i]=rfData[i]&0xFF;
			}
			pwm_red = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[0]&0xFF)/100;
			pwm_blue = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[1]&0xFF)/100;
			pwm_green = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[2]&0xFF)/100;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);


			if((rfData[3]&0b00000001)==0b00000001) {	// turn on back IR
				LED_IR1_LOW;
			} else {
				LED_IR1_HIGH;
			}

			if((rfData[3]&0b00000010)==0b00000010) {	// turn on front IRs
				LED_IR2_LOW;
			} else {
				LED_IR2_HIGH;
			}

			if((rfData[3]&0b00000100)==0b00000100) {	// check the 3rd bit to enable/disable the IR receiving
				irEnabled = 1;
			} else {
				irEnabled = 0;
			}

			if((rfData[3]&0b00010000)==0b00010000) {	// check the 5th bit to start calibration of all sensors
				if(!startCalibration) {					// if not already calibrating the sensors
					startCalibration = 1;
					calibrationCycle = 0;
					pwm_red = 0;
					pwm_green = 0;
					pwm_blue = 0;
					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);
				}
			}

			if((rfData[3]&0b01000000)==0b01000000) {	// check the seventh bit to enable/disable obstacle avoidance
				obstacleAvoidanceEnabled = 1;
			} else {
				obstacleAvoidanceEnabled = 0;
			}

			if((rfData[3]&0b10000000)==0b10000000) {	// check the eight bit to enable/disable obstacle avoidance
				cliffAvoidanceEnabled = 1;
			} else {
				cliffAvoidanceEnabled = 0;
			}

			// handle small green leds
			#ifdef HW_REV_3_1			

				if(bit_is_set(rfData[6], 0) ) {
					GRREEN_LED0_ON;
				} else {
					GRREEN_LED0_OFF;
				}
				
				if(bit_is_set(rfData[6], 1) ) {
					GRREEN_LED1_ON;
				} else {
					GRREEN_LED1_OFF;
				}
				
				if(bit_is_set(rfData[6], 2) ) {
					GRREEN_LED2_ON;
				} else {
					GRREEN_LED2_OFF;
				}												

				if(bit_is_set(rfData[6], 3) ) {
					GRREEN_LED3_ON;
				} else {
					GRREEN_LED3_OFF;
				}

				if(bit_is_set(rfData[6], 4) ) {
					GRREEN_LED4_ON;
				} else {
					GRREEN_LED4_OFF;
				}

				if(bit_is_set(rfData[6], 5) ) {
					GRREEN_LED5_ON;
				} else {
					GRREEN_LED5_OFF;
				}

				if(bit_is_set(rfData[6], 6) ) {
					GRREEN_LED6_ON;
				} else {
					GRREEN_LED6_OFF;
				}

				if(bit_is_set(rfData[6], 7) ) {
					GRREEN_LED7_ON;
				} else {
					GRREEN_LED7_OFF;
				}

			#endif

			// read and handle the remaining bytes of the payload (at the moment not used)


			//packetId = (packetId+1)%256;
			//writeAckPayload(&packetId, 1);
			//for(i=0; i<12; i++) {
			//	ackPayload[i] = (proximityValue[(i*2)+1]>>2);
			//}

			ackPayload[0] = packetId&0xFF;

			switch(packetId) {
				case 3:
					ackPayload[1] = proximityResult[0]&0xFF;
					ackPayload[2] = proximityResult[0]>>8;
					ackPayload[3] = proximityResult[1]&0xFF;
					ackPayload[4] = proximityResult[1]>>8;
					ackPayload[5] = proximityResult[2]&0xFF;
					ackPayload[6] = proximityResult[2]>>8;
					ackPayload[7] = proximityResult[3]&0xFF;
					ackPayload[8] = proximityResult[3]>>8;
					ackPayload[9] = proximityResult[5]&0xFF;
					ackPayload[10] = proximityResult[5]>>8;
					ackPayload[11] = proximityResult[6]&0xFF;
					ackPayload[12] = proximityResult[6]>>8;
					ackPayload[13] = proximityResult[7]&0xFF;
					ackPayload[14] = proximityResult[7]>>8;
					#ifdef HW_REV_3_1
						ackPayload[15] = CHARGE_ON | (BUTTON0 << 1) | (CHARGE_STAT << 2);
					#else
						ackPayload[15] = CHARGE_ON | (BUTTON0 << 1);
					#endif
					packetId = 4;
					break;

				case 4:
					ackPayload[1] = proximityResult[4]&0xFF;
					ackPayload[2] = proximityResult[4]>>8;
					ackPayload[3] = proximityResult[8]&0xFF;
					ackPayload[4] = proximityResult[8]>>8;
					ackPayload[5] = proximityResult[9]&0xFF;
					ackPayload[6] = proximityResult[9]>>8;
					ackPayload[7] = proximityResult[10]&0xFF;
					ackPayload[8] = proximityResult[10]>>8;
					ackPayload[9] = proximityResult[11]&0xFF;
					ackPayload[10] = proximityResult[11]>>8;
					ackPayload[11] = accX&0xFF;	//((-accOffsetY)&0x03FF)
					ackPayload[12] = accX>>8;
					ackPayload[13] = accY&0xFF;
					ackPayload[14] = accY>>8;
					ackPayload[15] = ir_move;
					packetId = 5;
					break;

				case 5:
					ackPayload[1] = proximityValue[0]&0xFF;
					ackPayload[2] = proximityValue[0]>>8;
					ackPayload[3] = proximityValue[2]&0xFF;
					ackPayload[4] = proximityValue[2]>>8;
					ackPayload[5] = proximityValue[4]&0xFF;
					ackPayload[6] = proximityValue[4]>>8;
					ackPayload[7] = proximityValue[6]&0xFF;
					ackPayload[8] = proximityValue[6]>>8;
					ackPayload[9] = proximityValue[10]&0xFF;
					ackPayload[10] = proximityValue[10]>>8;
					ackPayload[11] = proximityValue[12]&0xFF;
					ackPayload[12] = proximityValue[12]>>8;
					ackPayload[13] = proximityValue[14]&0xFF;
					ackPayload[14] = proximityValue[14]>>8;
					ackPayload[15] = currentSelector;
					packetId = 6;
					break;

				case 6:
					ackPayload[1] = proximityValue[8]&0xFF;
					ackPayload[2] = proximityValue[8]>>8;
					ackPayload[3] = proximityValue[16]&0xFF;
					ackPayload[4] = proximityValue[16]>>8;
					ackPayload[5] = proximityValue[18]&0xFF;
					ackPayload[6] = proximityValue[18]>>8;
					ackPayload[7] = proximityValue[20]&0xFF;
					ackPayload[8] = proximityValue[20]>>8;
					ackPayload[9] = proximityValue[22]&0xFF;
					ackPayload[10] = proximityValue[22]>>8;
					ackPayload[11] = accZ&0xFF;
					ackPayload[12] = accZ>>8;
					ackPayload[13] = batteryLevel&0xFF;
					ackPayload[14] = batteryLevel>>8;
					ackPayload[15] = 0;
					packetId = 3;
					break;


			}

			writeAckPayload(ackPayload, 16);

		}
//PORTB |= (1 << 7);


		if(currentSelector == 0) {	// no control

			// compute velocities even if they aren't used...
			if(compute_left_vel) {
				last_left_vel = left_vel_sum>>2;
				left_vel_changed = 1;
				compute_left_vel = 0;
				left_vel_sum = 0;
			}

			if(compute_right_vel) {
				last_right_vel = right_vel_sum>>2;
				right_vel_changed = 1;
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

			update_pwm = 1;


		} else if(currentSelector == 1) {		// only horizontal speed control

			if(obstacleAvoidanceEnabled) {
				obstacleAvoidance();
			}

			if(compute_left_vel) {

				last_left_vel = left_vel_sum>>2;
				left_vel_changed = 1;
				compute_left_vel = 0;
				left_vel_sum = 0;

				pwm_left_working = pwm_left_desired;
				pwm_left_desired_to_control = pwm_left_desired;

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
				right_vel_changed = 1;
				compute_right_vel = 0;
				right_vel_sum = 0;

				pwm_right_working = pwm_right_desired;
				pwm_right_desired_to_control = pwm_right_desired;

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

		} else if(currentSelector == 2) {		// both speed control horizontal and vertical

			pwm_left_working = pwm_left_desired;
			pwm_right_working = pwm_right_desired;
			if(obstacleAvoidanceEnabled) {
				obstacleAvoidance(&pwm_left_working, &pwm_right_working);
			}
			pwm_left_desired_to_control = pwm_left_working;
			pwm_right_desired_to_control = pwm_right_working;

			if(compute_left_vel) {

				last_left_vel = left_vel_sum>>2;
				left_vel_changed = 1;
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
				right_vel_changed = 1;
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

		if(update_pwm) {

			update_pwm = 0;
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

		}

	} // while(1)

}
