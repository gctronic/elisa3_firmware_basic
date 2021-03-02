
#include <avr\io.h>
#include <avr\interrupt.h>

#include <stdlib.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"


int main(void) {

	unsigned long int startTime = 0, endTime = 0;
	unsigned char prevSelector=0;

	initPeripherals();

	calibrateSensors();

	initBehaviors();

	startTime = getTime100MicroSec();


	while(1) {

		currentSelector = getSelector();	// update selector position

		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();
	
		endTime = getTime100MicroSec();
		if((endTime-startTime) >= (PAUSE_2_SEC)) {
			readBatteryLevel();				// the battery level is updated every two seconds
             		
			if(currentSelector==4 || currentSelector==5) {
				pwm_red = rand() % 255;
				pwm_green = rand() % 255;
				pwm_blue = rand() % 255;
			}

			startTime = getTime100MicroSec();
		}


		handleIRRemoteCommands();


		handleRFCommands();


		usart0Transmit(currentSelector,0);		// send the current selector position through uart as debug info


		switch(currentSelector) {
    
			case 0:	// motors in direct power control (no speed control)
					handleMotorsWithNoController();
					break;
             
			case 1:	// obstacle avoidance enabled (the robot does not move untill commands are 
					// received from the radio or tv remote)
             		enableObstacleAvoidance();
					break;
             
			case 2:	// cliff avoidance enabled (the robot does not move untill commands are 
					// received from the radio or tv remote)
             		enableCliffAvoidance();
					break;
    
			case 3:	// both obstacle and cliff avoidance enabled (the robot does not move untill commands are
					// received from the radio or tv remote)
            		enableObstacleAvoidance();
					enableCliffAvoidance();
					break;
            
			case 4:	// random colors on RGB leds; small green leds turned on
					GREEN_LED0_ON;
					GREEN_LED1_ON;
					GREEN_LED2_ON;
					GREEN_LED3_ON;
					GREEN_LED4_ON;
					GREEN_LED5_ON;
					GREEN_LED6_ON;
					GREEN_LED7_ON;
					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);
					break;
             
			case 5:	// random colors on RGB leds; obstacle avoidance enabled; robot start moving automatically
					// (motors speed setting)
					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);
					enableObstacleAvoidance();
					setLeftSpeed(25);
					setRightSpeed(25);
					break;
  
		}

		if(currentSelector != 0) {
			handleMotorsWithSpeedController();  
		}

		if(prevSelector != currentSelector) {	// in case the selector is changed, reset the robot state
			disableObstacleAvoidance();
			disableCliffAvoidance();
			GREEN_LED0_OFF;
			GREEN_LED1_OFF;
			GREEN_LED2_OFF;
			GREEN_LED3_OFF;
			GREEN_LED4_OFF;
			GREEN_LED5_OFF;
			GREEN_LED6_OFF;
			GREEN_LED7_OFF;
			pwm_red = 255;
			pwm_green = 255;
			pwm_blue = 255;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
		}
		prevSelector = currentSelector;


	} // while(1)

}
