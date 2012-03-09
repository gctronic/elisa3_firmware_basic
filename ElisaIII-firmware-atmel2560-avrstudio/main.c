
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

	initPeripherals();

	calibrateSensors();

	initBehaviors();

	startTime = getTime100MicroSec();

	setRightSpeed(50);

	while(1) {

		currentSelector = getSelector();	// update selector position

		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();
	
		endTime = getTime100MicroSec();
		if((endTime-startTime) >= (PAUSE_2_SEC)) {
			readBatteryLevel();				// the battery level is updated every two seconds
			startTime = getTime100MicroSec();
			setRightSpeed(-50);
		}


		handleIRRemoteCommands();


		handleRFCommands();


		handleMotorsWithSpeedController();


	} // while(1)

}
