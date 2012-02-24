
#include "behaviors.h"


void obstacleAvoidance(signed int *pwmLeft, signed int *pwmRight) {

	// Obstacle avoidance using all the proximity sensors based on a simplified 
	// force field method.
	//
	// General schema of the robot and related parameters:
	// 
	//		forward
	//
	//			0
	//		7		1
	//	velL	x	 velR
	//	  |		|	  |
	//	  6	  y_0	  2
	//
	//
	//		5		3
	//			4
	//
	// The follwoing table shows the weights (simplified respect to the trigonometry)
	// of all the proximity sensors for the resulting repulsive force:
	//
	//		0		1		2		3		4		5		6		7
	//	x	-1		-0.5	0		0.5		1		0.5		0		-0.5
	//	y	0		0.5		1		0.5		0		-0.5	-1		-0.5

	unsigned int i=0;
	signed int long res=0;
	signed int sumSensorsX=0, sumSensorsY=0;
	signed int desL=*pwmLeft, desR=*pwmRight;

	// consider small values to be noise thus set them to zero in order to not influence
	// the resulting force
	for(i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
	}

	// sum the contribution of each sensor (based on the previous weights table);
	// give more weight to prox2 and prox6 (side proximities) in order to get more stability in narrow aisles;
	// add some noise to the sum in order to escape from dead-lock positions
	sumSensorsX = -proximityResultLinear[0] - (proximityResultLinear[1]>>1) + (proximityResultLinear[3]>>1) + proximityResultLinear[4] + (proximityResultLinear[5]>>1) - (proximityResultLinear[7]>>1) + ((rand()%60)-30);
	sumSensorsY = (proximityResultLinear[1]>>1) + (proximityResultLinear[2]>>2) + (proximityResultLinear[3]>>1) - (proximityResultLinear[5]>>1) - (proximityResultLinear[6]>>2) - (proximityResultLinear[7]>>1)+ ((rand()%60)-30);
	//sumSensorsY = (proximityResultLinear[1]>>1) + (proximityResultLinear[3]>>1) - (proximityResultLinear[5]>>1) - (proximityResultLinear[7]>>1) + (rand()%30);

	// modify the velocity components based on sensor values
	if(desL >= 0) {
		res = (signed long int)desL + (((signed long int)desL * ((signed long int)sumSensorsX - (signed long int)sumSensorsY))>>7);
		*pwmLeft = (signed int)res;
	} else {
		res = (signed long int)desR - (((signed long int)desR * ((signed long int)sumSensorsX + (signed long int)sumSensorsY))>>7);
		*pwmLeft = (signed int)res;
	}
	if(desR >=0) {
		res = (signed long int)desR + (((signed long int)desR * ((signed long int)sumSensorsX + (signed long int)sumSensorsY))>>7);
		*pwmRight = (signed int)res;
	} else {
		res = (signed long int)desL - (((signed long int)desL * ((signed long int)sumSensorsX - (signed long int)sumSensorsY))>>7);
		*pwmRight = (signed int)res;
	}
		
	// force the values to be in the pwm maximum range
	if (*pwmRight>(MAX_MOTORS_PWM/2)) *pwmRight=(MAX_MOTORS_PWM/2);
	if (*pwmLeft>(MAX_MOTORS_PWM/2)) *pwmLeft=(MAX_MOTORS_PWM/2);
	if (*pwmRight<-(MAX_MOTORS_PWM/2)) *pwmRight=-(MAX_MOTORS_PWM/2);
	if (*pwmLeft<-(MAX_MOTORS_PWM/2)) *pwmLeft=-(MAX_MOTORS_PWM/2);

}


char cliffDetected() {

	// tell whether a cliff is detected or not
	if(proximityResult[8]<CLIFF_THR || proximityResult[9]<CLIFF_THR || proximityResult[10]<CLIFF_THR || proximityResult[11]<CLIFF_THR) {
		return 1;
	} else {
		return 0;
	}


}


