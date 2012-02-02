
#include "behaviors.h"


void obstacleAvoidance() {
	// obstacle avoidance using the 3 front proximity sensors

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
	// The follwoing tables shows the weights (simplified respect to the trigonometry)
	// of all the proximity sensors for the resulting force:
	//
	//		0		1		2		3		4		5		6		7
	//	x	-1		-0.5	0		0.5		1		0.5		0		-0.5
	//	y	0		0.5		1		0.5		0		-0.5	-1		-0.5

	signed int velX=0, velY=0;
	unsigned int sumSensorsX=0, sumSensorsY=0;
	signed int speedL=0, speedR=0;


	if(pwm_right_desired < 0) {
		speedr = -speedr;
	}
	if(pwm_left_desired < 0) {
		speedl = - speedl;
	}

	velX = (speedr + speedl)/2;
	velY = (speedr - speedl)/2;

	sumSensorsX = -proximityResult[0] - proximityResult[1]/2 + proximityResult[3]/2 + proximityResult[4] + proximityResult[5]/2 - proximityResult[7]/2;
	sumSensorsY = proximityResult[1]/2 + proximityResult[2] + proximityResult[3]/2 - proximityResult[5]/2 - proximityResult[6] - proximityResult[7]/2;

	velX += sumSensorsX/4;
	velY += sumSensorsY/4;

	speedR = (velX + velY);
	speedL = (velX - velY);

	if(speedL < 0) {
		speedL = -speedL;
		pwm_left_working = -(speedL<<2);
	} else {
		pwm_left_working = speedL<<2;
	}

	if(speedR < 0) {
		speedR = -speedR;
		pwm_right_working = -(speedR<<2);
	} else {
		pwm_right_working = speedR<<2;
	}


	if (pwm_right_working>(MAX_MOTORS_PWM/2)) pwm_right_working=(MAX_MOTORS_PWM/2);
	if (pwm_left_working>(MAX_MOTORS_PWM/2)) pwm_left_working=(MAX_MOTORS_PWM/2);
	if (pwm_right_working<-(MAX_MOTORS_PWM/2)) pwm_right_working=-(MAX_MOTORS_PWM/2);
	if (pwm_left_working<-(MAX_MOTORS_PWM/2)) pwm_left_working=-(MAX_MOTORS_PWM/2);



/*
	signed int currentProxValue1=0, currentProxValue2=0, speedL=0, speedR=0;

	if(speedr==0 || speedl==0) {
		pwm_right_working = 0;
		pwm_left_working = 0;
		return;
	}

	// max speed tested = 40
	if(speedr > 40) {
		speedr = 40;
	}
	if(speedl > 40) {
		speedl = 40;
	}

	currentProxValue1 = proximityValue[0] - proximityValue[1];	// ambient - (ambient+reflected)
	if(currentProxValue1 < 0) {
		currentProxValue1 = 0;
	}
	currentProxValue2 = proximityValue[2] - proximityValue[3];	// ambient - (ambient+reflected)
	if(currentProxValue2 < 0) {
		currentProxValue2 = 0;
	}
	rightProxSum = currentProxValue1/2 + currentProxValue2;
	currentProxValue2 = proximityValue[14] - proximityValue[15];	// ambient - (ambient+reflected)
	if(currentProxValue2 < 0) {
		currentProxValue2 = 0;
	}
	leftProxSum = currentProxValue1/4 + currentProxValue2;


	rightProxSum = rightProxSum/(int)(300/speedr);	// scale the sum to have a moderate impact on the velocity
	leftProxSum = leftProxSum/(int)(300/speedl);

	if(rightProxSum > (int)(speedr*2)) {		// velocity of the motors goes from -30 to +30
		rightProxSum = (int)(speedr*2);
	}
	if(leftProxSum > (int)(speedl*2)) {
		leftProxSum = (int)(speedl*2);
	}


	speedL = (int)speedl - rightProxSum;
	speedR = (int)speedr - leftProxSum;

	if(speedL < 0) {
		speedL = -speedL;
		pwm_left_working = -(speedL<<2);
	} else {
		pwm_left_working = speedL<<2;
	}

	if(speedR < 0) {
		speedR = -speedR;
		pwm_right_working = -(speedR<<2);
	} else {
		pwm_right_working = speedR<<2;
	}


	if (pwm_right_working>(MAX_MOTORS_PWM/2)) pwm_right_working=(MAX_MOTORS_PWM/2);
	if (pwm_left_working>(MAX_MOTORS_PWM/2)) pwm_left_working=(MAX_MOTORS_PWM/2);
	if (pwm_right_working<-(MAX_MOTORS_PWM/2)) pwm_right_working=-(MAX_MOTORS_PWM/2);
	if (pwm_left_working<-(MAX_MOTORS_PWM/2)) pwm_left_working=-(MAX_MOTORS_PWM/2);
*/

}


void cliffAvoidance() {

	signed int g0=0, g1=0, g2=0, g3=0;

	g0 = proximityValue[16] - proximityValue[17];	// ambient - (ambient+reflected)
	if(g0 < 0) {
		g0 = 0;
	}

	g1 = proximityValue[18] - proximityValue[19];	// ambient - (ambient+reflected)
	if(g1 < 0) {
		g1 = 0;
	}

	g2 = proximityValue[20] - proximityValue[21];	// ambient - (ambient+reflected)
	if(g2 < 0) {
		g2 = 0;
	}

	g3 = proximityValue[22] - proximityValue[23];	// ambient - (ambient+reflected)
	if(g3 < 0) {
		g3 = 0;
	}

	minGroundValue = g0;
	minGround = GROUND_LEFT;
	if(g1 < minGroundValue) {
		minGroundValue = g1;
		minGround = GROUND_CENTER_LEFT;
	}
	if(g2 < minGroundValue) {
		minGroundValue = g2;
		minGround = GROUND_CENTER_RIGHT;
	}
	if(g3 < minGroundValue) {
		minGroundValue = g3;
		minGround = GROUND_RIGHT;
	}

	if(minGroundValue <= CLIFF_THR) {
		pwm_right_working = 0;
		pwm_left_working = 0;
	}

}


