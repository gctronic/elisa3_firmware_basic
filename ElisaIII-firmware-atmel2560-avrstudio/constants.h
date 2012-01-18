#ifndef CONSTANTS
#define CONSTANTS

/***************/
/*** VARIOUS ***/
/***************/
#define PI 3.14159265
#define RAD_2_DEG 57.2957796
#ifndef USE_LEDS_IN_DEBUG
#define USE_LEDS_IN_DEBUG 0
#endif
#ifndef ALARM_PAUSE_1_SEC
#define ALARM_PAUSE_1_SEC 1u
#endif
#ifndef ALARM_PAUSE_10_SEC
#define ALARM_PAUSE_10_SEC 10u
#endif
#ifndef ALARM_PAUSE_1_MIN
#define ALARM_PAUSE_1_MIN 60u
#endif

/***********/
/*** NRF ***/
/***********/
#define BIDIRECTIONAL
#define USE_REDUCED_PACKET
#define CHANGE_STATE 0x0
#define CHANGE_RF 0x1
#define RF_ADDR 3200			// used to define hardware revision
#ifdef USE_REDUCED_PACKET
	#define PAYLOAD_SIZE 6
#else
	#define PAYLOAD_SIZE 31
#endif

/************************/
/*** SPEED CONTROLLER ***/
/************************/
#define MAX_MOTORS_PWM 614 //1023/100*60		// max value for the pwm registers of the motors; 60% of maximum
#define P_ORIZZONTAL 100
#define I_ORIZZONTAL 2
#define D_ORIZZONTAL 10
#define I_LIMIT_ORIZZONTAL 3200
#define K_FF_ORIZZONTAL 120 			// optimized for low velocities //30
#define PERIOD_MOTORS_100HZ	40000			// old controller: We need a period time of 10 ms (100 Hz) => 0.01 * 4000000 = 40000
#define MAX_PWM PERIOD_MOTORS_100HZ/100*60	// old controller: 60% of maximum

#ifndef MAX_MEAS_SPEED
#define MAX_MEAS_SPEED 800								// measured speed at 60% duty cycle (free run)
#endif
#ifndef PWM_THRESHOLD
#define PWM_THRESHOLD 		PERIOD_MOTORS_100HZ/100*20	// percentage of max velocity (motors period); values are between 0..PERIOD_MOTORS
#endif
#ifndef INIT_KFF
#define INIT_KFF 150	// speed controller
#endif

/****************/
/*** RGB LEDS ***/
/****************/
#define MAX_LEDS_PWM 255				// max value for the pwm registers of the leds

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
#define STEP_MOTORS 30					// step used to increase/decrease the pwm of the motors when receiving a command through TV remote
										// 30/1024 = 2.9%

/*********************/
/*** ACCELEROMETER ***/
/*********************/
#ifndef DELTA_ANGLE_FACT
#define DELTA_ANGLE_FACT MAX_PWM/180					// scale the error to be in the range 0..MAX_PWM
#endif
#ifndef NULL_ANGLE_THRESHOLD
#ifdef _HARDWARE_V2_1
	#define NULL_ANGLE_THRESHOLD 75
#else
	#define NULL_ANGLE_THRESHOLD 150
#endif
#endif
#ifndef SAME_POS_NUM
#define SAME_POS_NUM 5
#endif
#ifndef VERTICAL_POS
#define VERTICAL_POS 0
#endif
#ifndef ORIZZONTAL_POS
#define ORIZZONTAL_POS 1
#endif
#ifndef MMA7455L_ADDR
#define MMA7455L_ADDR (0x1D<<1)
#endif
#ifndef ADXL345_ADDR
#define ADXL345_ADDR (0x53<<1)
#endif
#ifndef USE_MMAX7455L
#define USE_MMAX7455L 0
#endif
#ifndef USE_ADXL345
#define USE_ADXL345 1
#endif
#ifndef USE_NO_ACCEL
#define USE_NO_ACCEL 2
#endif

/******************************/
/*** CONSUMPTION CONTROLLER ***/
/******************************/
#ifndef CURRENT_LIM_UP
#define CURRENT_LIM_UP 		60							// used to change the limit of the current consumption for a certain pwm/duty: limit = (CURRENT_LIM_UP-duty)/2
#endif
#ifndef MAX_CURRENT_MEAS									// this value was found with measures
#define MAX_CURRENT_MEAS 	60							// max adc value measured when the motor is stopped
#endif

/*********************/
/*** HARDWARE DEFS ***/
/*********************/
#if RF_ADDR >= 3201 && RF_ADDR <= 3203
	#define HW_REV_3_0
#endif
#if RF_ADDR == 3200
	#define HW_REV_3_0_1
#endif
#if RF_ADDR > 3203
	#define HW_REV_3_1
#endif
#define SEL0 		(PINC & _BV(PC0))>>0
#define SEL1 		(PINC & _BV(PC1))>>1
#define SEL2 		(PINC & _BV(PC2))>>2
#define SEL3 		(PINC & _BV(PC3))>>3
//#define ACC_ADXL345
#define ACC_MMA7455L
#define CHARGE_ON ((PINJ & _BV(PJ4))>>4)
#define BUTTON0 ((PINJ & _BV(PJ5))>>5)
#define LED_IR1_HIGH (PORTC |= (1<<4))
#define LED_IR1_LOW (PORTC &= ~(1<<4))
#define LED_IR2_HIGH (PORTC |= (1<<5))
#define LED_IR2_LOW (PORTC &= ~(1<<5))

/***********/
/*** ADC ***/
/***********/
#define ACTIVE_PHASE 0
#define PASSIVE_PHASE 1
#define SAVE_TO_PROX 0
#define SAVE_TO_RIGHT_MOTOR_CURRENT 1
#define SAVE_TO_RIGHT_MOTOR_VEL 2
#define SAVE_TO_LEFT_MOTOR_CURRENT 3
#define SAVE_TO_LEFT_MOTOR_VEL 4

/***********************/
/*** CLIFF AVOIDANCE ***/
/***********************/
#ifndef CLIFF_THR
#define CLIFF_THR 115
#endif
#ifndef LEFT_ROT
#define LEFT_ROT  1
#endif
#ifndef RIGHT_ROT
#define RIGHT_ROT 2
#endif
#ifndef GROUND_RIGHT
#define GROUND_RIGHT 0
#endif
#ifndef GROUND_CENTER_RIGHT
#define GROUND_CENTER_RIGHT 1
#endif
#ifndef GROUND_CENTER_LEFT
#define GROUND_CENTER_LEFT 2
#endif
#ifndef GROUND_LEFT
#define GROUND_LEFT 3
#endif

#endif
