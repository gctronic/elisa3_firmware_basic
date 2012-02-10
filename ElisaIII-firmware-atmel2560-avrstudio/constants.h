#include <avr\io.h>

/***************/
/*** VARIOUS ***/
/***************/
#ifndef RAD_2_DEG
#define RAD_2_DEG 57.2957796
#endif

#ifndef CALIBRATION_CYCLES
#define CALIBRATION_CYCLES 16
#endif

/***********/
/*** NRF ***/
/***********/
#ifndef RF_ADDR
#define RF_ADDR 3204			// used to define hardware revision
#endif

#ifndef PAYLOAD_SIZE
#define PAYLOAD_SIZE 13
#endif

/************************/
/*** SPEED CONTROLLER ***/
/************************/
#ifndef PERIOD_MOTORS_100HZ
#define PERIOD_MOTORS_100HZ	40000			// old controller: We need a period time of 10 ms (100 Hz) => 0.01 * 4000000 = 40000
#endif

#ifndef MAX_PWM
#define MAX_PWM PERIOD_MOTORS_100HZ/100*60	// old controller: 60% of maximum
#endif

#ifndef MAX_MOTORS_PWM
#define MAX_MOTORS_PWM 1024		// max value for the pwm registers of the motors; 50% of maximum is then used actually
#endif

// orizzontal speed controller
#ifndef P_ORIZZONTAL
#define P_ORIZZONTAL 25
#endif

#ifndef I_ORIZZONTAL
#define I_ORIZZONTAL 2
#endif

#ifndef D_ORIZZONTAL
#define D_ORIZZONTAL 3
#endif

#ifndef I_LIMIT_ORIZZONTAL
#define I_LIMIT_ORIZZONTAL 1600
#endif

// vertical speed controller
#ifndef P_VERTICAL
#define P_VERTICAL 10
#endif

#ifndef I_VERTICAL
#define I_VERTICAL 3
#endif

#ifndef D_VERTICAL
#define D_VERTICAL 2
#endif

#ifndef I_LIMIT_VERTICAL
#define I_LIMIT_VERTICAL 1600
#endif

#ifndef INIT_KFF
#define INIT_KFF 25	
#endif

/****************/
/*** RGB LEDS ***/
/****************/
#ifndef MAX_LEDS_PWM
#define MAX_LEDS_PWM 255				// max value for the pwm registers of the leds
#endif

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
#ifndef STEP_MOTORS
#define STEP_MOTORS 30					// step used to increase/decrease the pwm of the motors when receiving a command through TV remote
#endif									// 30/1024 = 2.9%

/*********************/
/*** ACCELEROMETER ***/
/*********************/
#ifndef VERTICAL_THRESHOLD
#define VERTICAL_THRESHOLD 15
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

/*********************/
/*** HARDWARE DEFS ***/
/*********************/
#if RF_ADDR >= 3201 && RF_ADDR <= 3203
	#ifndef HW_REV_3_0
		#define HW_REV_3_0
		//#warning "Hardware revision 3.0"
	#endif
#endif
#if RF_ADDR == 3200
	#ifndef HW_REV_3_0_1
		#define HW_REV_3_0_1
		//#warning "Hardware revision 3.0.1"
	#endif
#endif
#if RF_ADDR > 3203
	#ifndef HW_REV_3_1
		#define HW_REV_3_1
		//#warning "Hardware revision 3.1"
	#endif
#endif

#ifndef SEL0
#define SEL0 (PINC & _BV(PC0))>>0
#endif

#ifndef SEL1
#define SEL1 (PINC & _BV(PC1))>>1
#endif

#ifndef SEL2
#define SEL2 (PINC & _BV(PC2))>>2
#endif

#ifndef SEL3
#define SEL3 (PINC & _BV(PC3))>>3
#endif

#ifndef CHARGE_ON
#define CHARGE_ON ((PINJ & _BV(PJ4))>>4)
#endif

#ifndef BUTTON0
#define BUTTON0 ((PINJ & _BV(PJ5))>>5)
#endif

#ifndef CHARGE_STAT
#define CHARGE_STAT ((PINJ & _BV(PJ7))>>7)
#endif

#ifndef SENS_ENABLE_OFF
#define SENS_ENABLE_OFF (PORTC &= ~(1 << 6))
#endif

#ifndef SENS_ENABLE_ON
#define SENS_ENABLE_ON (PORTC |= (1 << 6))
#endif

#ifndef LED_IR1_HIGH
#define LED_IR1_HIGH (PORTC |= (1<<4))
#endif

#ifndef LED_IR1_LOW
#define LED_IR1_LOW (PORTC &= ~(1<<4))
#endif

#ifndef LED_IR2_HIGH
#define LED_IR2_HIGH (PORTC |= (1<<5))
#endif
 
#ifndef LED_IR2_LOW
#define LED_IR2_LOW (PORTC &= ~(1<<5))
#endif

// the LED_X_ON/OFF macros works only if the leds port pins aren't
// set in pwm mode
#ifndef LED_RED_ON
#define LED_RED_ON (PORTB |= (1<<5))
#endif

#ifndef LED_GREEN_ON
#define LED_GREEN_ON (PORTB |= (1<<6))
#endif

#ifndef LED_BLUE_ON
#define LED_BLUE_ON (PORTB |= (1<<7))
#endif

#ifndef LED_RED_OFF
#define LED_RED_OFF (PORTB &= ~(1<<5))
#endif

#ifndef LED_GREEN_OFF
#define LED_GREEN_OFF (PORTB &= ~(1<<6))
#endif

#ifndef LED_BLUE_OFF
#define LED_BLUE_OFF (PORTB &= ~(1<<7))
#endif

#ifndef GREEN_LEDS
#define GRREEN_LED0_ON (PORTL &= ~(1 << 0))
#define GRREEN_LED1_ON (PORTL &= ~(1 << 1))
#define GRREEN_LED2_ON (PORTL &= ~(1 << 2))
#define GRREEN_LED3_ON (PORTG &= ~(1 << 3))
#define GRREEN_LED4_ON (PORTL &= ~(1 << 4))
#define GRREEN_LED5_ON (PORTL &= ~(1 << 5))
#define GRREEN_LED6_ON (PORTL &= ~(1 << 6))
#define GRREEN_LED7_ON (PORTL &= ~(1 << 7))
#define GRREEN_LED0_OFF (PORTL |= (1 << 0))
#define GRREEN_LED1_OFF (PORTL |= (1 << 1))
#define GRREEN_LED2_OFF (PORTL |= (1 << 2))
#define GRREEN_LED3_OFF (PORTG |= (1 << 3))
#define GRREEN_LED4_OFF (PORTL |= (1 << 4))
#define GRREEN_LED5_OFF (PORTL |= (1 << 5))
#define GRREEN_LED6_OFF (PORTL |= (1 << 6))
#define GRREEN_LED7_OFF (PORTL |= (1 << 7))
#endif

/***********/
/*** ADC ***/
/***********/
#ifndef ACTIVE_PHASE
#define ACTIVE_PHASE 0
#endif

#ifndef PASSIVE_PHASE
#define PASSIVE_PHASE 1
#endif

#ifndef NO_PHASE
#define NO_PHASE 2	// when pwm = 0
#endif

#ifndef SAVE_TO_PROX
#define SAVE_TO_PROX 0
#endif

#ifndef SAVE_TO_RIGHT_MOTOR_CURRENT
#define SAVE_TO_RIGHT_MOTOR_CURRENT 1
#endif

#ifndef SAVE_TO_RIGHT_MOTOR_VEL
#define SAVE_TO_RIGHT_MOTOR_VEL 2
#endif

#ifndef SAVE_TO_LEFT_MOTOR_CURRENT
#define SAVE_TO_LEFT_MOTOR_CURRENT 3
#endif

#ifndef SAVE_TO_LEFT_MOTOR_VEL
#define SAVE_TO_LEFT_MOTOR_VEL 4
#endif

#ifndef SKIP_SAMPLE
#define SKIP_SAMPLE 5
#endif

/***************/
/*** SENSORS ***/
/***************/
#ifndef PROX0_AMBIENT
#define PROX0_AMBIENT 0
#define PROX0_REFLECTED 1
#define PROX1_AMBIENT 2
#define PROX1_REFLECTED 3
#define PROX2_AMBIENT 4
#define PROX2_REFLECTED 5
#define PROX3_AMBIENT 6
#define PROX3_REFLECTED 7
#define PROX4_AMBIENT 8
#define PROX4_REFLECTED 9
#define PROX5_AMBIENT 10
#define PROX5_REFLECTED 11
#define PROX6_AMBIENT 12
#define PROX6_REFLECTED 13
#define PROX7_AMBIENT 14
#define PROX7_REFLECTED 15
#define GROUND0_AMBIENT 16
#define GROUND0_REFLECTED 17
#define GROUND1_AMBIENT 18
#define GROUND1_REFLECTED 19
#define GROUND2_AMBIENT 20
#define GROUND2_REFLECTED 21
#define GROUND3_AMBIENT 22
#define GROUND3_REFLECTED 23
#endif

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

