#include "constants.h"

/***********/
/*** ADC ***/
/***********/
// channel 0..6:  prox0..6
// channel 7:	  prox7/battery
// channel 8..11: cliff0..3
// channel 12:	  active phase when going backward: motor right current; passive phase when going forward: motor right velocity 
// channel 13.	  active phase when going forward: motor right current; passive phase when going backward: motor right velocity
// channel 14:    active phase when going backward: motor left current; passive phase when going forward: motor left velocity 
// channel 15:    active phase when going forward: motor left current; passive phase when going backward: motor left velocity
// The channel selection sequence is:
volatile unsigned char currentAdChannel = 0;		// next channel to be sampled
unsigned char currentProx = 0;						// current array index for the current sampled proximity value
unsigned char currentMotLeftChannel = 0;			// current channel to sample for the left motor based on forward/backward and active/passive phase
unsigned char currentMotRightChannel = 0;			// current channel to sample for the right motor based on forward/backward and active/passive phase
unsigned char rightMotorPhase = 0;					// current right motor pwm phase (active or passive)
unsigned char leftMotorPhase = 0;					// current left motor pwm phase (active or passive)
volatile unsigned int proximityValue[24] = {0};		// array containing the proximity values: 
													// index	sensor	value
													// 0		prox0	passive (no pulse) value
													// 1		prox0	active (pulse) value
													// 2		prox1	passive value
													// 3		prox1	active value
													// ...
													// 16		cliff0	passive value
													// 17		cliff0	active value
													// ...
unsigned char adcSaveDataTo = 0;					// indicate where to save the currently sampled channel
unsigned char adcSamplingState = 0;					// indicate which channel to select
unsigned char rightChannelPhase = 0;				// right motor phase when the channel was selected
unsigned char leftChannelPhase = 0;					// left motor phase when the channel was selected
unsigned int batteryLevel = 0;
unsigned char measBattery = 0;

/******************************/
/*** CONSUMPTION CONTROLLER ***/
/******************************/
unsigned int left_current_avg = 0;					// the left motor current consumption is sampled from 0 to duty cycle at each motor period; 
unsigned int right_current_avg = 0;					// the average of all these samples is given as the new current consumption
unsigned int last_left_current = 0;					// current consumption of the previous/last period
unsigned int last_right_current = 0;

/************************/
/*** SPEED CONTROLLER ***/
/************************/
unsigned int num_lvel_samples_avg = 0;				// current number of samples received for filtering (average) the left velocity value
volatile unsigned int last_num_lvel_samples_avg = 0;
unsigned int num_rvel_samples_avg = 0;				// current number of samples received for filtering (average) the right velocity value
volatile unsigned int last_num_rvel_samples_avg = 0;
unsigned int left_vel_sum = 0;						// sum of all the adc values (at the end they will be divided by the total number of samples received)
volatile unsigned int last_left_vel_sum = 0;
unsigned int right_vel_sum = 0;
volatile unsigned int last_right_vel_sum = 0;
unsigned int last_left_vel = 0;
unsigned int last_right_vel = 0;
signed long int max_pwm_right = MAX_MOTORS_PWM;		// if the period of the motors is changed at runtime, also the maximum pwm (60%) value
signed long int max_pwm_left = MAX_MOTORS_PWM;		// must be changed with these vars (the new values is period_motor_x/100*60)
signed long int pwm_right = 0;						// value to set to the pwm register; this value is expressed in the range 0..PERIOD MOTORS
signed long int pwm_left = 0;
//signed long int pwm_right_working = 0;				// current temporary pwm used in the controllers
//signed long int pwm_left_working = 0;
signed long int pwm_right_desired = 0;				// the pwm given by a command through IR or RF
signed long int pwm_left_desired = 0;				// this pwm is expressed in the range 0..MAX_MOTORS_PWM (1023 corresponds to 100% duty cycle)
//unsigned int p_speed_control;						// PID parameters
//unsigned int d_speed_control;
//unsigned int i_speed_control;
//unsigned int i_limit_speed_control;				// maximum value for the I parameter
//unsigned int k_ff_speed_control_left;
//unsigned int k_ff_speed_control_right;
signed long int pwm_right_speed_controller = 0;		// the pwm values after the speed controller adaptation
signed long int pwm_left_speed_controller = 0;
signed long int delta_left_speed = 0;
signed long int delta_right_speed = 0;
signed long int delta_left_speeds[2];
signed long int delta_right_speeds[2];
signed long int delta_left_speed_sum = 0;
signed long int delta_right_speed_sum = 0;
signed long int left_increment = 0;
signed long int right_increment = 0;
//signed int current_angle = 0;
unsigned char compute_left_vel = 0;					// set at the beginning of the period to indicate that a new measure of the motor speed can be
unsigned char compute_right_vel = 0;
unsigned char start_control = 0;					// set at the beginning of the motors period to indicate that the controllers can be started
													// using information coming from the previous cycle
unsigned char left_vel_changed = 0;					// indicate that a new speed for the left motor is measured and ready to be used
unsigned char right_vel_changed = 0;				// the same for the right motor
signed long int pwm_right_working = 0;				// current temporary pwm used in the controllers
signed long int pwm_left_working = 0;
unsigned char update_pwm = 0;						// indicate that the controllers finished and thus the motors pwm can be updated

/***********/
/*** NRF ***/
/***********/
unsigned int dataLED[3];							// array containing the value received through the radio
unsigned char speedl=0, speedr=0;					// current speed for left and right motors received through the radio
unsigned char rfData[PAYLOAD_SIZE];					// data received through the radio
unsigned char ackPayload[16];						// data to send back to the base-station

/****************/
/*** RGB LEDS ***/
/****************/
unsigned char pwm_red = 255, pwm_green = 255, pwm_blue = 255;	// value to set to the pwm registers; this value is expressed in the range 0 (max power) to 255 (off)
unsigned char blinkState = 0;									// used to toggle the blue led

/************/
/*** UART ***/
/************/
unsigned char peripheralChoice = 5;	// red led=0, green led=1, blue led=2, right motor=3, left motor=4, send adc values=5s
unsigned char choosePeripheral = 1;	// uart state: choose the peripheral or act on choosen peripheral
unsigned char sendAdcValues = 0;

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
unsigned char ir_move = 0;
unsigned char command_received = 0;
unsigned char colorState = 0;		// used with command 0 to switch from one color to the next

/*********************/
/*** ACCELEROMETER ***/
/*********************/
#ifdef ACC_MMA7455L
	unsigned char accelAddress = 0x1D<<1;
#endif
#ifdef ACC_ADXL345
	unsigned char accelAddress = 0x53<<1;
#endif

signed int accX=0, accY=0, accZ=0;						// raw accelerometer calibrated values
unsigned int absAccX = 0, absAccY = 0, absAccZ = 0;
unsigned int accOffsetX = 0;							// values obtained during the calibration process; acc = raw_acc - offset
unsigned int accOffsetY = 0;							// before calibration: values between -3g and +3g corresponds to values between 0 and 1024
unsigned int accOffsetZ = 0;							// after calibration: values between -3g and +3g corresponds to values between -512 and 512
signed int currentAngle = 0;							// current orientation of the robot extracted from both the x and y axes

/***************/
/*** VARIOUS ***/
/***************/
unsigned char myTimeout = 0;
unsigned int delayCounter = 0;

