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
int proximityResult[12] = {0};				// contains the values of the ambient - (ambient+reflected)
signed int proximityOffset[12] = {0};				// contains the calibration values
unsigned char adcSaveDataTo = 0;					// indicate where to save the currently sampled channel
unsigned char adcSamplingState = 0;					// indicate which channel to select
unsigned char rightChannelPhase = 0;				// right motor phase when the channel was selected
unsigned char leftChannelPhase = 0;					// left motor phase when the channel was selected
unsigned int batteryLevel = 0;
unsigned char measBattery = 0;
signed int currentProxValue = 0;
unsigned long proximitySum[12] = {0};
unsigned char proxUpdated = 0;
int proximityResultLinear[8] = {0};

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
signed int pwm_right_desired_to_control = 0;				// the pwm given by a command through IR or RF
signed int pwm_left_desired_to_control = 0;
unsigned int left_vel_sum = 0;						// sum of all the adc values (at the end they will be divided by the total number of samples received)
unsigned int right_vel_sum = 0;
signed int last_left_vel = 0;
signed int last_right_vel = 0;
signed long int max_pwm_right = MAX_MOTORS_PWM;		// if the period of the motors is changed at runtime, also the maximum pwm (60%) value
signed long int max_pwm_left = MAX_MOTORS_PWM;		// must be changed with these vars (the new values is period_motor_x/100*60)
signed int pwm_right = 0;						// value to set to the pwm register; this value is expressed in the range 0..PERIOD MOTORS
signed int pwm_left = 0;
signed int prev_pwm_right = 0;
signed int prev_pwm_left = 0;
//signed long int pwm_right_working = 0;				// current temporary pwm used in the controllers
//signed long int pwm_left_working = 0;
signed int pwm_right_desired = 0;				// the pwm given by a command through IR or RF
signed int pwm_left_desired = 0;				// this pwm is expressed in the range 0..MAX_MOTORS_PWM (1023 corresponds to 100% duty cycle)
//signed int p_speed_control = 25; //100;						// PID parameters
//unsigned int d_speed_control;
//signed int i_speed_control = 2;
//unsigned int i_limit_speed_control;				// maximum value for the I parameter
signed int k_ff_speed_control_left=INIT_KFF;
signed int k_ff_speed_control_right=INIT_KFF;
signed int pwm_right_speed_controller = 0;		// the pwm values after the speed controller adaptation
signed int pwm_left_speed_controller = 0;
signed int delta_left_speed = 0;
signed int delta_right_speed = 0;
signed int delta_left_speed_prev;
signed int delta_left_speed_current;
signed int delta_right_speed_prev;
signed int delta_right_speed_current;
signed int delta_left_speed_sum = 0;
signed int delta_right_speed_sum = 0;
//signed int current_angle = 0;
unsigned char compute_left_vel = 1;					// set at the beginning of the period to indicate that a new measure of the motor speed can be
unsigned char compute_right_vel = 1;
unsigned char left_vel_changed = 0;					// indicate that a new speed for the left motor is measured and ready to be used
unsigned char right_vel_changed = 0;				// the same for the right motor
signed int pwm_right_working = 0;				// current temporary pwm used in the controllers
signed int pwm_left_working = 0;
unsigned char update_pwm = 0;						// indicate that the controllers finished and thus the motors pwm can be updated
unsigned char firstSampleRight = 1;
unsigned char firstSampleLeft = 1;

/***********/
/*** NRF ***/
/***********/
unsigned int dataLED[3];							// array containing the value received through the radio
signed int speedl=0, speedr=0;					// current speed for left and right motors received through the radio
unsigned char rfData[PAYLOAD_SIZE];					// data received through the radio
unsigned char ackPayload[16];						// data to send back to the base-station
unsigned char packetId = 3;

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
unsigned char irEnabled = 1;
unsigned char checkGlitch = 1;
unsigned char behaviorState = 0;

/*********************/
/*** ACCELEROMETER ***/
/*********************/
unsigned char accelAddress = MMA7455L_ADDR;
unsigned char useAccel = USE_MMAX7455L;
signed int accX=0, accY=0, accZ=0;						// raw accelerometer calibrated values
unsigned int absAccX = 0, absAccY = 0, absAccZ = 0;
signed int accOffsetX = 0;							// values obtained during the calibration process; acc = raw_acc - offset
signed int accOffsetY = 0;							// before calibration: values between -3g and +3g corresponds to values between 0 and 1024
signed int accOffsetZ = 0;							// after calibration: values between -3g and +3g corresponds to values between -512 and 512
signed int currentAngle = 0;							// current orientation of the robot extracted from both the x and y axes
signed int accOffsetXSum = 0;
signed int accOffsetYSum = 0;
signed int accOffsetZSum = 0;
unsigned char prevPosition=HORIZONTAL_POS, currPosition=HORIZONTAL_POS;				// 1=horizontal, 0=vertical
unsigned char timesInSamePos = 0;
unsigned char robotPosition = 1;							// indicate whether the robot is in vertical (=0) or horizontal (=1) position

/***************/
/*** VARIOUS ***/
/***************/
unsigned char myTimeout = 0;
unsigned int delayCounter = 0;
unsigned char currentSelector = 0;
unsigned char startCalibration = 0;
signed int calibrationCycle = 0;

/**************************/
/*** OBSTACLE AVOIDANCE ***/
/**************************/
unsigned char obstacleAvoidanceEnabled = 0;
signed int rightProxSum=0;
signed int leftProxSum=0;

/***********************/
/*** CLIFF AVOIDANCE ***/
/***********************/
unsigned char cliffAvoidanceEnabled = 0;
unsigned int minGroundValue = 0;
unsigned int minGround = GROUND_LEFT;
unsigned char prevRot = 0;
unsigned char cliffDetectedFlag = 0;


