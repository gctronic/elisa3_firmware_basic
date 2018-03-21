#include "constants.h"

/***********/
/*** ADC ***/
/***********/
volatile unsigned char currentAdChannel = 0;		// next channel to be sampled (channel changes at the adc interrupt)
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
int proximityResult[12] = {0};						// contains the calibrated values => ambient - (ambient+reflected)
signed int proximityOffset[12] = {0};				// contains the calibration values
unsigned long proximitySum[12] = {0};				// contains the sum of the sensor values during calibration (this value will then be divided by the number
													// of samples taken to get the calibration offsets)
unsigned char adcSaveDataTo = 0;					// indicate where to save the currently sampled channel
unsigned char adcSamplingState = 0;					// indicate which channel to select
unsigned char rightChannelPhase = 0;				// right motor phase when the channel was selected
unsigned char leftChannelPhase = 0;					// left motor phase when the channel was selected
unsigned int batteryLevel = 0;						// level of the battery sampled
unsigned char measBattery = 0;						// flag indicating when the battery is sampled (once every 2 second at the moment)
unsigned char proxUpdated = 0;						// flag indicating that all the sensors (proximity and ground) got a new value
int proximityResultLinear[8] = {0};					// array containing the linearized values for the proximity senosrs (used in obstacle avoidance)
signed long int rightMotSteps = 0;
signed long int leftMotSteps = 0;

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
signed int pwm_right_desired_to_control = 0;		// the pwm given by the last command through IR or RF; this value is passed to the speed controller and 
signed int pwm_left_desired_to_control = 0;			// it is not changed untill the next time the speed controller is called (even if a new command is received
													// during speed controller computation)
unsigned int left_vel_sum = 0;						// sum of all the adc values (at the end they will be divided by the total number of samples received)
unsigned int right_vel_sum = 0;
signed int last_left_vel = 0;						// last measured velocity (average of the sum)
signed int last_right_vel = 0;
signed int pwm_right = 0;							// value to set to the pwm register; this value is expressed in the range 0..MAX_MOTORS_PWM
signed int pwm_left = 0;
signed int pwm_right_desired = 0;					// the pwm given by a command through IR or RF
signed int pwm_left_desired = 0;					// this pwm is expressed in the range 0..MAX_MOTORS_PWM (1023 corresponds to 100% duty cycle)
//signed int p_speed_control = 25;					// PID parameters used during debugging to change dynamically these parameters
//unsigned int d_speed_control;
//signed int i_speed_control = 2;
//unsigned int i_limit_speed_control;
signed int k_ff_speed_control_left=INIT_KFF;		// feed forward term for the vertical speed controller
signed int k_ff_speed_control_right=INIT_KFF;
signed int pwm_right_speed_controller = 0;			// the pwm values after the speed controller adaptation
signed int pwm_left_speed_controller = 0;			
signed int delta_left_speed_current;				// current error between desired and measured speed
signed int delta_right_speed_current;
signed int delta_left_speed_prev;					// previous error between desired and measured speed  (used with the D term)
signed int delta_right_speed_prev;					
signed int delta_left_speed_sum = 0;				// sum of the errors (used with the I term)
signed int delta_right_speed_sum = 0;
unsigned char compute_left_vel = 1;					// flag indicating that enough samples are taken for computing the current velocity (during passive phase of motors pwm)
unsigned char compute_right_vel = 1;
signed int pwm_right_working = 0;					// current temporary pwm used in the controllers
signed int pwm_left_working = 0;
unsigned char firstSampleRight = 1;					// flag indicating the beginning of the passive phase of the motors pwm
unsigned char firstSampleLeft = 1;

/***********/
/*** NRF ***/
/***********/
unsigned int dataLED[3];							// array containing the value received through the radio
signed int speedl=0, speedr=0;						// current speed for left and right motors received through the radio (absolute value)
unsigned char rfData[PAYLOAD_SIZE];					// data received through the radio
unsigned char ackPayload[16];						// data to send back to the base-station
unsigned char packetId = 3;
unsigned int rfAddress = 0;							// this number define the robot ID, used also as the robot
													// RF address; from this number is then obtained the hardware revision
unsigned char rfFlags = 0;							// bit0: 1 = spi comm. ok, 0 = spi comm. not ok
													// bit1: 1 = rf comm. ok, 0 = rf comm. not ok
unsigned char spiCommError=0;

/****************/
/*** RGB LEDS ***/
/****************/
unsigned char pwm_red = 255, pwm_green = 255, pwm_blue = 255;	// value to set to the pwm registers; this value is expressed in the range 0 (max power) to 255 (off)
unsigned char blinkState = 0;									// used to toggle the blue led

/************/
/*** UART ***/
/************/
unsigned char peripheralChoice = 5;					// red led=0, green led=1, blue led=2, right motor=3, left motor=4, send adc values=5
unsigned char choosePeripheral = 1;					// uart state: choose the peripheral (1) or act on choosen peripheral (0)
unsigned char sendAdcValues = 0;
unsigned char commError = 0;
unsigned int byteCount = 0;
unsigned char uartBuff[UART_BUFF_SIZE] = {0};
unsigned char nextByteIndex = 0;
unsigned char currByteIndex = 0;

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
unsigned char irCommand = 0;						// command received through TV remote
unsigned char command_received = 0;					// flag indicating that a new command from TV remote is received
unsigned char colorState = 0;						// used with command 0 to switch from one color to the next
unsigned char irEnabled = 1;						// flag indicating whether the TV remote commands are interpreted or discarded
unsigned char checkGlitch = 1;						// flag indicating the phase in which the signal is checked for glitches
unsigned char behaviorState = 0;					// used to switch between behaviors (for small remote control)

/*********************/
/*** ACCELEROMETER ***/
/*********************/
unsigned char accelAddress = MMA7455L_ADDR;			// accelerometer I2C communication address
unsigned char useAccel = USE_MMAX7455L;				// flag indicatin which accelerometer (or none) is active
signed int accX=0, accY=0, accZ=0;					// accelerometer calibrated values
signed int accOffsetX = 0;							// values obtained during the calibration process; acc = raw_acc - offset
signed int accOffsetY = 0;							// before calibration: values between -3g and +3g corresponds to values between 0 and 1024
													// after calibration: values between -3g and +3g corresponds to values between -512 and 512
signed int accOffsetXSum = 0;						// contains the sum of the accelerometer values during calibration (these values will then be 
signed int accOffsetYSum = 0;						// divided by the number of samples taken to get the calibration offsets)
signed int accXMax=0, accXMin=0, accYMax=0, accYMin=0;
signed int currentAngle = 0;						// current orientation of the robot (in a vertical wall) extracted from both the x and y axes
unsigned char currPosition=HORIZONTAL_POS;			//  "currPosition" is used to change the "robotPosition" in a smoother way
unsigned int timesInSamePos = 0;					// number of cycles in which the new robot position remain stable; after some stability it will 
													// change the current "robotPosition"
unsigned char robotPosition = 1;					// indicate whether the robot is in vertical (=0) or horizontal (=1) position
signed char accBuff[6] = {0};
unsigned temperature = 0;

/***************/
/*** VARIOUS ***/
/***************/
uint32_t clockTick = 0;					// this is the base time, each tick corresponds to 104 us (incremented inside adc isr);
													// beware that this variable is never reset (4294967295/10000/60/60/24 = about 5 days before overflow)
unsigned char currentSelector = 0;					// current selector position
signed int calibrationCycle = 0;					// indicate how many samples are currently taken for calibration
unsigned char startCalibration;						// flag indicating when a calibration is in progress
unsigned char hardwareRevision = HW_REV_3_0;		// hardware revision based on the address saved in eeprom
unsigned char currentOsccal;
uint32_t lastTick = 0;

/**************************/
/*** OBSTACLE AVOIDANCE ***/
/**************************/
unsigned char obstacleAvoidanceEnabled = 0;			// flag indicating that obstacle avoidance is enabled

/***********************/
/*** CLIFF AVOIDANCE ***/
/***********************/
unsigned char cliffAvoidanceEnabled = 0;			// flag indicating that cliff avoidance is enabled
unsigned char cliffDetectedFlag = 0;				// flag indicating a cliff is detected => stop the motors

/****************/
/*** ODOMETRY ***/
/****************/
float thetaAcc = 0.0;


