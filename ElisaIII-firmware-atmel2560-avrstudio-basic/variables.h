#include "constants.h"

/***********/
/*** ADC ***/
/***********/
extern volatile unsigned char currentAdChannel;
extern unsigned char currentProx;
extern unsigned char currentMotLeftChannel;
extern unsigned char currentMotRightChannel;
extern unsigned char rightMotorPhase;
extern unsigned char leftMotorPhase;
extern volatile unsigned int proximityValue[24];
extern int proximityResult[12];
extern signed int proximityOffset[12];
extern unsigned long proximitySum[12];
extern unsigned char adcSaveDataTo;
extern unsigned char adcSamplingState;
extern unsigned char rightChannelPhase;
extern unsigned char leftChannelPhase;
extern unsigned int batteryLevel;
extern unsigned char measBattery;
extern unsigned char proxUpdated;
extern int proximityResultLinear[8];
extern signed long int rightMotSteps;
extern signed long int leftMotSteps;

/******************************/
/*** CONSUMPTION CONTROLLER ***/
/******************************/
extern unsigned int left_current_avg;
extern unsigned int right_current_avg;
extern unsigned int last_left_current;
extern unsigned int last_right_current;

/************************/
/*** SPEED CONTROLLER ***/
/************************/
extern signed int pwm_right_desired_to_control;
extern signed int pwm_left_desired_to_control;
extern unsigned int left_vel_sum;
extern unsigned int right_vel_sum;
extern signed int last_left_vel;
extern signed int last_right_vel;
extern signed int pwm_right;
extern signed int pwm_left;
extern signed int pwm_right_desired;
extern signed int pwm_left_desired;
extern signed int k_ff_speed_control_left;
extern signed int k_ff_speed_control_right;
extern signed int pwm_right_speed_controller;
extern signed int pwm_left_speed_controller;
extern signed int delta_left_speed_current;
extern signed int delta_right_speed_current;
extern signed int delta_left_speed_prev;
extern signed int delta_right_speed_prev;
extern signed int delta_left_speed_sum;
extern signed int delta_right_speed_sum;
extern unsigned char compute_left_vel;
extern unsigned char compute_right_vel;
extern signed int pwm_right_working;
extern signed int pwm_left_working;
extern unsigned char firstSampleRight;
extern unsigned char firstSampleLeft;

/***********/
/*** NRF ***/
/***********/
extern unsigned int dataLED[3];
extern signed int speedl;
extern signed int speedr;
extern unsigned char rfData[PAYLOAD_SIZE];
extern unsigned char ackPayload[16];
extern unsigned char packetId;
extern unsigned int rfAddress;
extern unsigned char rfFlags;
extern unsigned char spiCommError;

/****************/
/*** RGB LEDS ***/
/****************/
extern unsigned char pwm_red;
extern unsigned char pwm_green;
extern unsigned char pwm_blue;
extern unsigned char blinkState;

/************/
/*** UART ***/
/************/
extern unsigned char peripheralChoice;
extern unsigned char choosePeripheral;
extern unsigned char sendAdcValues;
extern unsigned char commError;
extern unsigned int byteCount;
extern unsigned char uartBuff[UART_BUFF_SIZE];
extern unsigned char nextByteIndex;
extern unsigned char currByteIndex;

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
extern unsigned char irCommand;
extern unsigned char command_received;
extern unsigned char colorState;
extern unsigned char irEnabled;
extern unsigned char checkGlitch;
extern unsigned char behaviorState;

/*********************/
/*** ACCELEROMETER ***/
/*********************/
extern int accelAddress;
extern unsigned char useAccel;
extern signed int accX;
extern signed int accY;
extern signed int accZ;
extern signed int accOffsetX;
extern signed int accOffsetY;
extern signed int accOffsetXSum;
extern signed int accOffsetYSum;
extern signed int accXMax, accXMin, accYMax, accYMin;
extern signed int currentAngle;
extern unsigned char currPosition;
extern unsigned int timesInSamePos;
extern unsigned char robotPosition;
extern signed char accBuff[6];
extern unsigned temperature;

/***************/
/*** VARIOUS ***/
/***************/
extern uint32_t clockTick;
extern unsigned char currentSelector;
extern signed int calibrationCycle;
extern unsigned char startCalibration;
extern unsigned char hardwareRevision;
extern unsigned char currentOsccal;
extern uint32_t lastTick;

/**************************/
/*** OBSTACLE AVOIDANCE ***/
/**************************/
extern unsigned char obstacleAvoidanceEnabled;

/***********************/
/*** CLIFF AVOIDANCE ***/
/***********************/
extern unsigned char cliffAvoidanceEnabled;
extern unsigned char cliffDetectedFlag;

/****************/
/*** ODOMETRY ***/
/****************/
extern float thetaAcc;

