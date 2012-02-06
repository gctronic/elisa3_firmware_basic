#include "constants.h"


// adc
extern volatile unsigned char currentAdChannel;
extern unsigned char currentProx;
extern unsigned char currentMotLeftChannel;
extern unsigned char currentMotRightChannel;
extern unsigned char rightMotorPhase;
extern unsigned char leftMotorPhase;
extern volatile unsigned int proximityValue[24];
extern unsigned char adcSaveDataTo;
extern unsigned char adcSamplingState;
extern unsigned char rightChannelPhase;
extern unsigned char leftChannelPhase;
extern unsigned int batteryLevel;
extern unsigned char measBattery;
extern signed int currentProxValue;
extern int proximityResult[12];
extern unsigned int proximityOffset[12];
extern unsigned char updateProx;
extern unsigned long proximitySum[12];
extern unsigned char proxUpdated;

// consumption controller
extern unsigned int left_current_avg;
extern unsigned int right_current_avg;
extern unsigned int last_left_current;
extern unsigned int last_right_current;

// speed controller
extern signed int pwm_right_desired_to_control;
extern signed int pwm_left_desired_to_control;
extern unsigned int left_vel_sum;
extern unsigned int right_vel_sum;
extern signed int pwm_right;
extern signed int pwm_left;
extern unsigned char compute_left_vel;
extern unsigned char compute_right_vel;
extern signed int pwm_right_desired;
extern signed int pwm_left_desired;
extern unsigned char left_vel_changed;
extern unsigned char right_vel_changed;
extern signed int last_left_vel;
extern signed int last_right_vel;
extern signed int pwm_right_working;
extern signed int pwm_left_working;
extern unsigned char update_pwm;
extern unsigned char firstSampleRight;
extern unsigned char firstSampleLeft;
extern signed int prev_pwm_right;
extern signed int prev_pwm_left;

// uart
extern unsigned char peripheralChoice;
extern unsigned char choosePeripheral;
extern unsigned char sendAdcValues;

// rgb leds
extern unsigned char pwm_red;
extern unsigned char pwm_green;
extern unsigned char pwm_blue;
extern unsigned char blinkState;

// nrf
extern unsigned int dataLED[3];
extern signed int speedl;
extern signed int speedr;
extern unsigned char rfData[PAYLOAD_SIZE];
extern unsigned char ackPayload[16];
extern unsigned char packetId;

// various
extern unsigned char myTimeout;
extern unsigned int delayCounter;
extern unsigned char currentSelector;
extern unsigned char startCalibration;
extern signed int calibrationCycle;

// ir remote control
extern unsigned char ir_move;
extern unsigned char command_received;
extern unsigned char colorState;
extern unsigned char irEnabled;
extern unsigned char behaviorState;
extern unsigned char checkGlitch;

// accelerometer
extern int accelAddress;
extern signed int accX;
extern signed int accY;
extern signed int accZ;
extern unsigned int absAccX, absAccY, absAccZ;
extern signed int accOffsetX;							// values obtained during the calibration process; acc = raw_acc - offset
extern signed int accOffsetY;							// before calibration: values between -3g and +3g corresponds to values between 0 and 1024
extern signed int accOffsetZ;							// after calibration: values between -3g and +3g corresponds to values between -512 and 512
extern signed int currentAngle;							// current orientation of the robot extracted from both the x and y axes
extern unsigned char useAccel;
extern signed int accOffsetXSum;
extern signed int accOffsetYSum;
extern signed int accOffsetZSum;
extern unsigned char prevPosition; 
extern unsigned char currPosition;
extern unsigned char timesInSamePos;
extern unsigned char robotPosition;

// obstacle avoidance
extern unsigned char obstacleAvoidanceEnabled;
extern signed int rightProxSum;
extern signed int leftProxSum;

// cliff avoidance
extern unsigned char cliffAvoidanceEnabled;
extern unsigned int minGroundValue;
extern unsigned int minGround;
extern unsigned char prevRot;

