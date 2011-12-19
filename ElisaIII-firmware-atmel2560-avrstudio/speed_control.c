
#include "speed_control.h"

//--external variables--//
//extern signed int current_angle;
extern unsigned int last_left_vel;
extern unsigned int last_right_vel;
extern signed long int max_pwm_right;
extern signed long int max_pwm_left;
extern signed long int pwm_right_speed_controller;
extern signed long int pwm_left_speed_controller;
extern signed long int delta_left_speed;
extern signed long int delta_right_speed;
extern signed long int delta_left_speeds[2];
extern signed long int delta_right_speeds[2];
extern signed long int delta_left_speed_sum;
extern signed long int delta_right_speed_sum;
extern signed long int left_increment;
extern signed long int right_increment;

void init_speed_control() {
	//p_speed_control = 40;
	//d_speed_control = 5;
	//i_speed_control = 10;
	//i_limit_speed_control = 3200;
	//k_ff_speed_control_left = INIT_KFF;
	//k_ff_speed_control_right = INIT_KFF;
}

void start_vertical_speed_control(signed long int *pwm_left, signed long int *pwm_right) {

}

void start_orizzontal_speed_control(signed long int *pwm_left, signed long int *pwm_right) {

	signed long int pwm_left_working = *pwm_left;
	signed long int pwm_right_working = *pwm_right;

	if(*pwm_left==0 && *pwm_right==0)
		return;

	delta_right_speeds[1] = delta_right_speeds[0];
	if(pwm_right_working >= 0) {
		delta_right_speed = (signed long int)pwm_right_working - (signed long int)last_right_vel;
	} else {
		delta_right_speed = (signed long int)pwm_right_working + (signed long int)last_right_vel;
	}
	delta_right_speeds[0] = delta_right_speed;
	delta_right_speed_sum += delta_right_speed;

	if(delta_right_speed_sum > (signed long int)I_LIMIT_ORIZZONTAL ) {
		delta_right_speed_sum = (signed long int)I_LIMIT_ORIZZONTAL;
	}else if(delta_right_speed_sum < -(signed long int)I_LIMIT_ORIZZONTAL) {
		delta_right_speed_sum = -(signed long int)I_LIMIT_ORIZZONTAL;
	}

	right_increment = (signed long int)((float)P_ORIZZONTAL*((float)delta_right_speed) + (float)D_ORIZZONTAL*((float)(delta_right_speeds[0]-delta_right_speeds[1])) + (float)I_ORIZZONTAL*(float)delta_right_speed_sum);
     
	pwm_right_speed_controller = (signed long int)(K_FF_ORIZZONTAL*pwm_right_working) + right_increment;

	if(pwm_right_speed_controller < 0 && pwm_right_working >= 0) {	// avoid changing moving direction
		pwm_right_speed_controller = 0;
	}
	if(pwm_right_speed_controller > 0 && pwm_right_working < 0 ) {
		pwm_right_speed_controller = 0;
	}

	if (pwm_right_speed_controller>MAX_PWM) pwm_right_speed_controller=MAX_PWM;
	if (pwm_right_speed_controller<-MAX_PWM) pwm_right_speed_controller=-MAX_PWM;

	delta_left_speeds[1] = delta_left_speeds[0]; 
	if(pwm_left_working >= 0) {
		delta_left_speed = (signed long int)pwm_left_working - (signed long int)last_left_vel; 
	} else {
		delta_left_speed = (signed long int)pwm_left_working + (signed long int)last_left_vel; 
	}
	delta_left_speeds[0] = delta_left_speed;
	delta_left_speed_sum += delta_left_speed;

	if(delta_left_speed_sum > (signed long int)I_LIMIT_ORIZZONTAL) {
		delta_left_speed_sum = (signed long int)I_LIMIT_ORIZZONTAL;
	} else if(delta_left_speed_sum < -(signed long int)I_LIMIT_ORIZZONTAL) {
		delta_left_speed_sum = -(signed long int)I_LIMIT_ORIZZONTAL;
	}
   
	left_increment = (signed long int)((float)P_ORIZZONTAL*((float)delta_left_speed) + (float)D_ORIZZONTAL*((float)(delta_left_speeds[0]-delta_left_speeds[1])) + (float)I_ORIZZONTAL*(float)delta_left_speed_sum);
       
	pwm_left_speed_controller = (signed long int)(K_FF_ORIZZONTAL*pwm_left_working) + left_increment;

	if(pwm_left_speed_controller < 0 && pwm_left_working >= 0) {
		pwm_left_speed_controller = 0;
	}
	if(pwm_left_speed_controller > 0 && pwm_left_working < 0 ) {
		pwm_left_speed_controller = 0;
	}

	if (pwm_left_speed_controller>MAX_PWM) pwm_left_speed_controller=MAX_PWM;
	if (pwm_left_speed_controller<-MAX_PWM) pwm_left_speed_controller=-MAX_PWM;

	*pwm_left = pwm_left_speed_controller*MAX_MOTORS_PWM/MAX_PWM;
	*pwm_right = pwm_right_speed_controller*MAX_MOTORS_PWM/MAX_PWM;

}
