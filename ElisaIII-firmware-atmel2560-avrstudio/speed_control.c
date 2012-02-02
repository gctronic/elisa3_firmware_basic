
#include "speed_control.h"

//--external variables--//
extern signed int last_left_vel;
extern signed int last_right_vel;
extern signed long int max_pwm_right;
extern signed long int max_pwm_left;
extern signed int pwm_right_speed_controller;
extern signed int pwm_left_speed_controller;
extern signed int delta_left_speed_prev;
extern signed int delta_left_speed_current;
extern signed int delta_right_speed_prev;
extern signed int delta_right_speed_current;
extern signed int delta_left_speed_sum;
extern signed int delta_right_speed_sum;
extern signed int currentAngle;
extern signed int k_ff_speed_control_left;
extern signed int k_ff_speed_control_right;

void init_speed_control() {
	//p_speed_control = 40;
	//d_speed_control = 5;
	//i_speed_control = 10;
	//i_limit_speed_control = 3200;
	//k_ff_speed_control_left = INIT_KFF;
	//k_ff_speed_control_right = INIT_KFF;
}

void start_vertical_speed_control_left(signed int *pwm_left) {

	if(*pwm_left==0) {
		delta_left_speed_sum = 0;
		delta_left_speed_current = 0;
		delta_left_speed_prev = 0;
		return;
	}

	// change the feedforward based on the current measured angle
	if(currentAngle >= 270) {			// pointing down-right
		if(*pwm_left > 0) {
			k_ff_speed_control_left = INIT_KFF - ((360 - currentAngle)>>2);
		} else {
			k_ff_speed_control_left = INIT_KFF + ((360 - currentAngle)>>2);
		}
	} else if(currentAngle >= 180) {	// pointing down-left
		if(*pwm_left > 0) {
			k_ff_speed_control_left = INIT_KFF + ((180 - currentAngle)>>2);
		} else {
			k_ff_speed_control_left = INIT_KFF - ((180 - currentAngle)>>2);
		}
	} else if(currentAngle >= 90) {	// pointing up-left
		if(*pwm_left > 0) {
			k_ff_speed_control_left = INIT_KFF + ((180 - currentAngle)>>2);
		} else {
			k_ff_speed_control_left = INIT_KFF - ((180 - currentAngle)>>2);
		}
	} else {							// pointing up-right
		if(*pwm_left > 0) {
			k_ff_speed_control_left = INIT_KFF + (currentAngle>>2);
		} else {
			k_ff_speed_control_left = INIT_KFF - (currentAngle>>2);
		}
	}

	delta_left_speed_prev = delta_left_speed_current; 
	if(*pwm_left >= 0) {
		delta_left_speed_current = (*pwm_left) - last_left_vel; 
	} else {
		delta_left_speed_current = (*pwm_left) + last_left_vel; 
	}
	delta_left_speed_sum += delta_left_speed_current;

	if(delta_left_speed_sum > I_LIMIT_VERTICAL) {
		delta_left_speed_sum = I_LIMIT_VERTICAL;
	} else if(delta_left_speed_sum < -I_LIMIT_VERTICAL) {
		delta_left_speed_sum = -I_LIMIT_VERTICAL;
	}
	    
	pwm_left_speed_controller = (signed int)(k_ff_speed_control_left*(*pwm_left)); //((*pwm_left) << 3); //<< 5);
	pwm_left_speed_controller += (signed int)(P_VERTICAL * delta_left_speed_current);
	pwm_left_speed_controller -= (signed int)((delta_left_speed_current-delta_left_speed_prev)*D_VERTICAL);
	pwm_left_speed_controller += (signed int)(I_VERTICAL*delta_left_speed_sum);

	if(pwm_left_speed_controller < 0 && *pwm_left >= 0) {
		pwm_left_speed_controller = 0;
	}
	if(pwm_left_speed_controller > 0 && *pwm_left < 0 ) {
		pwm_left_speed_controller = 0;
	}

	if (pwm_left_speed_controller>MAX_PWM) pwm_left_speed_controller=MAX_PWM;
	if (pwm_left_speed_controller<-MAX_PWM) pwm_left_speed_controller=-MAX_PWM;

	//pwm_left_speed_controller = pwm_left_speed_controller*(MAX_MOTORS_PWM/2)/MAX_PWM;
	// since the pwm_left_speed_controller goes from 0 to 24000 then the pwm_left goes
	// from 0 to 375
	*pwm_left = ((signed int)pwm_left_speed_controller)>>4; //>>6;

	if (*pwm_left>(MAX_MOTORS_PWM/2)) *pwm_left=(MAX_MOTORS_PWM/2);
    if (*pwm_left<-(MAX_MOTORS_PWM/2)) *pwm_left=-(MAX_MOTORS_PWM/2);


}


void start_vertical_speed_control_right(signed int *pwm_right) {

	if(*pwm_right==0) {
		delta_right_speed_sum = 0;
		delta_right_speed_current = 0;
		delta_right_speed_prev = 0;
		return;
	}

	// change the feedforward based on the current measured angle
	if(currentAngle >= 270) {			// pointing down-right
		if(*pwm_right > 0) {
			k_ff_speed_control_right = INIT_KFF - ((360 - currentAngle)>>2);
		} else {
			k_ff_speed_control_right = INIT_KFF + ((360 - currentAngle)>>2);
		}
	} else if(currentAngle >= 180) {	// pointing down-left
		if(*pwm_right > 0) {
			k_ff_speed_control_right = INIT_KFF + ((180 - currentAngle)>>2);
		} else {
			k_ff_speed_control_right = INIT_KFF - ((180 - currentAngle)>>2);
		}
	} else if(currentAngle >= 90) {	// pointing up-left
		if(*pwm_right > 0) {
			k_ff_speed_control_right = INIT_KFF + ((180 - currentAngle)>>2);
		} else {
			k_ff_speed_control_right = INIT_KFF - ((180 - currentAngle)>>2);
		}
	} else {							// pointing up-right
		if(*pwm_right > 0) {
			k_ff_speed_control_right = INIT_KFF + (currentAngle>>2);
		} else {
			k_ff_speed_control_right = INIT_KFF - (currentAngle>>2);
		}
	}

	delta_right_speed_prev = delta_right_speed_current;
	if(*pwm_right >= 0) {
		delta_right_speed_current = (*pwm_right) - last_right_vel;
	} else {
		delta_right_speed_current = (*pwm_right) + last_right_vel;
	}
	delta_right_speed_sum += delta_right_speed_current;

	if(delta_right_speed_sum > I_LIMIT_VERTICAL ) {
		delta_right_speed_sum = I_LIMIT_VERTICAL;
	}else if(delta_right_speed_sum < -I_LIMIT_VERTICAL) {
		delta_right_speed_sum = -I_LIMIT_VERTICAL;
	}

	pwm_right_speed_controller = (signed int)(k_ff_speed_control_right*(*pwm_right)); //(signed int)((*pwm_right) << 3); //<< 5);
	pwm_right_speed_controller += (signed int)(P_VERTICAL * delta_right_speed_current);
	pwm_right_speed_controller -= (signed int)((delta_right_speed_current-delta_right_speed_prev)*D_VERTICAL);
	pwm_right_speed_controller += (signed int)(I_VERTICAL*delta_right_speed_sum);

	if(pwm_right_speed_controller < 0 && *pwm_right >= 0) {	// avoid changing moving direction
		pwm_right_speed_controller = 0;
	}
	if(pwm_right_speed_controller > 0 && *pwm_right < 0 ) {
		pwm_right_speed_controller = 0;
	}

	if (pwm_right_speed_controller>MAX_PWM) pwm_right_speed_controller=MAX_PWM;
	if (pwm_right_speed_controller<-MAX_PWM) pwm_right_speed_controller=-MAX_PWM;

	//pwm_right_speed_controller = pwm_right_speed_controller*(MAX_MOTORS_PWM/2)/MAX_PWM;
	*pwm_right = ((signed int)pwm_right_speed_controller)>>4; //>>6;

	if (*pwm_right>(MAX_MOTORS_PWM/2)) *pwm_right=(MAX_MOTORS_PWM/2);
    if (*pwm_right<-(MAX_MOTORS_PWM/2)) *pwm_right=-(MAX_MOTORS_PWM/2);

}

void start_orizzontal_speed_control_right(signed int *pwm_right) {

	if(*pwm_right==0) {
		delta_right_speed_sum = 0;
		delta_right_speed_current = 0;
		delta_right_speed_prev = 0;
		return;
	}

	delta_right_speed_prev = delta_right_speed_current;
	if(*pwm_right >= 0) {
		delta_right_speed_current = (*pwm_right) - last_right_vel;
	} else {
		delta_right_speed_current = (*pwm_right) + last_right_vel;
	}
	delta_right_speed_sum += delta_right_speed_current;

	if(delta_right_speed_sum > I_LIMIT_ORIZZONTAL ) {
		delta_right_speed_sum = I_LIMIT_ORIZZONTAL;
	}else if(delta_right_speed_sum < -I_LIMIT_ORIZZONTAL) {
		delta_right_speed_sum = -I_LIMIT_ORIZZONTAL;
	}

	pwm_right_speed_controller = (signed int)((*pwm_right) << 3); //<< 5);
	pwm_right_speed_controller += (signed int)(delta_right_speed_current*P_ORIZZONTAL);
	pwm_right_speed_controller -= (signed int)((delta_right_speed_current-delta_right_speed_prev)*D_ORIZZONTAL);
	pwm_right_speed_controller += (signed int)(delta_right_speed_sum*I_ORIZZONTAL);

	if(pwm_right_speed_controller < 0 && *pwm_right >= 0) {	// avoid changing moving direction
		pwm_right_speed_controller = 0;
	}
	if(pwm_right_speed_controller > 0 && *pwm_right < 0 ) {
		pwm_right_speed_controller = 0;
	}

	if (pwm_right_speed_controller>MAX_PWM) pwm_right_speed_controller=MAX_PWM;
	if (pwm_right_speed_controller<-MAX_PWM) pwm_right_speed_controller=-MAX_PWM;

	//pwm_right_speed_controller = pwm_right_speed_controller*(MAX_MOTORS_PWM/2)/MAX_PWM;
	*pwm_right = ((signed int)pwm_right_speed_controller)>>4; //>>6;
	if(*pwm_right > 0) {
		*pwm_right += 30;
	} else if(*pwm_right < 0) {
		*pwm_right -= 30;
	}

	if (*pwm_right>(MAX_MOTORS_PWM/2)) *pwm_right=(MAX_MOTORS_PWM/2);
    if (*pwm_right<-(MAX_MOTORS_PWM/2)) *pwm_right=-(MAX_MOTORS_PWM/2);

}

void start_orizzontal_speed_control_left(signed int *pwm_left) {

	if(*pwm_left==0) {
		delta_left_speed_sum = 0;
		delta_left_speed_current = 0;
		delta_left_speed_prev = 0;
		return;
	}

	delta_left_speed_prev = delta_left_speed_current; 
	if(*pwm_left >= 0) {
		delta_left_speed_current = (*pwm_left) - last_left_vel; 
	} else {
		delta_left_speed_current = (*pwm_left) + last_left_vel; 
	}
	delta_left_speed_sum += delta_left_speed_current;

	if(delta_left_speed_sum > I_LIMIT_ORIZZONTAL) {
		delta_left_speed_sum = I_LIMIT_ORIZZONTAL;
	} else if(delta_left_speed_sum < -I_LIMIT_ORIZZONTAL) {
		delta_left_speed_sum = -I_LIMIT_ORIZZONTAL;
	}
	    
	pwm_left_speed_controller = (signed int)((*pwm_left) << 3); //<< 5);
	pwm_left_speed_controller += (signed int)(delta_left_speed_current*P_ORIZZONTAL);
	pwm_left_speed_controller -= (signed int)((delta_left_speed_current-delta_left_speed_prev)*D_ORIZZONTAL);
	pwm_left_speed_controller += (signed int)(delta_left_speed_sum*I_ORIZZONTAL);

	if(pwm_left_speed_controller < 0 && *pwm_left >= 0) {
		pwm_left_speed_controller = 0;
	}
	if(pwm_left_speed_controller > 0 && *pwm_left < 0 ) {
		pwm_left_speed_controller = 0;
	}

	if (pwm_left_speed_controller>MAX_PWM) pwm_left_speed_controller=MAX_PWM;
	if (pwm_left_speed_controller<-MAX_PWM) pwm_left_speed_controller=-MAX_PWM;

	//pwm_left_speed_controller = pwm_left_speed_controller*(MAX_MOTORS_PWM/2)/MAX_PWM;
	// since the pwm_left_speed_controller goes from 0 to 24000 then the pwm_left goes
	// from 0 to 375
	*pwm_left = ((signed int)pwm_left_speed_controller)>>4; //>>6;
	if(*pwm_left > 0) {
		*pwm_left += 30;
	} else if(*pwm_left < 0) {
		*pwm_left -= 30;
	}

	if (*pwm_left>(MAX_MOTORS_PWM/2)) *pwm_left=(MAX_MOTORS_PWM/2);
    if (*pwm_left<-(MAX_MOTORS_PWM/2)) *pwm_left=-(MAX_MOTORS_PWM/2);

}

