
#include "utility.h"

unsigned char getSelector() {
   return (SEL0) + 2*(SEL1) + 4*(SEL2) + 8*(SEL3);
}
    

void initPeripherals(void) {

	cli();			// disable global interrupts (by default it should already be disabled)
	
	// reset all registers touched by arduino in the "init()" functions (wiring.c) not used by the robot
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;
	TCCR5A = 0;
	TCCR5B = 0;

	rfAddress = eeprom_read_word((uint16_t*)4094);

	// some code parts change based on hardware revision
	if(rfAddress >= 3201 && rfAddress <= 3203) {
		hardwareRevision = HW_REV_3_0;
	}

	if(rfAddress == 3200) {
		hardwareRevision = HW_REV_3_0_1;
	}

	if(rfAddress > 3203) {
		hardwareRevision = HW_REV_3_1;
	}

	initPortsIO();
	initAdc();
	initMotors();
	initRGBleds();
	initSPI();
	mirf_init();
	initUsart();
	initAccelerometer();
	init_ir_remote_control();

	sei();			// enable global interrupts

	
}

// used only for wake-up from sleep
ISR(TIMER2_OVF_vect) {

}

void sleep(unsigned char seconds) {

	unsigned int pause = seconds*30;	// the timer2 used to wake-up from sleep is configured to run at 30 Hz

	// disable external interrupt because it uses the timer2 to interpret the tv
	// remote signal and the timer2 must be free in order to be used for wake-up from sleep
	PCICR &= ~(1 << PCIE1);			// disable interrupt from falling edge
	PCMSK1 &= ~(1 << PCINT15);		
	PCIFR |= (1 << PCIF1);			// clear interrupt flag

	// disable adc
	ADCSRA = 0x00;					// disable interrupt and turn off adc
	ADCSRA |= (1 << ADIF);			// clear interrupt flag

	// disable motors pwm
	TCCR3A = 0x00;	// turn off timer
	TCCR3B = 0x00;
	TIMSK3 = 0x00;	// disable interrupt
	TIFR3 |= (1 << OCF3A) | (1 << OCF3B) | (1 << TOV3);	// clear output compares and timer overflow interrupt flags
	TCCR4A = 0x00;
	TCCR4B = 0x00;
	TIMSK4 = 0x00;
	TIFR4 |= (1 << OCF4A) | (1 << OCF4B) | (1 << TOV4);	// clear output compares and timer overflow interrupt flags

	// disable leds pwm
	TCCR1A = 0x00;	// turn off timer
	TCCR1B = 0x00;

	// close communication channels
	closeUsart();
	closeSPI();
	i2c_close();

	// set port pins
	initPortsIO();
	//PORTC &= ~(1 << 7); // sleep pin => not supported by microcontroller (adc voltage reference restrictions)

	//PORTB &= ~(1 << 4);	// radio CE pin
	//DDRD = 0xFF;
	//PORTD = 0x00;	// I2C and uart pins to 0

	// set extendend standby mode and enable it
	SMCR |= (1 << SM2) | (1 << SM1) | (1 << SM0) | (1 << SE);	// extended standby
	//SMCR |= (1 << SM1) | (1 << SE);	// power-down mode
	//SMCR |= (1 << SE);	// idle mode

	// set timer2 for wake-up: 
	// source clock = 8 MHz
	// prescaler = 1/1024 => 7812.5 Hz
	// max delay = 7812.5 / 256 = about 30 Hz (33 ms)
	TIMSK2 = 0x01; //(1 << TOIE2);
	TCCR2A &= ~(1 << WGM21); 	// mode 0 => normal mode
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);	// 1/1024 prescaler

	while(pause > 0) {	
		// enter extended standby mode
		//sleep_cpu();
		__asm__("sleep");
		pause--;
//		PORTB ^= (1 << 6);
	}

	// disable power mode
	//SMCR &= ~(1 << SE);
	SMCR = 0x00;

	// disable timer2 and its timer overflow interrupt
	TCCR2B &= ~(1 << CS22) &~(1 << CS21) &~(1 << CS20);	// disable timer2
	TIMSK2 = 0;					// disable all interrupt for timer2
	TCCR2A |= (1 << WGM21); 	// mode 2 => CTC mode

	pwm_red = 255;
	pwm_green = 255;
	pwm_blue = 255;
	pwm_right = 0;
	pwm_left = 0;
	initPeripherals();

}

unsigned long int getTime100MicroSec() {
	return clockTick;
}

void readBatteryLevel() {
	measBattery = 1;
}

