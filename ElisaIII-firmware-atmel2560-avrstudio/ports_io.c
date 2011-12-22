#include <avr\io.h>

void initPortsIO(void) {

	DDRA = 0xFF;	// proximity pulses as output
	PORTA = 0x00;	// proximity pulses turned off
	
	DDRB = 0xF7;	// pwm for led r/g/b as output; CE, MOSI, SCK, SS as output (master) 
	PORTB = 0xE0;
	//PORTB |= (1 << 5) | (1 << 6) | (1 << 7); // leds off	
	//PORTB &= ~(1 << 5) & ~(1 << 6) & ~(1 << 7); // leds off
	//PORTB &= ~(1 << 7);

	DDRC = 0xF0;	// selector as input; IR leds as output; sens-enable, sleep as output
	PORTC = 0xB0;	// sleep = 1, IR leds = 1

	DDRD = 0xFC;	// all pins to output; when usart and i2c peripherals are activated they change the pins direction accordingly
	PORTD = 0x03;	// default for unused pins is 0

	//DDRE = 0x18;	// pwm and dir for motor right as output; when usart is activated it changes the pins direction accordingly
	DDRE = 0xFF;	// all pins to output
	PORTE = 0x00;	// default for unused pins is 0; pwm for motors set to 0 when stopped

	DDRF = 0x00;	// adc channel pins as input		

	DDRG = 0xFF;	// unused pins as output
	PORTG = 0x00;	// default for unused pins is 0
	
	//DDRH = 0x58;	// pwm and dir for motor left as output; when usart is activated it changes the pins direction accordingly
	DDRH = 0xFF;	// all pins to output
	PORTH = 0x00;	// default for unused pins is 0; pwm for motors set to 0 when stopped

	DDRJ = 0x8F;	// cliff pulses as output; charge-on, button0, remote as input; unused as output
	PORTJ = 0x00;	// cliff pulse turned off

	DDRK = 0x00;	// adc channel pins as input

	DDRL = 0xFF;	// all pins to output
	PORTL = 0x00;	// pwm (unused) and unused pins to 0

		
}
