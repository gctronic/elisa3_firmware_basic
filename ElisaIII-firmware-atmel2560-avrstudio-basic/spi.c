/*
 * spi.c
 *
 *  Created on: 24-Jan-2009
 *      Author: Neil MacMillan
 *
 *  Functions for using the AT90 as an SPI master.
 */


#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define SPI_DDR DDRB	// DDR of SPI port
#define SPI_PORT PORTB	// SPI port
#define SPI_MOSI PORTB2	// MOSI pin (Master out, Slave in)
#define SPI_MISO PORTB3	// MISO pin (Master in, Slave out)
#define SPI_SCK PORTB1	// SCK pin (SPI clock)
#define SPI_SS PORTB0	// SS pin (Slave Select)

// wait for an SPI read/write operation to complete
//#define SPI_WAIT()              while ((SPSR & _BV(SPIF)) == 0);
void SPI_WAIT() {
	unsigned int timeout=0;
	while (1) {
		timeout++;
		if(timeout>=10000) {
			spiCommError = 1;
		}
	
		if(SPSR & _BV(SPIF)) {
			return;
		}
	}
}

void initSPI() {

    SPI_DDR &= ~((1<<SPI_MOSI)|(1<<SPI_MISO)|(1<<SPI_SS)|(1<<SPI_SCK));
    // Define the following pins as output
    SPI_DDR |= ((1<<SPI_MOSI)|(1<<SPI_SS)|(1<<SPI_SCK));

    
    SPCR = ((1<<SPE)|               // SPI Enable
            (0<<SPIE)|              // SPI Interupt Enable
            (0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
            (1<<MSTR)|              // 1:Master/ 0:Slave
            (0<<SPR1)|(0<<SPR0)|    // SPI Clock Rate => default 1/4 => 2 MHz
            (0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
            (0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)

    SPSR |= (1<<SPI2X);              // Double Clock Rate

}

void closeSPI() {

	SPCR = 0x00;
	SPSR = 0x00;
}

void SPI_ReadWrite_Block(uint8_t* data, uint8_t* buffer, uint8_t len) {
    uint8_t i;
    for (i = 0; i < len; i++) {
          SPDR = data[i];
          SPI_WAIT();
		  if(spiCommError) {
			return;
		  }
          buffer[i] = SPDR;
    }
}

void SPI_Write_Block(uint8_t* data, uint8_t len) {
    uint8_t i;
    for (i = 0; i < len; i++) {
          SPDR = data[i];
          SPI_WAIT();
		  if(spiCommError) {
			return;
		  }
    }

}

uint8_t SPI_Write_Byte(uint8_t byte) {
    SPDR = byte;
    SPI_WAIT();
    return SPDR;
}
