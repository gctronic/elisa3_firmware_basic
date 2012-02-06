/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#include "mirf.h"
#include "nRF24L01.h"
#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )


// Flag which denotes transmitting mode
volatile uint8_t PTX;

void mirf_init() 
// Initializes pins as interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
    // Define CSN and CE as Output and set them to default
    //DDRB |= ((1<<CSN)|(1<<CE));
    mirf_CE_hi;
    mirf_CSN_hi;

	mirf_config();
}


void mirf_config() 
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
{

	uint8_t temp[3];

	// power down
	mirf_config_register(CONFIG, 0x0D);

	// address width
	mirf_config_register(SETUP_AW, 0x01);

	// tx address
	temp[0] = (RF_ADDR>>8)&0xFF;
	temp[1] = RF_ADDR & 0xFF;
	temp[2] = 0x00;
	mirf_write_register(TX_ADDR, temp, 3);	

	// rx address => same as tx address for auto ack
	mirf_write_register(RX_ADDR_P0, temp, 3);

	// enable auto ack for pipe0
	mirf_config_register(EN_AA, 0x01);

	// enable pipe0
	mirf_config_register(EN_RXADDR, 0x01);

	// 500�s (+ 86�s on-air), 2 re-transmissions
	mirf_config_register(SETUP_RETR, 0x12);

    // select RF channel
    mirf_config_register(RF_CH,40);

	// RX payload size; it isn't needed because the dynamic payload length is activated for ACK+PAYLOAD feature
    mirf_config_register(RX_PW_P0, PAYLOAD_SIZE);

	// enable extra features
    mirf_CSN_lo;
    SPI_Write_Byte(NRF_ACTIVATE);
    SPI_Write_Byte(0x73);
    mirf_CSN_hi;
	
	// enable dynamic payload for pipe0
	mirf_config_register(NRF_DYNPD, 0x01);

	// enable payload with ACK and dynamic payload length
	mirf_config_register(NRF_FEATURE, 0x06);
		
	// power up; enable crc (2 bytes); prx; max_rt, tx_ds enabled
	mirf_config_register(CONFIG, 0x0F);	

    // Start receiver 
    //PTX = 0;        // Start in receiving mode
    //RX_POWERUP;     // Power up in receiving mode
    //mirf_CE_hi;     // Listening for pakets
}

void mirf_set_RADDR(uint8_t * adr) 
// Sets the receiving address
{
    mirf_CE_lo;
    mirf_write_register(RX_ADDR_P0,adr,5);
    mirf_CE_hi;
}

void mirf_set_TADDR(uint8_t * adr)
// Sets the transmitting address
{
    mirf_write_register(TX_ADDR, adr,5);
}

/*
#if defined(__AVR_ATmega8__)
SIGNAL(SIG_INTERRUPT0) 
#endif // __AVR_ATmega8__
#if defined(__AVR_ATmega168__)
SIGNAL(SIG_PIN_CHANGE2) 
#endif // __AVR_ATmega168__  
// Interrupt handler 
{
    uint8_t status;   
    // If still in transmitting mode then finish transmission
    if (PTX) {
    
        // Read MiRF status 
        mirf_CSN_lo;                                // Pull down chip select
        status = spi_fast_shift(NOP);               // Read status register
        mirf_CSN_hi;                                // Pull up chip select

        mirf_CE_lo;                             // Deactivate transreceiver
        RX_POWERUP;                             // Power up in receiving mode
        mirf_CE_hi;                             // Listening for pakets
        PTX = 0;                                // Set to receiving mode

        // Reset status register for further interaction
        mirf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)); // Reset status register
    }
}
*/

uint8_t mirf_data_ready() 
// Checks if data is available for reading
{
    if (PTX) return 0;
    uint8_t status;
    // Read MiRF status 
    mirf_CSN_lo;                                // Pull down chip select
    status = SPI_Write_Byte(NOP);               // Read status register
    mirf_CSN_hi;                                // Pull up chip select
    return status & (1<<RX_DR);
}

uint8_t rx_fifo_is_empty() {
	
	uint8_t fifo_status = 0;

	mirf_read_register(FIFO_STATUS, &fifo_status, 1);
	
	return (uint8_t)(fifo_status&0x01);
}

void flush_rx_fifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_RX);
    mirf_CSN_hi;

}

void mirf_get_data(uint8_t * data) 
// Reads mirf_PAYLOAD bytes into data array
{
    mirf_CSN_lo;                               		// Pull down chip select
    SPI_Write_Byte( R_RX_PAYLOAD );            		// Send cmd to read rx payload
    SPI_ReadWrite_Block(data,data,PAYLOAD_SIZE); 	// Read payload
    mirf_CSN_hi;                               		// Pull up chip select
    mirf_config_register(STATUS,(1<<RX_DR));   		// Reset status register
}

void mirf_config_register(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Byte(value);
    mirf_CSN_hi;
}

void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(R_REGISTER | (REGISTER_MASK & reg));
    SPI_ReadWrite_Block(value,value,len);
    mirf_CSN_hi;
}

void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Block(value,len);
    mirf_CSN_hi;
}


void mirf_send(uint8_t * value, uint8_t len) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    while (PTX) {}                  // Wait until last paket is send

    mirf_CE_lo;

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up
    
    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( W_TX_PAYLOAD ); // Write cmd to write payload
    SPI_Write_Block(value,len);   // Write payload
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CE_hi;                     // Start transmission
}

void writeAckPayload(unsigned char *data, unsigned char size) {

	unsigned char k = 0;

	flushTxFifo();

    mirf_CSN_lo;

	SPI_Write_Byte(NRF_W_ACK_PAYLOAD_P0);

	for(k=0; k<size; k++) {
		SPI_Write_Byte(data[k]);
	}	

    mirf_CSN_hi;


}


void flushTxFifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_TX);
    mirf_CSN_hi;

}

