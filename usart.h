
#ifndef USART_H
#define USART_H


/**
 * \file usart.h
 * \brief Usart module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The usart peripheral is used primarly for debugging purposes; it's initialized to work at 57600 baud that 
 is the maximum throughput usable with the main clock at 8 MHz. An interrupt is generated at each character 
 reception; a function for transfer data is also available.
*/


#include "variables.h"
#include <avr\io.h>
#include <avr\interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure the usart0 registers to work at 57600 baud (8-bit data, no parity, 1 stop bit).
 * Moreover the interrupt for reception is enabled.
 * \return none
 */
void initUsart0();

/**
 * \brief Configure the usart1 registers to work at 57600 baud (8-bit data, no parity, 1 stop bit).
 * Moreover the interrupt for reception is enabled.
 * \return none
 */
void initUsart1();

/**
 * \brief Transfer one byte of data; it's blocking (wait until the buffer is empty).
 * \param data data to be sent through usart0
 * \return none
 */
void usart0Transmit(unsigned char data, unsigned char isBlocking);

/**
 * \brief Transfer one byte of data; it's blocking (wait until the buffer is empty).
 * \param data data to be sent through usart1
 * \return none
 */
void usart1Transmit(unsigned char data, unsigned char isBlocking);

/**
 * \brief Close the usart peripheral and disable all interrupts.
 * \return none
 */
void closeUsart();

/**
 * \brief Check whether there is something to read from usart0.
 * \retval 0 input buffer not empty
 * \retval 1 input buffer empty
 */
char usart0InputBufferEmpty();

/**
 * \brief Wait until a byte has been received or the timeout is expired.
 * \return byte received
 */
unsigned char usart0Receive();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
