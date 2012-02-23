
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


/**
 * \brief Configure the usart registers to work at 57600 baud (8-bit data, no parity, 1 stop bit).
 * Moreover the interrupt for reception is enabled.
 * \return none
 */
void initUsart();

/**
 * \brief Transfer one byte of data; it's blocking (wait until the buffer is empty).
 * \param data data to be sent through usart
 * \return none
 */
void usartTransmit(unsigned char data);

/**
 * \brief Close the usart peripheral and disable all interrupts.
 * \return none
 */
void closeUsart();

#endif
