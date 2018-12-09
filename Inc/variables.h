/*
 * variables.h
 *
 *  Created on: 3.5.2018
 *      Author: krouvsa
 */

#ifndef VARIABLES_H_
#define VARIABLES_H_

#include "main.h"
#include "stm32f0xx_hal.h"


#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

struct usb
{
	uint32_t USBdataLen;
	uint8_t buffer[100];
	uint8_t connected;
	uint8_t dataInBuffer;
	uint8_t dataRdyForModule;
} usb;


struct tick
{
	uint32_t Start;
	uint32_t Stop;
	uint32_t Value;
	uint8_t flag;
} tick;

struct uart
{
	uint8_t rxbuffer[100];
	uint8_t txbuffer[100];
	uint8_t buffercopy[100];
	uint8_t rxbyte;
	uint8_t dataready;
};


#endif /* VARIABLES_H_ */
