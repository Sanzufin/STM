/*
 * MP3.c
 *
 *  Created on: 9.11.2018
 *      Author: Santtu
 */

#include "variables.h"
#include "MP3.h"

extern UART_HandleTypeDef huart1;

void uart_Send(uint8_t data[])
{
	HAL_UART_Transmit(&huart1,&data[0],8,100);	// Change this to match your uart transmit function
}

void MP3_Init()
{
	const uint8_t init[] = {0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xEF};
	CDC_Transmit_FS(&init,8);
	uart_Send(init);
}
void MP3_play()
{
	const uint8_t play[] = {0x7E, 0xFF, 0x06, 0x0D, 0x00, 0x00, 0x00, 0xEF};
	CDC_Transmit_FS(&play,8);
	uart_Send(play);
}

void MP3_pause()
{
	const uint8_t pause[] = {0x7E, 0xFF, 0x06, 0x0E, 0x00, 0x00, 0x00, 0xEF};
	uart_Send(pause);
}

void MP3_volUp()
{
	const uint8_t volUp[] = {0x7E, 0xFF, 0x06, 0x04, 0x00, 0x00, 0x00, 0xEF};
	uart_Send(volUp);
}

void MP3_volDown()
{
	const uint8_t volDown[] = {0x7E, 0xFF, 0x06, 0x05, 0x00, 0x00, 0x00, 0xEF};
	uart_Send(volDown);
}

void MP3_volSet(uint8_t volume)
{
	const uint8_t volSet[] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, volume, 0xEF};
	uart_Send(volSet);
}

void MP3_next()
{
	const uint8_t next[] = {0x7E, 0xFF, 0x06, 0x01, 0x00, 0x00, 0x00, 0xEF};
	uart_Send(next);
}

void MP3_prev()
{
	const uint8_t prev[] = {0x7E, 0xFF, 0x06, 0x02, 0x00, 0x00, 0x00, 0xEF};
	uart_Send(prev);
}

void MP3_playWithIndex()
{
	const uint8_t playIndex[] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xEF};
	CDC_Transmit_FS(&playIndex,8);
	uart_Send(playIndex);
}
