/*
 * MP3.h
 *
 *  Created on: 9.11.2018
 *      Author: Santtu
 */

#ifndef MP3_H_
#define MP3_H_

void uart_Send(uint8_t data[100]);
void MP3_play();
void MP3_pause();
void MP3_volUp();
void MP3_volDown();
void MP3_volSet(uint8_t volume);
void MP3_next();
void MP3_prev();
void MP3_playWithIndex();
void MP3_Init();
#endif /* MP3_H_ */
