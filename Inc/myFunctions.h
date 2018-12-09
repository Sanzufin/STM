/*
 * myFunctions.h
 *
 *  Created on: 2.11.2018
 *      Author: Santtu
 */

#ifndef MYFUNCTIONS_H_
#define MYFUNCTIONS_H_

void Output_Write(uint8_t channel, uint8_t state);
void Output_Toggle(uint8_t channel);
void Pulse(uint8_t channel, uint16_t pulse_time);
void timer(uint16_t time);
void doStuff(uint8_t input_code);
int Input();
int PressedButton();


uint8_t Input_Number = 0;		// Save pressed inputs here 0-127

#endif /* MYFUNCTIONS_H_ */
