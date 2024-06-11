/*
 * menu.h
 *
 *  Created on: May 25, 2024
 *      Author: julian
 */

#ifndef MENU_H_
#define MENU_H_

#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"

#define MENU_ERROR 0
#define MENU_OK 1

#define MENU_LINE_LENGTH 17
#define MENU_LINES 2
/*
 * These functions names are relative to the cursor's position
 * in the LCD, not the cursor position value. Moving the cursor down means
 * incrementing its position value..
 */
void menuCursorUp();
void menuCursorDown();
void menuEnter();
void menuBack();

void buildMenu();
uint8_t getStringLine(uint8_t lineIndex, uint8_t lineLength, char stringLine[MENU_LINE_LENGTH]);

uint8_t getMenuHasChanged();
uint8_t getMenuCurrentLevel();
uint8_t getMenuCurrentItem();

void menuSetPressedKey(uint8_t key);
void menuSetCounter(uint16_t counter);
void menuSetBacklight(uint8_t backlightState);
void menuSetDACValue(uint16_t DACValueInMv);
void menuSetTempValue(float tempValue);



#endif /* MENU_H_ */
