#ifndef _KEYPAD_H
#define _KEYPAD_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "global.h"

#define KEYPAD_INIT_OK 0
#define KEYPAD_INIT_ERROR 1

#define NO_KEY 0
#define NO_KEY_ROW 255
#define NO_KEY_COL NO_KEY_ROW

/*
 * Used ports must be manually initialized before.
 */
typedef struct
{
	GPIO_TypeDef *port;
	uint32_t pin;
} GPIO_PortPin;

typedef struct {
	//Number of rows
	uint8_t nRows;

	//Number of columns
	uint8_t nCols;

	/*
	 * Pins attached to rows. Starting from row 0.
	 * They'll be treated as push-pull outputs.
	 */
	GPIO_PortPin *rowPins;

	/*
	 * Pins attached to columns. Starting from column 0.
	 * They'll be treated as pull-down inputs.
	 */
	GPIO_PortPin *colPins;

	/*
	 * Map characters to cells. This array must be sized
	 * according to nRows*nCols.
	 */
	uint8_t *keyMapping;

} keypad_InitTypeDef;

//Configure GPIO used for the keypad
uint8_t keypadInit(keypad_InitTypeDef keypad_InitStruct);

//Return key being pressed.
void processKeypad(uint8_t* pressedKey);



#endif
