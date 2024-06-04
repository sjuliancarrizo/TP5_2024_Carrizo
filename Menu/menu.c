/*
 * menu.c
 *
 *  Created on: May 25, 2024
 *      Author: julian
 */

#include "menu.h"
#include "string.h"
#include "stdio.h"

#define MENU_ITEMS 5
#define MENU_LEVELS 2

char menuL0Item[MENU_ITEMS][MENU_LINE_LENGTH] = {
		" Tecla Presionad\0",
		" Backlight      \0",
		" Contador       \0",
		" Datos Alumno   \0",
		" Datos Materia  \0"
};

uint8_t _cursorCurrentItem = 0;
uint8_t _menuCurrentLevel = 0;
uint8_t _menuHasChanged = 1;
uint8_t _key;
uint16_t _counter;
uint8_t _backlightState;
char menuLine[MENU_LINES][MENU_LINE_LENGTH];

void menuCursorDown()
{
	if (_menuCurrentLevel != 0)
		return;

	if (_cursorCurrentItem < MENU_ITEMS - 1)
		_cursorCurrentItem++;
	else
		_cursorCurrentItem = 0;

	_menuHasChanged = 1;
}

void menuCursorUp()
{
	if (_menuCurrentLevel != 0)
			return;

	if (_cursorCurrentItem > 0)
		_cursorCurrentItem--;
	else
		_cursorCurrentItem = MENU_ITEMS - 1;

	_menuHasChanged = 1;
}

void menuEnter()
{
	if (_menuCurrentLevel < MENU_LEVELS - 1)
		_menuCurrentLevel++;

	_menuHasChanged = 1;
}
void menuBack()
{
	if (_menuCurrentLevel > 0)
		_menuCurrentLevel--;

	_menuHasChanged = 1;
}

void buildMenu()
{
	uint8_t currentPage, itemStart;

	currentPage = _cursorCurrentItem / MENU_LINES;
	itemStart = currentPage * MENU_LINES;
	if (_menuCurrentLevel == 0)
	{
		for (uint8_t i = 0; i < MENU_LINES; i++)
		{
			if((_cursorCurrentItem + i) >= MENU_ITEMS)
			{
				strcpy(menuLine[i],"                \0");
			}
			else
			{
				strcpy(menuLine[i], menuL0Item[itemStart + i]);


				if (i == (_cursorCurrentItem - (currentPage * MENU_LINES)))
					menuLine[i][0] = '>';
			}

		}
	}
	else if (_menuCurrentLevel == 1)
	{
		char stringToPrint[MENU_LINE_LENGTH];
		strcpy(menuLine[MENU_LINES - 1],"                \0");
		switch(_cursorCurrentItem)
		{
		case 0:
			sprintf(stringToPrint, "Tecla: %c", _key);
			strcpy(menuLine[0],stringToPrint);

		break;

		case 1:
			if (_backlightState)
				sprintf(stringToPrint, "Backlight: ON");
			else
				sprintf(stringToPrint, "Backlight: OFF");
			strcpy(menuLine[0],stringToPrint);
		break;

		case 2:
			sprintf(stringToPrint, "Tiempo: %d seg", _counter);
			strcpy(menuLine[0],stringToPrint);
		break;

		case 3:
			strcpy(menuLine[0],"Julian");
			strcpy(menuLine[1],"Carrizo");
		break;
		case 4:
			strcpy(menuLine[0],"Ing. A. Laiuppa");
			strcpy(menuLine[1],"Tec. Digitales 2");
		break;
		}
	}
	_menuHasChanged = 0;
}

void menuSetPressedKey(uint8_t key)
{
	if (key != _key)
	{
		_key = key;
		_menuHasChanged = 1;
	}
}
void menuSetCounter(uint16_t counter)
{
	if (counter != _counter)
	{
		_counter = counter;
		_menuHasChanged = 1;
	}
}
void menuSetBacklight(uint8_t backlightState)
{
	if (backlightState != _backlightState)
	{
		_backlightState = backlightState;
		_menuHasChanged = 1;
	}
}

uint8_t getStringLine(uint8_t lineIndex, uint8_t lineLength, char stringLine[MENU_LINE_LENGTH])
{
	if (lineLength != MENU_LINE_LENGTH || lineIndex >= MENU_LINES)
		return MENU_ERROR;
	strcpy(stringLine, menuLine[lineIndex]);
	return MENU_OK;
}

uint8_t getMenuHasChanged()
{
	return _menuHasChanged;
}
