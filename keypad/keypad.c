#include "keypad.h"

#define KEY_PRESSED 1
#define KEY_NOT_PRESSED 0

#define DEBOUNCE_TIME_IN_MS 20

typedef enum {
	ST_READING = 0,
	ST_DEBOUNCE,
	ST_PRESSING_CHECK,
	ST_PRESSING_VALID,
	ST_PRESSING_NOT_VALID
}states;

typedef struct {
	uint8_t value;
	uint8_t row;
	uint8_t col;

}keyStruct;

uint8_t _nRows;
uint8_t _nCols;
GPIO_PortPin *_rowPins;
GPIO_PortPin *_colPins;
uint8_t *_keyMapping;

uint32_t debounceStartTimeInMs = 0;

states _state = ST_READING;
keyStruct _currentKey = {.value = NO_KEY, .row = NO_KEY_ROW, .col = NO_KEY_COL};
keyStruct _noKey = {.value = NO_KEY, .row = NO_KEY_ROW, .col = NO_KEY_COL};

uint8_t  keypadInit(keypad_InitTypeDef keypad_InitStruct)
{
	_nRows = keypad_InitStruct.nRows;
	_nCols = keypad_InitStruct.nCols;
	_rowPins = keypad_InitStruct.rowPins;
	_colPins = keypad_InitStruct.colPins;
	_keyMapping = keypad_InitStruct.keyMapping;

	//Check pointers size
	if (sizeof(_rowPins) != _nRows || sizeof(_colPins) != _nCols)
	//		|| sizeof(_keyMapping) != (_nCols*_nRows))
	{
		return KEYPAD_INIT_ERROR;
	}

	//Set row pins as outputs
	GPIO_InitTypeDef GPIO_InitStruct;
	for (uint8_t i = 0; i < _nRows; i++)
	{
		GPIO_InitStruct.GPIO_Pin = _rowPins[i].pin;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(_rowPins[i].port, &GPIO_InitStruct);
	}

	//Set column pins as pull-down inputs.
	for (uint8_t i = 0; i < _nCols; i++)
	{
		GPIO_InitStruct.GPIO_Pin = _colPins[i].pin;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(_colPins[i].port, &GPIO_InitStruct);
	}

	return KEYPAD_INIT_OK;
}

void clearRows()
{
	for (uint8_t i = 0; i < _nCols; i++)
	{
		GPIO_ResetBits(_rowPins[i].port, _rowPins[i].pin);
	}
}

//Ask for a specific key, and return its state (pressed or not)
uint8_t getKeyStatus(uint8_t row, uint8_t col)
{
	if (row == NO_KEY_ROW || col == NO_KEY_COL)
		return KEY_NOT_PRESSED;

	uint8_t retValue;

	//Enable selected row pin
	GPIO_SetBits(_rowPins[row].port, _rowPins[row].pin);

	//Read selected col pin
	if (GPIO_ReadInputDataBit(_colPins[col].port, _colPins[col].pin) == KEY_PRESSED)
		retValue = KEY_PRESSED;
	else
		retValue = KEY_NOT_PRESSED;

	//Disable selected row pin
	GPIO_ResetBits(_rowPins[row].port, _rowPins[row].pin);

	return retValue;
}

keyStruct readKeypadAndGetKey()
{
	keyStruct pressedKey = {.value = NO_KEY, .row = NO_KEY_ROW, .col = NO_KEY_COL};
	for (uint8_t r = 0; r < _nRows; r++)
	{
		GPIO_SetBits(_rowPins[r].port, _rowPins[r].pin);
		for (uint8_t c = 0; c < _nCols; c++)
		{
			if (GPIO_ReadInputDataBit(_colPins[c].port, _colPins[c].pin) == KEY_PRESSED)
			{
				pressedKey.value = _keyMapping[_nCols*r+c];
				pressedKey.row = r;
				pressedKey.col = c;
				return pressedKey;
			}
		}
		GPIO_ResetBits(_rowPins[r].port, _rowPins[r].pin);
	}

	return pressedKey;
}

void processKeypad(uint8_t* _pressedKey)
{
	switch (_state)
	{
	case ST_READING:
		//Make sure all the outputs are low.
		clearRows();
		*_pressedKey = NO_KEY;
		_currentKey = readKeypadAndGetKey();
		if ( _currentKey.value == NO_KEY)
			break;
		else
		{
			_state = ST_DEBOUNCE;
			debounceStartTimeInMs = getGlobalSystickValue();
		}
		break;

	case ST_DEBOUNCE:
		if ((getGlobalSystickValue() - debounceStartTimeInMs) >= DEBOUNCE_TIME_IN_MS)
			_state = ST_PRESSING_CHECK;

		break;

	case ST_PRESSING_CHECK:
		if (getKeyStatus(_currentKey.row,_currentKey.col) == KEY_NOT_PRESSED)
			_state = ST_PRESSING_NOT_VALID;
		else
			_state = ST_PRESSING_VALID;
		break;

	case ST_PRESSING_VALID:
		if (getKeyStatus(_currentKey.row,_currentKey.col) == KEY_NOT_PRESSED)
		{
			_state = ST_DEBOUNCE;
			debounceStartTimeInMs = getGlobalSystickValue();
		}
		else
			*_pressedKey = _currentKey.value;
		break;

	case ST_PRESSING_NOT_VALID:
		_currentKey = _noKey;
		_state = ST_READING;
		break;
	}

}
