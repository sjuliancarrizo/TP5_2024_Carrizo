/*  UTN - FRBB
*   Técnicas Digitales II - Ingeniería Electrónica
*   Julián Carrizo
*   E-mail: sjuliancarrizo@gmail.com
*
*	Placa de desarrollo: NUCLEO-F446RE
*	TP5: ADC y DAC
*/

#include "LCD.h"
#include "keypad.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "global.h"
#include "menu.h"
#include "ctype.h"
#include "analog.h"
#include "ntc.h"

#define COUNTER_1_US 13

#define BACKLIGHT_PIN GPIO_Pin_13
#define BACKLIGHT_PORT GPIOB

/*
 * 4x4 keyboard pinout (front view)
 *
 * 		| R1 | R2 | R3 | R4 | C1 | C2 | C3 | C4 |
 */

#define N_ROWS 4
#define N_COLS 4

//Number of samples the ADC must take before calculating the analog value.
#define ADC_SAMPLES 16

//Time in ms between ADC samples.
#define ADC_SAMPLES_INTERVAL 5

/****************************
 * Global vars and structs
 ****************************/

typedef struct {
	uint16_t intervalInMs;
	uint32_t startTimeInMs;
} taskData;

typedef enum{
	ST_KEY_NOT_PRESSED = 0,
	ST_KEY_PRESSING,
	ST_KEY_WAITING_RELEASE
} keypadStates;

typedef enum{
	ST_IDLE = 0,
	ST_SAMPLING,
	ST_GET_VALUE
} analogStates;

uint32_t systickGlobal = 0;
uint8_t pressedKey = NO_KEY, tick = 0, firstKey = 0, backlightState;
uint16_t counter = 0, DACSelectedValueInMv = 0, DACCurrentValueInMv = 0, ADCValue;
float ADCValueInMv;
float NTCValueInC, NTCValueInF;

taskData LCDTaskData, keypadTaskData, counterTaskData, analogTaskData;

keypad_InitTypeDef keypadInitStruct;

GPIO_PortPin rowsPortPin[N_ROWS] = {
		{.port = GPIOC, .pin = GPIO_Pin_0},
		{.port = GPIOC, .pin = GPIO_Pin_3},
		{.port = GPIOC, .pin = GPIO_Pin_2},
		{.port = GPIOC, .pin = GPIO_Pin_1}
};
GPIO_PortPin colsPortPin[N_COLS] = {
		{.port = GPIOB, .pin = GPIO_Pin_0},
		{.port = GPIOA, .pin = GPIO_Pin_8},
		{.port = GPIOB, .pin = GPIO_Pin_7},
		{.port = GPIOA, .pin = GPIO_Pin_15}
};
uint8_t keypadKeyMapping[] = {
		'1', '2', '3', 'A',
		'4', '5', '6', 'B',
		'7', '8', '9', 'C',
		'*', '0', '#', 'D'
};

char stringToPrint[16] = "                ";

/*******************************
 * End global vars and structs
 *******************************/

/*******************************
 * Function prototypes
 *******************************/
void initGPIO();
void backlightOn();
void backlightOff();
void delay_us (uint16_t timeInUs);
uint8_t taskIsReadtToRun(taskData *task);
void LCDTask();
void keypadTask();
void counterTask();
void analogTask();
uint32_t getGlobalSystickValue();
void taskHandler();

/*******************************
 * End function prototypes
 *******************************/

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);

	initGPIO();
	initDAC();
	initADC();

	LCDTaskData.intervalInMs = 250;
	keypadTaskData.intervalInMs = 10;
	counterTaskData.intervalInMs = 1000;

	/*
	 * This also affects the interval the DAC is updated.
	 * It may not be desired though.
	 */
	analogTaskData.intervalInMs = ADC_SAMPLES * ADC_SAMPLES_INTERVAL;

	while(1)
	{
		taskHandler();
	}
}

void initGPIO()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;

	//Inicializo BACKLIGHT
	GPIO_InitStruct.GPIO_Pin = BACKLIGHT_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BACKLIGHT_PORT, &GPIO_InitStruct);

	//Init Keypad
	keypadInitStruct.nRows = N_ROWS;
	keypadInitStruct.nCols = N_COLS;

	keypadInitStruct.rowPins = rowsPortPin;
	keypadInitStruct.colPins = colsPortPin;
	keypadInitStruct.keyMapping = keypadKeyMapping;

	keypadInit(keypadInitStruct);

	LCD_init();
	backlightOn();
}


/*******************************
 * Handler and Tasks functions
 *******************************/
void taskHandler()
{
	if(tick)
	{
		tick = 0;
		if (taskIsReadtToRun(&LCDTaskData))
		{
			LCDTaskData.startTimeInMs = systickGlobal;
			LCDTask();
		}
		if (taskIsReadtToRun(&keypadTaskData))
		{
			keypadTaskData.startTimeInMs = systickGlobal;
			keypadTask();
		}
		if (taskIsReadtToRun(&counterTaskData))
		{
			counterTaskData.startTimeInMs = systickGlobal;
			counterTask();
		}
		if (taskIsReadtToRun(&analogTaskData))
		{
			analogTaskData.startTimeInMs = systickGlobal;
			analogTask();
		}
	}
}

uint8_t taskIsReadtToRun(taskData *task)
{
	uint8_t ret;
	if ((systickGlobal - task->startTimeInMs) > task->intervalInMs)
		ret = 1;
	else
		ret = 0;

	return ret;
}

void LCDTask()
{
	menuSetPressedKey(pressedKey);
	menuSetCounter(counter);
	menuSetBacklight(backlightState);

	if (!getMenuHasChanged())
		return;

	buildMenu();

	for (uint8_t i = 0; i < MENU_LINES; i++)
	{
		if (getStringLine(i, MENU_LINE_LENGTH , stringToPrint) != MENU_ERROR)
		{
			LCD_WriteString(0, i, "                ");
			LCD_WriteString(0, i, stringToPrint);
		}
	}
}

void keypadTask()
{
	uint8_t currentKey = NO_KEY;
	static keypadStates keypadTaskState = ST_KEY_NOT_PRESSED;

	switch (keypadTaskState)
	{
		case ST_KEY_NOT_PRESSED:
			processKeypad(&currentKey);
			if (currentKey != NO_KEY)
			{
				keypadTaskState = ST_KEY_PRESSING;
				pressedKey = currentKey;
			}

		break;

		case ST_KEY_PRESSING:
			switch (pressedKey)
			{
				case 'A':
					menuCursorUp();
				break;

				case 'B':
					menuCursorDown();
				break;

				case 'C':
					menuEnter();
				break;

				case 'D':
					menuBack();
				break;

				case '#':
					backlightOn();
				break;

				case '*':
					backlightOff();
				break;

				default:
					if (isdigit(pressedKey))
					{
						//This must be improved in order to not use literals
						if (getMenuCurrentLevel() == 1 && getMenuCurrentItem() == 3)
						{
							DACSelectedValueInMv = (pressedKey - 48) * 100;
							if (DACSelectedValueInMv != DACCurrentValueInMv)
								menuSetDACValue(DACSelectedValueInMv);
						}

					}
				break;
			}

			keypadTaskState = ST_KEY_WAITING_RELEASE;
		break;

		case ST_KEY_WAITING_RELEASE:
			processKeypad(&currentKey);
			if (currentKey == NO_KEY)
				keypadTaskState = ST_KEY_NOT_PRESSED;
		break;

	}
}

void analogTask()
{
	static analogStates analogTaskState = ST_IDLE;
	static uint8_t currentSampleIndex = 0;
	static uint16_t accumulativeADCValue = 0;
	//static float ADCResolutionInMv = (float)ANALOG_VREF_MV / (float)ADC_RES_BITS;
	uint16_t temp;

	if (DACSelectedValueInMv != DACCurrentValueInMv)
	{
		DACCurrentValueInMv = DACSelectedValueInMv;
		writeDAC(DACCurrentValueInMv);
	}

	switch (analogTaskState)
	{
		case ST_IDLE:
			currentSampleIndex = 0;
			accumulativeADCValue = 0;
			temp = 0;
			analogTaskState = ST_SAMPLING;

		break;

		case ST_SAMPLING:
			if (currentSampleIndex < ADC_SAMPLES)
			{
				readADCCh7(&temp);
				accumulativeADCValue += temp;
				currentSampleIndex++;
			}
			else
				analogTaskState = ST_GET_VALUE;
		break;

		case ST_GET_VALUE:
			accumulativeADCValue /= ADC_SAMPLES;
			calculateNTCTemp(accumulativeADCValue);
			NTCValueInC = getTempInC();
			NTCValueInF = getTempInF();
			menuSetTempValue(NTCValueInC, NTCValueInF);
			analogTaskState = ST_IDLE;
		break;
	}
}


void counterTask()
{
	counter++;
}

/***********************************
 * End Handler and Tasks functions
 ***********************************/

/****************************
 * Timing functions
 ****************************/
void delay_us (uint16_t timeInUs)
{
	for(uint16_t j = 0; j < timeInUs; j++)
	{
		for (uint16_t i = 0; i < COUNTER_1_US; i++)
		{
			continue;
		}
	}
}

void delay_ms(uint16_t timeInMs)
{
	timeInMs += systickGlobal;
	while (timeInMs > systickGlobal);
}


void SysTick_Handler(void)
{
	systickGlobal++;
	tick++;
}

uint32_t getGlobalSystickValue()
{
	return systickGlobal;
}
/****************************
 * End timing functions
 ****************************/

/****************************
 * Other functions
 ****************************/
void backlightOn()
{
	GPIO_SetBits(BACKLIGHT_PORT, BACKLIGHT_PIN);
	backlightState = 1;
}
void backlightOff()
{
	GPIO_ResetBits(BACKLIGHT_PORT, BACKLIGHT_PIN);
	backlightState = 0;
}

/****************************
 * End other functions
 ****************************/
