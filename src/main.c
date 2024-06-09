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

//DAC voltage reference in milivolts
#define DAC_VREF_MV 3300

//DAC resolutions. Value of 2^n
#define DAC_RES_BITS 4096

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

uint32_t systickGlobal = 0;
uint8_t pressedKey = NO_KEY, tick = 0, firstKey = 0, backlightState;
uint16_t counter = 0;

taskData LCDTaskData, keypadTaskData, counterTaskData;

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
void writeDAC(uint16_t valueInMv);
void initGPIO();
void backlightOn();
void backlightOff();
void delay_us (uint16_t timeInUs);
uint8_t taskIsReadtToRun(taskData *task);
void LCDTask();
void keypadTask();
void counterTask();
uint32_t getGlobalSystickValue();
void taskHandler();
void writeDAC(uint16_t valueInMv);

/*******************************
 * End function prototypes
 *******************************/

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);

	initGPIO();

	LCDTaskData.intervalInMs = 250;
	keypadTaskData.intervalInMs = 10;
	counterTaskData.intervalInMs = 1000;

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
	DAC_InitTypeDef DAC_InitStruct;
	//ADC_InitTypeDef ADC_InitStructure;

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

	//Init DAC GPIO A4
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);

	DAC_Cmd(DAC_Channel_1, ENABLE);


	/*
	//Init ADC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_3Cycles);

	//Enable ADC
	ADC_Cmd(ADC1, ENABLE);
	*/

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
						//I should ask for the current menu level before setting up the DAC value.
						uint16_t selectedValueInMv;
						selectedValueInMv = (pressedKey - 48) * 100;
						menuSetDACValue(selectedValueInMv);
						writeDAC(selectedValueInMv);
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
 * Analog functions
 ****************************/

void writeDAC(uint16_t valueInMv)
{
	//This is the minimum change in milivolts handled by the DAC.
	float DACResolutionInMv = (float)DAC_VREF_MV / (float)DAC_RES_BITS;

	uint16_t DACValue = valueInMv / DACResolutionInMv;
	DAC_SetChannel1Data(DAC_Align_12b_R, DACValue);
}
			
/****************************
 * End analog functions
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
