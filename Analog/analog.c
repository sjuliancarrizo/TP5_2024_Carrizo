/*
 * analog.c
 *
 *  Created on: Jun 11, 2024
 *      Author: julian
 */
#include "analog.h"

void initDAC();
void initADC();
void writeDAC(uint16_t valueInMv);

void initDAC()
{
	static DAC_InitTypeDef DAC_InitStruct;
	static GPIO_InitTypeDef GPIO_InitStruct;

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

	writeDAC(0);
}

void initADC()
{
	static GPIO_InitTypeDef GPIO_InitStruct;
	static ADC_InitTypeDef ADC_InitStructure;

	//Init ADC 1 / GPIOA 6 Channel 6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

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


}

void writeDAC(uint16_t valueInMv)
{
	//This is the minimum change in milivolts handled by the DAC.
	float DACResolutionInMv = (float)ANALOG_VREF_MV / (float)DAC_RES_BITS;

	uint16_t DACValue = valueInMv / DACResolutionInMv;
	DAC_SetChannel1Data(DAC_Align_12b_R, DACValue);
}

void readADCCh7(uint16_t *value)
{
	ADC_SoftwareStartConv(ADC1);

	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	*value = ADC_GetConversionValue(ADC1);
}

