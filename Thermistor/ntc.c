/*
 * ntc.c

 *
 *  Created on: Jun 18, 2024
 *      Author: julian
 */

#include "ntc.h"

float tempInK, tempInC, tempInF;

void calculateNTCTemp(uint16_t ADCValue)
{
	float tAmbienInverse = 1.0 / (float) T_25C_IN_K;
	double logArgument = (double)ADCValue / (double)(ADC_MAX_VALUE - ADCValue);

	double logResult = log(logArgument);

	float denominator = (logResult / NTC_BETA) + tAmbienInverse;

	tempInK = (1 / denominator);
	tempInC = tempInK - C_TO_K_CONSTANT;
	tempInF = (tempInC * 9.0 / 5.0) + 32.0;
}

float getTempInC()
{
	return tempInC;
}
float getTempInK()
{
	return tempInK;
}
float getTempInF()
{
	return tempInF;
}

