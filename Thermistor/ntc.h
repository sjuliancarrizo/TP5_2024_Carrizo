/*
 * ntc.h
 *
 *  Created on: Jun 18, 2024
 *      Author: julian
 */

#ifndef NTC_H_
#define NTC_H_

#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "math.h"

#define R_DIVIDER 10000
#define ADC_MAX_VALUE 4095
#define R_NTC_25C 10000
#define NTC_BETA 4050
#define T_25C_IN_K 298.15
#define C_TO_K_CONSTANT 273.15
void calculateNTCTemp(uint16_t ADCValue);

float getTempInC();
float getTempInK();
float getTempInF();


#endif /* NTC_H_ */
