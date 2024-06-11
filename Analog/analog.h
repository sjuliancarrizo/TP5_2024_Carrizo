/*
 * analog.h
 *
 *  Created on: Jun 11, 2024
 *      Author: julian
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"

//Analog voltage reference in milivolts
#define ANALOG_VREF_MV 3300

//DAC resolutions. Value of 2^n
#define DAC_RES_BITS 4096

//ADC resolutions. Value of 2^n
#define ADC_RES_BITS 4096

void initDAC();
void initADC();

void writeDAC(uint16_t valueInMv);
void readADCCh7(uint16_t *value);

#endif /* ANALOG_H_ */
