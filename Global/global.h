#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "stm32f4xx.h"

//extern uint32_t systickGlobal;
uint32_t getGlobalSystickValue();

void delay_ms(uint16_t timeInMs);

void delay_us(uint16_t timeInUs);

#endif
