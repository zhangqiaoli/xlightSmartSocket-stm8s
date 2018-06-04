#ifndef _I_collect_H_
#define _I_collect_H_

#include "stm8s.h"
#include "stm8s_gpio.h"

//extern uint16_t ADC_buf[120];

void init_ADC();
void read_ADC_value();
uint16_t CalcEffectiveValue();

uint16_t GetCurrent();

uint16_t GetMinuteEQ(uint16_t* index);

#endif