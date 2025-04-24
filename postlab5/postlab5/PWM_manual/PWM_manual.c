/*
 * PWM_manual.c
 *
 * Created: 4/24/2025 10:42:03 AM
 *  Author: adria
 */ 

#include "PWM_manual.h"

void map_led(uint16_t ADC_var, uint8_t *TMR_val)
{
	TMR_val = (ADC_var * 254UL) / 1023;
}