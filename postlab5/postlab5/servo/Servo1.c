/*
 * Servo1.c
 *
 * Created: 4/11/2025 6:06:30 PM
 * Author: Adrián Fernández
 */ 

#include "Servo1.h"

void map_servo(uint16_t ADC_var, uint16_t *PWM_var)
{
	*PWM_var = (ADC_var * 40000) / 1023 + 1000;
	
}