/*
 * PWM_manual.h
 *
 * Created: 4/24/2025 10:42:17 AM
 *  Author: adria
 */ 


#ifndef PWM_MANUAL_H_
#define PWM_MANUAL_H_

#include <avr/io.h>

void map_led(uint8_t ADC_var, uint8_t *TMR_val);

#endif /* PWM_MANUAL_H_ */