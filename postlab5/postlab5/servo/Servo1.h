

/*
 * Servo1.h
 *
 * Created: 4/11/2025 6:06:57 PM
 *  Author: adria
 */ 


#ifndef SERVO1_H_
#define SERVO1_H_

#include <avr/io.h>

void map_servo(uint16_t ADC_var, uint16_t *PWM_var);

#endif /* SERVO1_H_ */