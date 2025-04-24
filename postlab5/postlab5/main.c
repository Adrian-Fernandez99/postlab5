/*
Universidad del Valle de Guatemala
IE2023 : Programación de Microcontroladores

postlab5.c

Created: 4/23/2025 10:33:36 PM
Author : Adrián Fernández

Descripción: Controlador de dos servos haciendo 
uso de dos potenciometros ditstintos.
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Prototipos de función
void PWM_init();
void ADC_init();
void TMR0_init();
uint16_t ADC_read(uint8_t PIN);

uint16_t ADC_servo1 = 0;
uint16_t ADC_servo2 = 0;
uint16_t ADC_led = 0;

uint16_t PWM_1 = 0;
uint16_t PWM_2 = 0;
uint8_t LED_TMR = 0;

uint8_t momento = 0;
uint8_t TMR_val = 254;

// MAIN LOOP
int main(void)
{
	TMR0_init();
	PWM_init();
	ADC_init();

	while (1)
	{
		sei();
		ADC_servo1 = ADC_read(6);
		ADC_servo2 = ADC_read(7);

		map_servo(uint16_t ADC_servo1, uint16_t &PWM_1);
		map_servo(uint16_t ADC_servo2, uint16_t &PWM_2);
		
		OCR1A = PWM_1;
		OCR1B = PWM_2;
		
		ADC_led = ADC_read(5);
		map_led(uint16_t ADC_led, uint8_t &TMR_val);

		_delay_ms(20);
	}
}

// NON-Interrupt subroutines
// Funciones de Seteo
void PWM_init()
{
	DDRB |= (1 << PINB1) | (1 << PINB2);  // D9 y D10 como salida

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);    // Modo Fast PWM 14, TOP = ICR1 y prescaler = 8

	ICR1 = 20000;  // Setear Top como 20ms
}

void ADC_init()
{
	ADMUX = (1 << REFS0);  // 5V de referencia
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // prescaler = 64
}

void TMR0_init()
{
	DDRD |= 0xFF;		// Puerto D es salida
	PORTD |= 0x00;
	
	CLKPR	= (1 << CLKPCE); // Habilitar cambio en el prescaler
	CLKPR	= (1 << CLKPS2); // Setea presc a 16 para 1Mhz
	
	TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 1024
	TIMSK0 = (1 << TOIE0);
	TCNT0 = TMR_val;
}

uint16_t ADC_read(uint8_t PIN)
{
	ADMUX = (ADMUX & 0xF0) | (PIN & 0x0F);   // Selecciona canal (0–7)
	ADCSRA |= (1 << ADSC);                   // Inicia conversión
	while (ADCSRA & (1 << ADSC));            // Espera a que termine
	return ADC;                              // Devuelve valor (10 bits)
}

// Interrupt routines

ISR(TIMER0_OVF_vect)
{
	cli();
	
	momento++;
	if (momento == 255)
	{
		momento = 0;  // Encender LED
	}
	
	if (momento < LED_TMR)
	{
		PORTD |= (1 << PORTD2);  // Encender LED
	}
	else
	{
		PORTD &= ~(1 << PORTD2); // Apagar LED
	}
	
	TCNT0 = TMR_val;  // Precarga el timer
	
	sei();
}