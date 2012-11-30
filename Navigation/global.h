#ifndef GLOBAL_H
#define GLOBAL_H


#define F_CPU 16000000UL
#define LEDPIN_TOGGLE              PINB |= 1<<5;

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "gpio.h"


extern uint8_t g_sreg;
#define DISABLE_INT() {g_sreg = SREG; cli();}
#define ENABLE_INT() {SREG = g_sreg;}

uint32_t now();
void init_timer();


typedef uint16_t uint;
typedef uint8_t byte;


#endif //GLOBAL_H
