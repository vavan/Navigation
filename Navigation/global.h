#ifndef GLOBAL_H
#define GLOBAL_H


#define F_CPU 16000000UL
#define LEDPIN_TOGGLE           PINB |= 1<<5;
#define LEDPIN_OFF				PORTB &= ~(1<<5);

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

#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define min(a, b) ((a) < (b) ? (a) : (b))


uint32_t now();
void init_timer();


#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
/*
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDPOS     4
#define PIDPOSR    5
#define PIDNAVR    6
#define PIDLEVEL   7
#define PIDMAG     8
#define PIDVEL     9 // not used currently
*/

#endif //GLOBAL_H
