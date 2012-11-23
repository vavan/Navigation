#ifndef GLOBAL_H
#define GLOBAL_H


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "uart.h"
#include "gpio.h"


typedef uint16_t uint;
typedef uint8_t byte;


#endif //GLOBAL_H
