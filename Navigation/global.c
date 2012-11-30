#include "global.h"

uint8_t g_sreg;

volatile uint32_t g_time;

ISR(TIMER0_OVF_vect) {
	g_time++;
}

uint32_t now() 
{
	uint32_t time;
	uint8_t sreg = SREG; 
	cli();
	time = (g_time << 8) + TCNT0;
	time = time / 2;
	SREG = sreg;
	return time;
}

void init_timer()
{
	g_time = 0;
	TCCR0B = (1 << CS01);
	TIMSK0 = (1 << TOIE0);
}