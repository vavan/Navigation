#include "global.h"
#include "control.h"

#define LMOTORPIN B, 1
#define RMOTORPIN B, 2


void init_motors()
{
	GPIO_DirOut(LMOTORPIN);
	GPIO_DirOut(RMOTORPIN);
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) |(1<<WGM11);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);
	ICR1 = 40000;
	
	set_motor(RMOTOR, 0);
	set_motor(LMOTOR, 0);
}

void set_motor(int motor, int value)
{
	if ((value >= -100) && (value <= +100)) {
		value = (value * 5) + 1500;
		if (motor == RMOTOR) OCR1A = value * 2;
		if (motor == LMOTOR) OCR1B = value * 2;
	}
}

#define RC1 D, 2
#define RC2 D, 6
#define RC3 D, 7

volatile byte rc_input[3];

ISR(PCINT2_vect) {
	byte delta;
	byte time = TCNT0;
	static byte edge_time[RC_CHANNEL_COUNT];
	
	if (GPIO_Get(RC1)) {
		edge_time[0] = time;
	} else {
		delta = time - edge_time[0];
		rc_input[0] = delta;
	}
}

void init_RC()
{
	GPIO_DirIn(RC1);
	GPIO_DirIn(RC2);
	GPIO_DirIn(RC3);

	PCICR = 1 << PCIE2;
	PCMSK2 = GPIO_Mask(RC1) | GPIO_Mask(RC2) | GPIO_Mask(RC3);
	
	TCCR0B = (1<<CS01) | (1<<CS00);
}
