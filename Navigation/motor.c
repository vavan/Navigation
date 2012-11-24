#include "global.h"
#include "motor.h"

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

