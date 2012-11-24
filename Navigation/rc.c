#include "global.h"
#include "rc.h"

#define RC1 D, 2
#define RC2 D, 6
#define RC3 D, 7

volatile byte rc_input[RC_CHANNEL_COUNT];

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
