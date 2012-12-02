#include "global.h"
#include "rc.h"

#define RC1 D, 2
#define RC2 D, 6
#define RC3 D, 7

volatile uint16_t rc_input[RC_CHANNEL_COUNT];

ISR(PCINT2_vect) {
	int i;
	uint16_t delta;
	static uint32_t edge_time[RC_CHANNEL_COUNT];
	//uint8_t pin[RC_CHANNEL_COUNT];
	uint32_t time = now();

	//pin[0] = GPIO_Get(RC1);
	//pin[1] = GPIO_Get(RC2);
	//pin[2] = GPIO_Get(RC3);
	//sei();
	i = 0;
	//for (i=0; i<RC_CHANNEL_COUNT; i++) {
		if (GPIO_Get(RC1)) {
			edge_time[i] = time;
		} else {
			delta = time - edge_time[i];
			rc_input[i] = delta;
		}
	//}
}

void init_RC()
{
	GPIO_DirIn(RC1);
	GPIO_DirIn(RC2);
	GPIO_DirIn(RC3);

	PCICR = 1 << PCIE2;
	PCMSK2 = GPIO_Mask(RC1) | GPIO_Mask(RC2) | GPIO_Mask(RC3);
}
