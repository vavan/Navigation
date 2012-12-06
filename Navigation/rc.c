#include "global.h"
#include "rc.h"

#define RC_R D, 6
#define RC_P D, 7
#define RC_Y D, 6
#define RC_T D, 2

//volatile uint16_t rc_input[RC_CHANNEL_COUNT];
volatile uint16_t rcData[8];//[RC_CHANNEL_COUNT];
int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

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
		if (GPIO_Get(RC_T)) {
			edge_time[i] = time;
		} else {
			delta = time - edge_time[i];
			rcData[i] = delta;
		}
	//}
}

void init_RC()
{
	GPIO_DirIn(RC_R);
	GPIO_DirIn(RC_P);
	GPIO_DirIn(RC_Y);
	GPIO_DirIn(RC_T);

	PCICR = 1 << PCIE2;
	PCMSK2 = GPIO_Mask(RC_R) | GPIO_Mask(RC_P) | GPIO_Mask(RC_Y) | GPIO_Mask(RC_T);
}
