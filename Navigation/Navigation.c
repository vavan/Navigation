#include "global.h"
#include "motor.h"
#include "rc.h"
#include "sensor.h"
#include "sysparam.h"


#define LED B, 5

void task_rc()
{
	uint16_t a;
	//cli();
	//a = rc_input[0];
	//sei();
	//a &= ~0x03f;
		
	//set_motor(RMOTOR, a);
	set_motor(LMOTOR, a);
}

void init()
{
	cli();
	load_sysparam();
	init_timer();
	init_uart();
	init_motors();
	init_RC();
	init_sensors();
	sei();
	
	printf("__START__\n");
}

int main(void)
{
	uint32_t prev_time = now();
	
	init();
	
	while(1) {
		uint32_t time = now();
		
		task_rc();
		task_sensors();

		if (time - prev_time > 500000) {
			printf("%06ld: %d\n", time, magADC[0]);
			prev_time = time;
		}
	}
}
