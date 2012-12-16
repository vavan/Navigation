#include "global.h"
#include "motor.h"
#include "rc.h"
#include "sensor.h"
#include "navigation.h"
#include "sysparam.h"


#define LED B, 5



void task_rc()
{
	//uint16_t a;
	//cli();
	//a = rcData[0];
	//sei();
	//a &= ~0x03f;
		
	//set_motor(RMOTOR, a);
	//set_motor(LMOTOR, a);
}


void init()
{

	
	cli();
	init_timer();
	init_uart();
	init_motors();
	//init_RC();
	sei();
	
	load_sysparam();
	init_sensors();
	init_navigation();	

	printf("__START__\n");
}



void print_navigation()
{
	//int16_t motor[2];
	//motor[0] = PIDMIX(+1, 0, 0); //LEFT
	//motor[1] = PIDMIX(-1, 0, 0); //RIGHT
	
	
	//printf("%06d\n", axisPID[2]);
	printf("%06d\n", heading);
}


int main()
{
	int i;
	uint32_t print_time = now();
	uint32_t navigation_time = now();
	uint16_t count = 0;
	
	init();
	
	for(i=0; i< 8; i ++) {
		rcData[i] = 1500;
	}
	rcData[THROTTLE] = 1200;
	
	while(1) {
		int16_t value[3];
		int16_t delta[3];
		
		uint32_t time = now();
		
		gyro_obtain(value);
		gyro_get_angle(value, delta);
		
		
		
		//task_rc();
		//if (time - navigation_time > 10000) {
		//	task_navigation();
		//	navigation_time = time;
		//}
		
		//task_motors();
		

		if (time - print_time > 1000000) {
			printf("%04d %04d %04d\n", delta[ROLL], delta[PITCH], delta[YAW]);
			//printf("%04d %04d %04d\n", accSmooth[ROLL], accSmooth[PITCH], accSmooth[YAW]);
			//printf("%04d %04d %04d\n", gyroData[ROLL]/8, gyroData[PITCH]/8, gyroData[YAW]/8);
			//printf("%04d %04d %04d\n", magADC[ROLL]/3, magADC[PITCH]/3, magADC[YAW]/3);
			//printf("%04d %04d %04d\n", angle[ROLL], angle[PITCH], heading);
			//printf("%04d\n", count);
			print_time = time;
		}
	}
}
