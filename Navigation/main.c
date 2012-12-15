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

//#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
extern int16_t gyroData[3];


void print_navigation()
{
	//int16_t motor[2];
	//motor[0] = PIDMIX(+1, 0, 0); //LEFT
	//motor[1] = PIDMIX(-1, 0, 0); //RIGHT
	
	
	//printf("%06d\n", axisPID[2]);
	printf("%06d\n", heading);
}


int main(void)
{
	int i;
	uint32_t prev_time = now();
	uint32_t gyro_time = now();
	uint16_t r = 0;
	
	init();
	
	for(i=0; i< 8; i ++) {
		rcData[i] = 1500;
	}
	rcData[THROTTLE] = 1200;
	
	while(1) {
		uint32_t time = now();
		int16_t d;
		
		
		
		//task_rc();
		task_navigation();
		//task_motors();
		//gyro_obtain();
		/*
		if (calibratingG == 0) {
			d = gyroADC[2];
			r += (float)d / (time - gyro_time);
			gyro_time = time;
		}*/			

		if (time - prev_time > 500000) {
			//int a = (int)(r * 1000);
			printf("%04d %04d %04d\n", magADC[ROLL]/3, magADC[PITCH]/3, magADC[YAW]/3);
			//printf("%04d %04d %04d\n", angle[ROLL], angle[PITCH], heading);
			prev_time = time;
		}
	}
}
