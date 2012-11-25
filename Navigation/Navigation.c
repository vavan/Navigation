#include "global.h"
#include "motor.h"
#include "rc.h"
#include "sensor.h"




int main(void)
{
	//int input = 123;
	cli();
	
	init_uart();
	init_motors();
	init_RC();
	init_sensors();
	
	sei();
	
	printf("__START__");
	
	while(1) {
		int a, i;
		cli();
		a = rc_input[0];
		sei();
		a -= 85;
		
		
		//gyro_getADC();
		//acc_getADC();
		mag_getADC();
		
		//for (i = 0; i < 3; i++) {
			i = 0;
			printf("%d\n", magADC[i]);
		//}
		
		
		set_motor(RMOTOR, a);
		set_motor(LMOTOR, a);
		
		//if (calibratingA == 0) _delay_ms(50);
	}

}
