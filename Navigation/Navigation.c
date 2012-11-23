#include "global.h"
#include "control.h"


int main(void)
{
	int input = 123;
	cli();
	
	init_uart();
	init_motors();
	init_RC();
	
	sei();
	
	printf("__START__");
	
	while(1) {
		int a;
		cli();
		a = rc_input[0];
		sei();
		printf("  %d\n", a);
		a -= 85;
		
		set_motor(RMOTOR, a);
		set_motor(LMOTOR, a);
		
	
		//_delay_ms(1000);
	}

}
