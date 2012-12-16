#ifndef NAVIGATION_H_
#define NAVIGATION_H_



void pid_calculation();
void task_navigation();
void init_navigation();

extern int16_t axisPID[3];
extern int16_t heading;
extern int16_t angle[2];

	
	
#endif /* NAVIGATION_H_ */