#ifndef NAVIGATION_H_
#define NAVIGATION_H_


void computeIMU();
void pid_calculation();
int16_t axisPID[3];
void task_navigation();
void init_navigation();


extern int16_t heading;
extern int16_t angle[2];
extern int16_t accSmooth[3];
extern int16_t gyroData[3];
	
	
#endif /* NAVIGATION_H_ */