#ifndef NAVIGATION_H_
#define NAVIGATION_H_


void computeIMU();
void pid_calculation();
int16_t axisPID[3];
void task_navigation();
void init_navigation();


#endif /* NAVIGATION_H_ */