#ifndef SENSOR_H_
#define SENSOR_H_

void task_sensors();
void init_sensors();
extern int16_t  gyroADC[3],accADC[3],magADC[3];

extern uint16_t calibratingA;
extern uint16_t calibratingG;

#endif /* SENSOR_H_ */