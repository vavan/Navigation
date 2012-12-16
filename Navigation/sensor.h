#ifndef SENSOR_H_
#define SENSOR_H_

#define GYRO 1
#define ACC 1
#define MAG 1
#define BARO 1

void gyro_obtain(int16_t *value);
void mag_obtain(int16_t *value);
void acc_obtain(int16_t *value);	

void init_sensors();
//extern int16_t  gyroADC[3],accADC[3],magADC[3];

extern uint16_t acc_1G;

#endif /* SENSOR_H_ */