#ifndef FILTER_H_
#define FILTER_H_

#include "math.h"

extern int16_t gyroData[3];
extern int16_t accSmooth[3];

void gyro_average(int16_t *gyroSmooth);
void acc_smooth(int16_t *accSmooth);
void mag_smooth(int16_t *mgSmooth);

void gyro_get_angle(float * deltaGyroAngle);
void complementary_filter(int16_t *accSmooth, int16_t *mgSmooth, vector_u *EstG, vector_u *EstM);
void filter_sensors(vector_u *EstG, vector_u *EstM);
void get_inclination(int16_t *angle);




#endif /* FILTER_H_ */