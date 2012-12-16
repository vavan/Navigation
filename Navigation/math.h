/*
 * math.h
 *
 * Created: 2012-12-16 21:53:07
 *  Author: x0163527
 */ 


#ifndef MATH_H_
#define MATH_H_


#define PI  3.14159265358979323846

typedef struct 
{
	float X;
	float Y;
	float Z;
} vector_t;

typedef union 
{
	float A[3];
	vector_t V;
} vector_u;

int16_t _atan2(float y, float x);
void rotateV(vector_t *v,float* delta);


#endif /* MATH_H_ */