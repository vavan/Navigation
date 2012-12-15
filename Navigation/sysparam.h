/*
 * sysparam.h
 *
 * Created: 2012-11-28 19:20:45
 *  Author: x0163527
 */ 


#ifndef SYSPARAM_H_
#define SYSPARAM_H_

typedef struct
{
	uint8_t is_calibrated;
	int16_t zero[3];
} ZeroParam_t;


typedef struct 
{
	uint8_t magic;
	uint8_t checksum;
	uint8_t version;
} SysParamHeader_t;

typedef struct
{
	SysParamHeader_t header;
	ZeroParam_t mag;
	ZeroParam_t acc;
	ZeroParam_t gyro;
} SysParam_t;

extern SysParam_t sysparam;

void load_sysparam();

void save_sysparam();


#endif /* SYSPARAM_H_ */