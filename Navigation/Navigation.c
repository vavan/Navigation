#include "global.h"
#include "sensor.h"
#include <stdlib.h>

int16_t angle[2]    = {0,0};
int16_t axisPID[3];

#define PI  3.14159265358979323846
int16_t  accSmooth[3];


//TBD
int16_t rcCommand[4];
int16_t  GPS_angle[2] = { 0, 0};
#define PIDITEMS 10
struct {
	uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];	
	int16_t angleTrim[2];
} conf;


void computeIMU()
{    
#if defined(GYRO_SMOOTHING)
	static int16_t gyroSmooth[3] = {0,0,0};
	for (axis = 0; axis < 3; axis++) {
		gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+gyroData[axis]+1 ) / conf.Smoothing[axis]);
		gyroSmooth[axis] = gyroData[axis];
	}
	//   #elif defined(TRI)
	//     static int16_t gyroYawSmooth = 0;
	//     gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW])/3;
	//     gyroYawSmooth = gyroData[YAW];
#endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 100
#endif

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef MG_LPF_FACTOR
//#define MG_LPF_FACTOR 4
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 400.0f
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#ifndef GYR_CMPFM_FACTOR
#define GYR_CMPFM_FACTOR 200.0f
#endif

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
// +-2000/sec deg scale
//#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
// +- 200/sec deg scale
// 1.5 is emperical, not sure what it means
// should be in rad/sec
#else
#define GYRO_SCALE (1.0f/200e6f)
// empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
// !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct fp_vector {
	float X;
	float Y;
	float Z;
} fp_vector;

typedef union {
	float   A[3];
	fp_vector V;
} t_fp_vector;

int16_t _atan2(float y, float x){
#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
	float z = y / x;
	int16_t zi = abs((int16_t)(z * 100)); 
	int8_t y_neg = fp_is_neg(y);
	if ( zi < 100 ){
		if (zi > 10) 
		z = z / (1.0f + 0.28f * z * z);
		if (fp_is_neg(x)) {
			if (y_neg) z -= PI;
			else z += PI;
		}
	} else {
		z = (PI / 2.0f) - z / (z * z + 0.28f);
		if (y_neg) z -= PI;
	}
	z *= (180.0f / PI * 10); 
	return z;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
	fp_vector v_tmp = *v;
	v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
}

void getEstimatedAttitude()
{
	uint8_t axis;
	int32_t accMag = 0;
	static t_fp_vector EstG;
	int SMALL_ANGLES_25;



#if MAG
	static t_fp_vector EstM;
#endif
#if defined(MG_LPF_FACTOR)
	static int16_t mgSmooth[3]; 
#endif
#if defined(ACC_LPF_FACTOR)
	static float accLPF[3];
#endif
	static uint16_t previousT;
	uint16_t currentT = now();
	float scale, deltaGyroAngle[3];

	scale = (currentT - previousT) * GYRO_SCALE;
	previousT = currentT;

	// Initialization
	for (axis = 0; axis < 3; axis++) {
		deltaGyroAngle[axis] = gyroADC[axis]  * scale;
		#if defined(ACC_LPF_FACTOR)
		accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
		accSmooth[axis] = accLPF[axis];
		#define ACC_VALUE accSmooth[axis]
		#else  
		accSmooth[axis] = accADC[axis];
		#define ACC_VALUE accADC[axis]
		#endif
		//    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
		accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
		#if MAG
		#if defined(MG_LPF_FACTOR)
		mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
		#define MAG_VALUE mgSmooth[axis]
		#else  
		#define MAG_VALUE magADC[axis]
		#endif
		#endif
	}
	accMag = accMag*100/((int32_t)acc_1G*acc_1G);

	rotateV(&EstG.V,deltaGyroAngle);
#if MAG
	rotateV(&EstM.V,deltaGyroAngle);
#endif 

	if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
		SMALL_ANGLES_25 = 1;
	} else {
		SMALL_ANGLES_25 = 0;
	}

	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if ( ( 36 < accMag && accMag < 196 ) || SMALL_ANGLES_25 )
	for (axis = 0; axis < 3; axis++) {
		int16_t acc = ACC_VALUE;
		EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
	}
#if MAG
	for (axis = 0; axis < 3; axis++)
	EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
#endif

	// Attitude of the estimated vector
	angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
	angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;

#if MAG
	// Attitude of the cross product vector GxM
	heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
	heading += MAG_DECLINIATION * 10; //add declination
	heading = heading /10;
	if ( heading > 180)      heading = heading - 360;
	else if (heading < -180) heading = heading + 360;
#endif
}

int16_t errorAngleI[2] = {0,0};
int16_t gyroData[3] = {0,0,0};	
int16_t errorGyroI[3] = {0,0,0};	
uint8_t dynP8[3], dynD8[3];	

void pid_calculation()
{
	uint8_t axis, ACC_MODE = 1;
	int16_t error,errorAngle;
	int16_t PTerm,ITerm,DTerm;
	int16_t delta,deltaSum;
	int16_t delta1[3],delta2[3];
	int16_t lastGyro[3] = {0,0,0};
	
	//**** PITCH & ROLL & YAW PID ****    
	for(axis=0;axis<3;axis++) {
		if (ACC_MODE && axis<2 )  { //LEVEL MODE
			// 50 degrees max inclination
			errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
			#ifdef LEVEL_PDF
				PTerm      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
			#else
				PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
			#endif
			PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

			errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
			ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
		} else { //ACRO MODE or YAW axis
			if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
			else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
			error -= gyroData[axis];

			PTerm = rcCommand[axis];
			
			errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
			if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
			ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
		}
		if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
		else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

		delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
		lastGyro[axis] = gyroData[axis];
		deltaSum       = delta1[axis]+delta2[axis]+delta;
		delta2[axis]   = delta1[axis];
		delta1[axis]   = delta;

		if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
		else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
		
		axisPID[axis] =  PTerm + ITerm - DTerm;
	}

}