#include "global.h"
#include "sensor.h"
#include "math.h"


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



/************************************************************************/
/*           Filter settings                                            */
/************************************************************************/

#define MAG_DECLINIATION  0.0f


// Gyro
#define GYRO_SMOOTH_DEPTH 3


// Acc
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 100
#endif

// Mag
/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef MG_LPF_FACTOR
//#define MG_LPF_FACTOR 4
#endif

// Complementary filters
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


/************************************************************************/
/*           Filter constants                                           */
/************************************************************************/
#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
// +-2000/sec deg scale
//#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)
// +- 200/sec deg scale
// 1.5 is emperical, not sure what it means
// should be in rad/sec

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))

#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))


/************************************************************************/
/*                                                                      */
/************************************************************************/
int16_t gyroData[3];
int16_t accSmooth[3];



/************************************************************************/
/*           Gyro average                                               */
/************************************************************************/
void gyro_average(int16_t *gyroPhy, int16_t *gyroSmooth)
{
	static int16_t gyro_phy_prev[3][GYRO_SMOOTH_DEPTH];
	static int gyro_smoot_index = 0;

	for (int axis = 0; axis < 3; axis++) {
		gyro_phy_prev[axis][gyro_smoot_index] =  gyroPhy[axis];
	}
	if (gyro_smoot_index < GYRO_SMOOTH_DEPTH) {
		gyro_smoot_index++;
	} else {
		gyro_smoot_index = 0;
	}
	
	memset(gyroSmooth, 0, sizeof(gyroSmooth));
	for (int i = 0; i < GYRO_SMOOTH_DEPTH; i++) {
		for (int axis = 0; axis < 3; axis++) {
			gyroSmooth[axis] += gyro_phy_prev[axis][i];
		}
	}
	for (int axis = 0; axis < 3; axis++) {
		gyroSmooth[axis] /= GYRO_SMOOTH_DEPTH;
	}
}

void gyro_get_angle(int16_t *gyroPhy, float *deltaGyroAngle) 
{
	float scale;
	static uint32_t previousT;
	uint32_t currentT = now();

	scale = (currentT - previousT) * GYRO_SCALE;
	previousT = currentT;
	
	for (int axis = 0; axis < 3; axis++) {
		deltaGyroAngle[axis] = gyroPhy[axis]  * scale;
	}
}

/************************************************************************/
/*           Acc smooth                                                 */
/************************************************************************/
void acc_smooth(int16_t *accPhy, int16_t *accSmooth)
{
	static float accLPF[3];
	
	for (int axis = 0; axis < 3; axis++) {
		#if defined(ACC_LPF_FACTOR)
			accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accPhy[axis] * (1.0f/ACC_LPF_FACTOR);
			accSmooth[axis] = accLPF[axis];
		#else
			accSmooth[axis] = accPhy[axis];
		#endif
	}
}

/************************************************************************/
/*           Mag smooth                                                 */
/************************************************************************/
void mag_smooth(int16_t *magPhy, int16_t *magSmooth)
{
	for (int axis = 0; axis < 3; axis++) {
		#if defined(MG_LPF_FACTOR)
			magSmooth[axis] = (magSmooth[axis] * (MG_LPF_FACTOR - 1) + magPhy[axis]) / MG_LPF_FACTOR;
		#else
			magSmooth[axis] = magPhy[axis];
		#endif
	}
}

/************************************************************************/
/*           Complementary filter                                       */
/************************************************************************/
void complementary_filter(int16_t *accSmooth, int16_t *mgSmooth, vector_u *EstG, vector_u *EstM)
{
	int32_t accMag = 0;
	int16_t acc_25deg = acc_1G * 0.423;
	
	int SMALL_ANGLES_25; //TBD if it is 0 - ACC is too much inclined or non calibrated
	
	for (int axis = 0; axis < 3; axis++) {
		accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;	
	}
	accMag = accMag*100/((int32_t)acc_1G*acc_1G);	
	
	if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
		SMALL_ANGLES_25 = 1;
	} else {
		SMALL_ANGLES_25 = 0;
	}

	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if ( ( 36 < accMag && accMag < 196 ) || SMALL_ANGLES_25 ) {
		for (int axis = 0; axis < 3; axis++) {
			EstG->A[axis] = (EstG->A[axis] * GYR_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
		}
	}

	for (int axis = 0; axis < 3; axis++) {
		EstM->A[axis] = (EstM->A[axis] * GYR_CMPFM_FACTOR  + mgSmooth[axis]) * INV_GYR_CMPFM_FACTOR;
	}
}


/************************************************************************/
/*           Apply filters                                              */
/************************************************************************/
void filter_sensors(vector_u *EstG, vector_u *EstM)
{
	static int16_t magSmooth[3];
	float deltaGyroAngle[3];
	int16_t  gyroPhy[3], accPhy[3], magPhy[3];

	gyro_obtain(gyroPhy);
	acc_obtain(accPhy);
	mag_obtain(magPhy);

	gyro_average(gyroPhy, gyroData);
	gyro_get_angle(gyroPhy, deltaGyroAngle);
	acc_smooth(accPhy, accSmooth);
	mag_smooth(magPhy, magSmooth);

	rotateV(&EstG->V, deltaGyroAngle);
	rotateV(&EstM->V, deltaGyroAngle);
	
	complementary_filter(accSmooth, magSmooth, EstG, EstM);
}

void get_inclination(int16_t *angle)
{
	vector_u EstG;
	vector_u EstM;
	filter_sensors(&EstG, &EstM);
		
	// Attitude of the estimated vector
	angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
	angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;

	// Attitude of the cross product vector GxM
	angle[YAW] = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
	angle[YAW] += MAG_DECLINIATION * 10; //add declination
	angle[YAW] = angle[YAW] /10;
	if ( angle[YAW] > 180)      angle[YAW] -= 360;
	else if (angle[YAW] < -180) angle[YAW] += 360;
}