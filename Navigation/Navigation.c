#include "global.h"
#include "sensor.h"
#include "rc.h"
#include "filter.h"
#include "math.h"

#include <stdlib.h>


#define MINTHROTTLE 1150
#define MAXTHROTTLE 1850
#define MINCOMMAND  1000

#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

int16_t angle[2];
int16_t heading;

int16_t axisPID[3];
int16_t errorAngleI[2] = {0,0};
int16_t errorGyroI[3] = {0,0,0};	
uint8_t dynP8[3], dynD8[3];	

//TBD
//int16_t rcCommand[4];
//int16_t  GPS_angle[2] = { 0, 0};
#define PIDITEMS 10
struct {
	uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];	
	int16_t angleTrim[2];
	uint8_t rcRate8;
	uint8_t rcExpo8;
	uint8_t rollPitchRate;
	uint8_t yawRate;
	uint8_t dynThrPID;
	uint8_t thrMid8;
	uint8_t thrExpo8;	  
} conf;

void init_navigation()
{
	int i;
	conf.P8[ROLL]  = 40;  conf.I8[ROLL] = 30; conf.D8[ROLL]  = 23;
	conf.P8[PITCH] = 40; conf.I8[PITCH] = 30; conf.D8[PITCH] = 23;
	conf.P8[YAW]   = 85;  conf.I8[YAW]  = 45;  conf.D8[YAW]  = 0;
	conf.angleTrim[ROLL]   = 0;
	conf.angleTrim[PITCH]  = 0;
	conf.rcRate8 = 90; conf.rcExpo8 = 65;
	conf.rollPitchRate = 0;
	conf.yawRate = 0;
	conf.dynThrPID = 0;
	conf.thrMid8 = 50; conf.thrExpo8 = 0;
	
	for(i=0;i<6;i++) {
		lookupPitchRollRC[i] = (2500+conf.rcExpo8*(i*i-25))*i*(int32_t)conf.rcRate8/2500;
	}
	for(i=0;i<11;i++) {
		int16_t tmp = 10*i-conf.thrMid8;
		uint8_t y = 1;
		if (tmp>0) y = 100-conf.thrMid8;
		if (tmp<0) y = conf.thrMid8;
		lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
		lookupThrottleRC[i] = MINTHROTTLE + (int32_t)(MAXTHROTTLE-MINTHROTTLE)* lookupThrottleRC[i]/1000;            // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
	}
		
}








void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
	//static uint32_t calibratedAccTime;
	uint16_t tmp,tmp2;
	uint8_t axis,prop1,prop2;

	#define BREAKPOINT 1500
	// PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
	if   (rcData[THROTTLE]<BREAKPOINT) {
		prop2 = 100;
	} else {
		if (rcData[THROTTLE]<2000) {
			prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
		} else {
			prop2 = 100 - conf.dynThrPID;
		}
	}

	for(axis=0;axis<3;axis++) {
		tmp = min(abs(rcData[axis]-MIDRC),500);
		#if defined(DEADBAND)
			if (tmp>DEADBAND) { tmp -= DEADBAND; }
			else { tmp=0; }
		#endif
		if(axis!=2) { //ROLL & PITCH
			tmp2 = tmp/100;
			rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
			prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
			prop1 = (uint16_t)prop1*prop2/100;
		} else {      // YAW
			rcCommand[axis] = tmp;
			prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
		}
		dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
		dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
		if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
	}
	tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
	tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
	tmp2 = tmp/100;
	rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
  
}


void pid_calculation()
{
	uint8_t axis;//, ACC_MODE = 0;
	int16_t error;//,errorAngle;
	int16_t PTerm,ITerm,DTerm;
	int16_t delta,deltaSum;
	int16_t delta1[3],delta2[3];
	int16_t lastGyro[3] = {0,0,0};
	
	//**** PITCH & ROLL & YAW PID ****    
	for(axis=0;axis<3;axis++) {
		/*if (ACC_MODE && axis<2 )  { //LEVEL MODE
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
		} else { */
			//ACRO MODE or YAW axis
		if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
		else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
		error -= gyroData[axis];

		PTerm = rcCommand[axis];
			
		errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
		if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
		ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
		//}
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



void task_navigation()
{
	int16_t _angle[3];
	get_inclination(_angle);

	angle[PITCH] = _angle[PITCH];
	angle[ROLL] = _angle[ROLL];
	heading = _angle[PITCH];
	
	annexCode();
	
	pid_calculation();	
}