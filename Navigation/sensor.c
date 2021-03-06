#include "global.h"
#include <stddef.h>
#include <stdlib.h>
#include "i2c_sync.h"
#include "sysparam.h"
#include "sensor.h"



/************************************************************************/
/*           Sensor outputs                                             */
/************************************************************************/

//int16_t  gyroADC[3],accADC[3],magADC[3];

uint16_t acc_1G;  



/************************************************************************/
/*           I2C Helpers                                                */
/************************************************************************/
uint8_t rawADC[6];

void i2c_getSixRawADC(uint8_t add, uint8_t reg) 
{
	i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}




/************************************************************************/
/*           Sensors configuration                                      */
/************************************************************************/
// Gyro & Acc calibration
#define CALIBRATION_COUNT 400
#define CALIBRATION_LAST 1


// Physical sensors
#define ITG3200
#define BMA180
#define HMC5883
#define BMP085

// Orientation
#define ACC_ORIENTATION(X, Y, Z)  {value[ROLL] = -X; value[PITCH] = -Y; value[YAW] =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {value[ROLL] =  Y; value[PITCH] = -X; value[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {value[ROLL] =  X; value[PITCH] =  Y; value[YAW] = -Z;}



// TBD
#define POWERPIN_PINMODE           ;
#define POWERPIN_ON                ;
#define POWERPIN_OFF               ;


/************************************************************************/
/*                    GYRO                                              */
/************************************************************************/

// ************************************************************************************************************
// I2C Gyroscope ITG3200
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C address is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************

#if defined(ITG3200)

/*********************    Lowpass filter for some gyros    ****************************/
/* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
    to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
    It will not help on feedback wobbles, so change only when copter is randomly twitching and all dampening and
    balancing options ran out. Uncomment only one option!
    IMPORTANT! Change low pass filter setting changes PID behavior, so re-tune your PID's after changing LPF.*/
#define ITG3200_LPF_256HZ 0
#define ITG3200_LPF_188HZ 1
#define ITG3200_LPF_98HZ  2
#define ITG3200_LPF_42HZ  3
#define ITG3200_LPF_20HZ  4
#define ITG3200_LPF_10HZ  5

// Actual LPF value
#define ITG3200_DLPF_CFG  ITG3200_LPF_256HZ


// Constants
#define ITG3200_ADDRESS 0x68


// Get physical values
void gyro_getADC (int16_t *value) 
{
	i2c_getSixRawADC(ITG3200_ADDRESS,0X1D);
	GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/4 , // range: +/- 8192; +/- 2000 deg/sec
					((rawADC[2]<<8) | rawADC[3])/4 ,
					((rawADC[4]<<8) | rawADC[5])/4 );
}

// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
void gyro_calibrate(int16_t *value) 
{
	static int32_t g[3];
	static uint16_t calibrating = CALIBRATION_COUNT;
	
	if (calibrating == CALIBRATION_COUNT) { // start
		for (int axis = 0; axis < 3; axis++) {
			g[axis] = 0;
		}
	} else if (calibrating > CALIBRATION_LAST) { // proceed
		for (int axis = 0; axis < 3; axis++) {
			g[axis] += value[axis];
		}		
	} else { // finish
		for (int axis = 0; axis < 3; axis++) {
			sysparam.gyro.zero[axis] = g[axis]/CALIBRATION_COUNT;
		}
		sysparam.gyro.is_calibrated = 1;
		save_sysparam();
	}
	calibrating--;
}

//anti gyro glitch, limit the variation between two consecutive readings
void gyro_anti_glitch(int16_t *value) 
{
	static int16_t previousGyroADC[3] = {0,0,0};
	for (int axis = 0; axis < 3; axis++) {
		value[axis] = constrain(value[axis], previousGyroADC[axis]-800, previousGyroADC[axis]+800);
		previousGyroADC[axis] = value[axis];
	}
}

void gyro_obtain(int16_t *value) 
{
	gyro_getADC(value);
	gyro_anti_glitch(value);
	
	if (sysparam.gyro.is_calibrated) {
		for (int axis = 0; axis < 3; axis++) {
			value[axis] -= sysparam.gyro.zero[axis];
		}
	} else {
		gyro_calibrate(value);
	}			
}

void gyro_init() 
{
  delay(100);
  i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80); //register: Power Management  --  value: reset device
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
  
  delay(100);
}
#endif



/************************************************************************/
/*      Accelerometer                                                   */
/************************************************************************/

// ************************************************************************************************************
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC)
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:      |                                           bw<3:0> |                        tcs<3:0> |
//                      |                                             150Hz |                        xxxxxxxx |
// 0x30    tco_z:       |                                                tco_z<5:0>    |     mode_config<1:0> |
//                      |                                                xxxxxxxxxx    |                   00 |
// 0x35    offset_lsb1: |          offset_x<3:0>              |                   range<2:0>       | smp_skip |
//                      |          xxxxxxxxxxxxx              |                    8G:   101       | xxxxxxxx |
// ************************************************************************************************************
#if defined(BMA180)

/*** I2C address ***/
#define BMA180_ADDRESS 0x40

void acc_getADC (int16_t *value) 
{
	i2c_getSixRawADC(BMA180_ADDRESS,0x02);
	//usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
	ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/16 ,
					((rawADC[3]<<8) | rawADC[2])/16 ,
					((rawADC[5]<<8) | rawADC[4])/16 );
}

void acc_calibrate(int16_t *value)
{
	// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
	static int32_t a[3];
	static uint16_t calibrating = CALIBRATION_COUNT;
	
	if (calibrating == CALIBRATION_COUNT) {
		// start
		for (int axis = 0; axis < 3; axis++) {
			a[axis] = 0;
		}
	} else if (calibrating > CALIBRATION_LAST) {
		// proceed
		for (int axis = 0; axis < 3; axis++) {
			a[axis] += value[axis];
		}			
	} else {
		// finish
		for (int axis = 0; axis < 3; axis++) {
			sysparam.acc.zero[axis] = a[axis]/CALIBRATION_COUNT;
		}
		sysparam.acc.zero[YAW] -= acc_1G;
		sysparam.acc.is_calibrated = 1;
		save_sysparam();
	} 
	calibrating--;
}

void acc_obtain(int16_t *value) 
{	
	acc_getADC(value);	
	if (sysparam.acc.is_calibrated) {
		for(int axis = 0; axis < 3; axis++) {
			value[axis] -= sysparam.acc.zero[axis];
		}
	} else {
		acc_calibrate(value);
	}					
}

void acc_init() 
{
	delay(10);
	//default range 2G: 1G = 4096 unit.
	i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
	delay(5);
	uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
	control = control & 0x0F;        // save tcs register
	control = control | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
	i2c_writeReg(BMA180_ADDRESS, 0x20, control);
	delay(5);
	control = i2c_readReg(BMA180_ADDRESS, 0x30);
	control = control & 0xFC;        // save tco_z register
	control = control | 0x00;        // set mode_config to 0
	i2c_writeReg(BMA180_ADDRESS, 0x30, control);
	delay(5);
	control = i2c_readReg(BMA180_ADDRESS, 0x35);
	control = control & 0xF1;        // save offset_x and smp_skip register
	control = control | (0x05 << 1); // set range to 8G
	i2c_writeReg(BMA180_ADDRESS, 0x35, control);
	delay(5);
	acc_1G = 255;
}
#endif



/************************************************************************/
/*    Magnetometer                                                      */
/************************************************************************/

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#if defined(HMC5883)
#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03
#define MAG_CALIBRATION_DURATION 30000000

static float   magCal[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

void mag_getADC(int16_t *value) 
{
	i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
	MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
					((rawADC[4]<<8) | rawADC[5]) ,
					((rawADC[2]<<8) | rawADC[3]) );
}

void mag_calibrate(int16_t *value)
{
	static int16_t magZeroTempMin[3];
	static int16_t magZeroTempMax[3];
	static uint32_t calibration_start = 0; 

	if (calibration_start == 0) {
		// start
		calibration_start = now();
	    for(int axis=0; axis<3; axis++) {
		    magZeroTempMin[axis] = value[axis];
		    magZeroTempMax[axis] = value[axis];
	    }
	} else if (now() - calibration_start < MAG_CALIBRATION_DURATION) {
		// proceed
	    LEDPIN_TOGGLE();
		for(int axis=0; axis<3; axis++) {
			if (value[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = value[axis];
			if (value[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = value[axis];
		}
		
	} else {
		// finish
		for(int axis=0; axis<3; axis++) {
			sysparam.mag.zero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
		}
		
		sysparam.mag.is_calibrated = 1;
		save_sysparam();
		LEDPIN_OFF();
	}
}

void mag_obtain(int16_t *value) 
{ 
	mag_getADC(value);
	for (int axis = 0; axis < 3; axis++) {
		value[axis]  = value[axis]  * magCal[axis];
	}		
  
	if (sysparam.mag.is_calibrated) {
		if (magInit) {
			for (int axis = 0; axis < 3; axis++) {
				value[axis] -= sysparam.mag.zero[axis];
			}
		}			
	} else {
		mag_calibrate(value);
	}	
}
  
void mag_init() 
{
	int16_t magPhy[3];
	delay(100);
	// force positiveBias
	i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
	delay(50);
	// set gains for calibration
	i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
	i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

	// read values from the compass -  self test operation
	// by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
	// The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
	delay(100);
	mag_getADC(magPhy);
	delay(10);
	magCal[ROLL]  =  1160.0 / abs(magPhy[ROLL]);
	magCal[PITCH] =  1160.0 / abs(magPhy[PITCH]);
	magCal[YAW]   =  1080.0 / abs(magPhy[YAW]);

	// leave test mode
	i2c_writeReg(MAG_ADDRESS, 0x00, 0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	i2c_writeReg(MAG_ADDRESS, 0x01, 0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	i2c_writeReg(MAG_ADDRESS, 0x02, 0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

	magInit = 1;
}

#endif



















































// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

#if defined(BMP085)
#define BMP085_ADDRESS 0x77
static int32_t  pressure;

static struct {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;  
#define OSS 2 //we can get more unique samples and get better precision using average

void i2c_BMP085_readCalibration(){
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void i2c_BMP085_UT_Start();
void i2c_BMP085_UT_Read();

void  Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  i2c_BMP085_UT_Start(); 
  delay(5);
  i2c_BMP085_UT_Read();
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);
  i2c_rep_start(BMP085_ADDRESS<<1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS<<1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}



void Baro_update() {
  //TBD //if (currentTime < bmp085_ctx.deadline) return; 
  //bmp085_ctx.deadline = currentTime;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  switch (bmp085_ctx.state) {
    case 0: 
      i2c_BMP085_UT_Start(); 
      bmp085_ctx.state++; bmp085_ctx.deadline += 4600; 
      break;
    case 1: 
      i2c_BMP085_UT_Read(); 
      bmp085_ctx.state++; 
      break;
    case 2: 
      i2c_BMP085_UP_Start(); 
      bmp085_ctx.state++; bmp085_ctx.deadline += 14000; 
      break;
    case 3: 
      i2c_BMP085_UP_Read(); 
      i2c_BMP085_Calculate(); 
      //BaroAlt = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f; //centimeter
      bmp085_ctx.state = 0; bmp085_ctx.deadline += 5000; 
      break;
  } 
}
#endif








void init_sensors() 
{
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) gyro_init();
  //if (BARO) Baro_init();
  if (MAG) mag_init();
  if (ACC) acc_init();

}
