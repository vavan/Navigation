#include "global.h"
#include <avr/eeprom.h>
#include <string.h>
#include "sysparam.h"


#define SYSPARAM_VERSION 0x08
#define SYSPARAM_MAGIC 0xAB



const SysParam_t initial = 
{
	.header = {
		.magic = SYSPARAM_MAGIC,
		.checksum = 0,
		.version = SYSPARAM_VERSION,
	},		
	.mag = {
		.is_calibrated = 0,
		.zero = {0,0,0},
	},
	.acc = {
		.is_calibrated = 0,
		.zero = {0,0,0},
	},
	.gyro = {
		.is_calibrated = 0,
		.zero = {0,0,0},
	},	
};

SysParam_t sysparam;

uint8_t calc_checksum(SysParam_t *sysparam, uint8_t size)
{
	uint8_t i, sum = SYSPARAM_MAGIC;
	uint8_t *src = (uint8_t*)sysparam + sizeof(SysParamHeader_t);
	for (i = 0; i < size; i++) {
		sum += (uint8_t)(src[i]);
	}
	return sum;
}	

void validate_sysparam()
{
	uint8_t checksum = calc_checksum(&sysparam, sizeof(sysparam));
	if ((sysparam.header.magic != SYSPARAM_MAGIC) ||
	    (sysparam.header.checksum != checksum) ||
		(sysparam.header.version != SYSPARAM_VERSION)) {
			printf("------------init- ");
			memcpy((void*)&sysparam, (const void*)&initial, sizeof(sysparam));
			save_sysparam();
		}
}

void load_sysparam()
{
	eeprom_read_block((void*)&sysparam, (const void*)0, sizeof(sysparam));
	printf("------------load- %d", sysparam.header.checksum);
	validate_sysparam();
}

void save_sysparam()
{
	sysparam.header.checksum = calc_checksum(&sysparam, sizeof(sysparam));
	eeprom_write_block((const void*)&sysparam, (void*)0, sizeof(sysparam));	
	printf("------------save- %d", sysparam.header.checksum);
}