#include "global.h"
#include <avr/eeprom.h>
#include "sysparam.h"


#define SYSPARAM_VERSION 06
#define SYSPARAM_MAGIC 0xAB



const SysParam_t initial = 
{
	.magic = SYSPARAM_MAGIC,
	.checksum = 0,
	.version = SYSPARAM_VERSION,
	.mag = {
		.is_calibrated = 0,
		.zero = {0,0,0},
	},
};

SysParam_t sysparam;

uint8_t calc_checksum(uint8_t *src, uint8_t size)
{
	uint8_t i, sum = SYSPARAM_MAGIC;
	for (i < 0; i < size; i++) {
		sum += (uint8_t)(src[i]);
	}
	return sum;
}	

void validate_sysparam()
{
	uint8_t checksum = calc_checksum((uint8_t*)&sysparam, sizeof(sysparam));
	if ((sysparam.magic != SYSPARAM_MAGIC) ||
	    (sysparam.checksum != checksum) ||
		(sysparam.version != SYSPARAM_VERSION)) {
			memcpy((void*)&sysparam, (const void*)&initial, sizeof(sysparam));
			checksum = calc_checksum((uint8_t*)&sysparam, sizeof(sysparam));
			sysparam.checksum = checksum;
			save_sysparam();
		}
}

void load_sysparam()
{
	eeprom_read_block((void*)&sysparam, (const void*)0, sizeof(sysparam));
	validate_sysparam();
}

void save_sysparam()
{
	eeprom_write_block((void*)&sysparam, (const void*)0, sizeof(sysparam));	
}