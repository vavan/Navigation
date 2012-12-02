/*
 * i2c_sync.h
 *
 * Created: 11/25/2012 11:42:48 PM
 *  Author: tvb
 */ 


#ifndef I2C_SYNC_H_
#define I2C_SYNC_H_

void i2c_init(void);
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size);
void swap_endianness(void *buf, size_t size);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
void i2c_rep_start(uint8_t address);
void i2c_write(uint8_t data );
void i2c_stop(void);
uint8_t i2c_readAck();
uint8_t i2c_readNak(void);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
	

#endif /* I2C_SYNC_H_ */