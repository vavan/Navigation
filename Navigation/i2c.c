#include "i2c.h"
#include <util/twi.h>

i2c_state_e i2c_state = I2C_IDLE;
uint8_t twi_status;
uint16_t i2c_dev_comm;
uint16_t i2c_dev_addr;
uint8_t i2c_data;
uint8_t i2c_retries;
uint8_t i2c_success = 0;
uint8_t i2c_dev_comm_len;

void i2c_startwriting(uint8_t dev_addr, uint16_t dev_comm, uint8_t dev_comm_len, uint8_t data)
{
	i2c_retries = 0;
	i2c_dev_addr = dev_addr;
	i2c_dev_comm = dev_comm;
	i2c_dev_comm_len = dev_comm_len;
	i2c_data = data;
	i2c_success = 0;
	i2c_state = I2C_WRITE_RETRY;  
}

void i2c_startreading(uint8_t dev_addr, uint16_t dev_comm, uint8_t dev_comm_len)
{
	i2c_retries = 0;
	i2c_dev_addr = dev_addr;
	i2c_dev_comm = dev_comm;
	i2c_dev_comm_len = dev_comm_len;
	i2c_data = 0;
	i2c_success = 0;
	i2c_state = I2C_READ_RETRY;  
}

void i2c_transmit(uint8_t type) 
{
	switch(type) 
	{
		case I2C_START:    // Send Start Condition
			TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
			break;
		case I2C_DATA:     // Send Data
			TWCR = (1 << TWINT) | (1 << TWEN);
			break;
		case I2C_STOP:     // Send Stop Condition
			TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
			break;
	}
}

void check_i2c_status(void)
{
	if (
		(I2C_WRITE_START_WAIT == i2c_state) || (I2C_WRITE_SLAVE_ADDR_WAIT == i2c_state) || 
		(I2C_WRITE_SLAVE_COMM1_WAIT == i2c_state) || (I2C_WRITE_SLAVE_COMM2_WAIT == i2c_state) ||
		(I2C_WRITE_DATA_WAIT == i2c_state) || (I2C_WRITE_STOP_WAIT == i2c_state) ||
		(I2C_READ_START_WAIT == i2c_state) || (I2C_READ_SLAVE_ADDR_WAIT == i2c_state) || 
		(I2C_READ_SLAVE_COMM1_WAIT == i2c_state) || (I2C_READ_SLAVE_COMM2_WAIT == i2c_state) ||
		(I2C_READ_START2_WAIT == i2c_state) || (I2C_READ_SLAVE_ADDR2_WAIT == i2c_state) || 
		(I2C_READ_DATA_WAIT == i2c_state) || (I2C_READ_STOP_WAIT == i2c_state)
		)
	{
		// Wait for TWINT flag set in TWCR Register
		if (TWCR & (1 << TWINT))
		{
			// Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
			twi_status = (TWSR & 0xF8);
			i2c_state++;
		}
	}
	if (I2C_IDLE == i2c_state)
	{
        //check_temperature_status(i2c_success, i2c_data);
        //act_temperature();              
	}
}

void act_i2c(void)
{
	switch (i2c_state)
	{
		/////////////////////////////  WRITE
		case I2C_WRITE_START_END:
		{
			// Check the TWI Status
			if (twi_status == TW_MT_ARB_LOST)
			{
				i2c_state = I2C_WRITE_RETRY;
			}
			else if ((twi_status != TW_START) && (twi_status != TW_REP_START))
			{
				i2c_state = I2C_WRITE_STOP;
			}
			else
			{
				// Send slave address (SLA_W)
				TWDR = ((i2c_dev_addr & 0x0E) << 1) | TW_WRITE;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}
		case I2C_WRITE_SLAVE_ADDR_END:
		{
			// Check the TWSR status
			if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST))
			{
				i2c_state = I2C_WRITE_RETRY;
			}
			else if (twi_status != TW_MT_SLA_ACK)
			{
				i2c_state = I2C_WRITE_STOP;
			}
			else
			{
				// Send the Low 8-bit of I2C command
				TWDR = i2c_dev_comm & 0xFF;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				if (1 == i2c_dev_comm_len)
				{
					i2c_state = I2C_WRITE_SLAVE_COMM2_WAIT;
				}
				else
				{
					i2c_state = I2C_WRITE_SLAVE_COMM1_WAIT;
				}
				break;
			}
		}
		case I2C_WRITE_SLAVE_COMM1_END:
		{
			// Check the TWSR status
			if (twi_status != TW_MT_DATA_ACK)
			{
				i2c_state = I2C_WRITE_STOP;
			}
			else
			{
				// Send the High 8-bit of I2C command
				TWDR = i2c_dev_comm >> 8;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}
		case I2C_WRITE_SLAVE_COMM2_END:
		{
			// Check the TWSR status
			if (twi_status != TW_MT_DATA_ACK)
			{
				i2c_state = I2C_WRITE_STOP;
			}
			else
			{
				// Put data into data register and start transmission
				TWDR = i2c_data;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}
		case I2C_WRITE_DATA_END:
		{
			if (twi_status != TW_MT_DATA_ACK)
			{
				i2c_success = 0;
			}
			else
			{
				i2c_success = 1;
			}
			i2c_state = I2C_WRITE_STOP;
		}
		case I2C_WRITE_STOP:
		{
			i2c_transmit(I2C_STOP);
			i2c_state++;
			break;
		}
		case I2C_WRITE_STOP_END:
		{
			i2c_state = I2C_IDLE;			
			break;
		}
		case I2C_WRITE_RETRY:
		{
			if (i2c_retries++ >= MAX_TRIES)
			{
				i2c_state = I2C_IDLE;
			}
			else
			{
				// Transmit Start Condition
				i2c_transmit(I2C_START);
				i2c_state++;
			}
			break;
		}
		
		/////////////////////////////  READ
		case I2C_READ_START_END:
		{
			// Check the TWI Status
			if (twi_status == TW_MT_ARB_LOST)
			{
				i2c_state = I2C_READ_RETRY;
			}
			else if ((twi_status != TW_START) && (twi_status != TW_REP_START))
			{
				i2c_state = I2C_READ_STOP;
			}
			else
			{
				// Send slave address (SLA_W)
				TWDR = ((i2c_dev_addr & 0x0E) << 1) | TW_WRITE;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}
		case I2C_READ_SLAVE_ADDR_END:
		{
			// Check the TWSR status
			if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST))
			{
				i2c_state = I2C_READ_RETRY;
			}
			else if (twi_status != TW_MT_SLA_ACK)
			{
				i2c_state = I2C_READ_STOP;
			}
			else
			{
				// Send the Low 8-bit of I2C command
				TWDR = i2c_dev_comm & 0xFF;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				if (1 == i2c_dev_comm_len)
				{
					i2c_state = I2C_READ_SLAVE_COMM2_WAIT;
				}
				else
				{
					i2c_state = I2C_READ_SLAVE_COMM1_WAIT;
				}
				break;
			}
		}
		case I2C_READ_SLAVE_COMM1_END:
		{
			// Check the TWSR status
			if (twi_status != TW_MT_DATA_ACK)
			{
				i2c_state = I2C_READ_STOP;
			}
			else
			{
				// Send the High 8-bit of I2C command
				TWDR = i2c_dev_comm >> 8;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}
		case I2C_READ_SLAVE_COMM2_END:
		{
			// Check the TWSR status
			if (twi_status != TW_MT_DATA_ACK)
			{
				i2c_state = I2C_READ_STOP;
			}
			else
			{
				// Send start Condition
				i2c_transmit(I2C_START);
				i2c_state++;
				break;
			}
		}
		case I2C_READ_START2_END:
		{
			// Check the TWI Status
			if (twi_status == TW_MT_ARB_LOST)
			{
				i2c_state = I2C_READ_RETRY;
			}
			else if ((twi_status != TW_START) && (twi_status != TW_REP_START))
			{
				i2c_state = I2C_READ_STOP;
			}
			else
			{
				// Send slave address (SLA_W)
				TWDR = ((i2c_dev_addr & 0x0E) << 1) | TW_READ;
				// Transmit I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}
		case I2C_READ_SLAVE_ADDR2_END:
		{
			// Check the TWSR status
			if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST))
			{
				i2c_state = I2C_READ_RETRY;
			}
			else if (twi_status != TW_MT_SLA_ACK)
			{
				i2c_state = I2C_READ_STOP;
			}
			else
			{
			    // Read I2C Data
				i2c_transmit(I2C_DATA);
				i2c_state++;
				break;
			}
		}		
		case I2C_READ_DATA_END:
		{
			if (twi_status != TW_MT_DATA_ACK)
			{
				i2c_success = 0;
			}
			else
			{
				i2c_success = 1;
				i2c_data = TWDR;
			}
			i2c_state = I2C_READ_STOP;
		}
		case I2C_READ_STOP:
		{
			i2c_transmit(I2C_STOP);
			i2c_state++;
			break;
		}
		case I2C_READ_STOP_END:
		{
			i2c_state = I2C_IDLE;			
			break;
		}
		case I2C_READ_RETRY:
		{
			if (i2c_retries++ >= MAX_TRIES)
			{
				i2c_state = I2C_IDLE;
			}
			else
			{
				// Transmit Start Condition
				i2c_transmit(I2C_START);
				i2c_state++;
			}
			break;
		}
		default:
			break;
	}
}

void i2c_setup(void)
{
	// TWI initialization
	// Bit Rate: 98,765 kHz
	TWBR=0x49;
	// Two Wire Bus Slave Address: 0x0
	// General Call Recognition: Off
	TWAR=0x00;
	// Generate Acknowledge Pulse: Off
	// TWI Interrupt: On
	TWCR=0x05;
	TWSR=0x00;
	//PRR0 &= ~(1 << PRTWI);
}
