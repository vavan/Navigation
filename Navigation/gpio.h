#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <avr/io.h>

// BGPIO configuration.
#define		BGPIO_VALUE		PORTB
#define		BGPIO_DIR		DDRB
#define		BGPIO_GET		PINB

// CGPIO configuration.
#define		CGPIO_VALUE		PORTC
#define		CGPIO_DIR		DDRC
#define		CGPIO_GET		PINC

// DGPIO configuration.
#define		DGPIO_VALUE		PORTD
#define		DGPIO_DIR		DDRD
#define		DGPIO_GET		PIND

// EGPIO configuration.
#define		EGPIO_VALUE		PORTE
#define		EGPIO_DIR		DDRE
#define		EGPIO_GET		PINE

#define		_GPIO_MASK(bit)	(1<<bit)
#define		_GPIO_MASKIO(port, bit)	_GPIO_MASK(bit)

// Internal port composing macros
#define _GPIO_PORT_VALUE(port)		port##GPIO_VALUE
#define _GPIO_PORT_DIR(port)		port##GPIO_DIR
#define _GPIO_PORT_GET(port)		port##GPIO_GET


// Value Read/Write macroses
#define _GPIO_Set(port, mask)		(_GPIO_PORT_VALUE(port) |= _GPIO_MASK(mask))
#define _GPIO_Clr(port, mask)		(_GPIO_PORT_VALUE(port) &= ~_GPIO_MASK(mask))
#define _GPIO_Get(port, mask)		(_GPIO_PORT_GET(port) & _GPIO_MASK(mask))

// Direction In/Out macroses
#define _GPIO_DirOut(port, mask)	(_GPIO_PORT_DIR(port) |= _GPIO_MASK(mask))
#define _GPIO_DirIn(port, mask)		(_GPIO_PORT_DIR(port) &= ~_GPIO_MASK(mask))

///////////////////////////////////////////////////////////////////////////////
//
//	Public API
//
///////////////////////////////////////////////////////////////////////////////
#define	GPIO_Mask(io)       _GPIO_MASKIO(io)
#define GPIO_Set(io)		_GPIO_Set(io)
#define GPIO_Clr(io)		_GPIO_Clr(io)
#define GPIO_Get(io)		_GPIO_Get(io)
#define GPIO_DirOut(io)		_GPIO_DirOut(io)
#define GPIO_DirIn(io)		_GPIO_DirIn(io)


#endif // GPIO_DRIVER_H

