#ifndef CONTROL_H
#define CONTROL_H


/************************************************************************/
/* OUTPUT CONTROLS - MOTORS */
/************************************************************************/

enum EMOTOR {
	LMOTOR,
	RMOTOR
};

/************************************************************************/
/* Init registers, timer for PWM for motors */
/************************************************************************/
void init_motors();


/**
 value: -100..0..+100 -> 1000..1500..2000 ms
*/

/************************************************************************/
/* Set motor power. Range -100 (full backward) to +100 (full fwd) */
/************************************************************************/
void set_motor(int motor, int value);


/************************************************************************/
/* INPUT CONTROLS - RC  */
/************************************************************************/

#define RC_CHANNEL_COUNT 3

/************************************************************************/
/* Init RC (times, pin change interrupts */
/************************************************************************/
void init_RC();

/************************************************************************/
/* RC values */
/************************************************************************/
volatile byte rc_input[RC_CHANNEL_COUNT];


#endif /* CONTROL_H */