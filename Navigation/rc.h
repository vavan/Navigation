#ifndef RC_H
#define RC_H

/************************************************************************/
/* Number of RC channels  */
/************************************************************************/
#define RC_CHANNEL_COUNT 4

/************************************************************************/
/* Init RC (times, pin-change interrupts */
/************************************************************************/
void init_RC();

/************************************************************************/
/* RC values */
/************************************************************************/
extern volatile uint16_t rcData[8];//[RC_CHANNEL_COUNT];
extern int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
extern int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE


#endif  /* RC_H */