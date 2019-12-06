// TouchSampling.h

#ifndef __TOUCHSAMPLING_H_
#define __TOUCHSAMPLING_H_

#include "Arduino.h"

static const uint8_t pin2tsi[] = {
	//0    1    2    3    4    5    6    7    8    9
	9, 10, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 13, 0, 6, 8, 7,
	255, 255, 14, 15, 255, 12, 255, 255, 255, 255,
	255, 255, 11, 5
};

/* Customized for SEED ROBOTICS */
// Lower current, higher number of scans, and higher prescaler
// increase sensitivity, but the trade-off is longer measurement
// time and decreased range.
#define CURRENT   3 // 0 to 15 - current to use, value is 2*(current+1)
#define NSCAN     9 // number of times to scan, 0 to 31, value is nscan+1
#define PRESCALE  5 // prescaler, 0 to 7 - value is 2^(prescaler+1)

#define SCAN_MODULUS 10 /* interval between samplings in milliseconds */

/* Filter tuning */
#define SHORT_AVG_MUL 2
#define SHORT_AVG_DIV 3

#define DEEP_AVG_MUL 9999
#define DEEP_AVG_DIV 10000

enum en_TouchSamplingState {
	tss_TouchChannel1,
	tss_TouchChannel2
};

int tsi_user_initiated_scan(byte channel);
void tsi_init(boolean);
void tsi_stop();

#endif

