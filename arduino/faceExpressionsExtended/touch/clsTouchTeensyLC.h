/****
4 Dec 2018 - Contribution by Seed Robotics (www.seedrobotics.com)
under the original NICo license.

Added command "cap" to query the readings of capacitive touch channels.
Capacitive touch channels are enabled via the custom Seed Robotics board
which is based on the Teensy LC board.

Pads are installed on pins X, Y Z, A

CAPACITIVE SENSITIVITY: the Teensyduino library function touchRead does not
work properly with the NICo head pads. This is because the chip can be set up
for different sensitivity but touchRead sets it up to a fixed
(lower) sensitivity.
Therefore, we use our own custom functions that set up the processor registers
to the appropriate sensitivity.

--
If do not want Touch support, #undefine TOUCH_SUPPORT
If you attempt to compike Touch support nad the board is not a Teensy LC
it will also terminate with error.

**/


#ifndef _TOUCHTEENSYLC_h
#define _TOUCHTEENSYLC_h

#include "arduino.h"

#define NR_CAPACITIVE_PADS 4

#define CAP_PAD0_PIN 3
#define CAP_PAD1_PIN 4 
#define CAP_PAD2_PIN 22
#define CAP_PAD3_PIN 23

/* Parameters to tune the TSI module sensisitivity and precision
   If you need to change this, do it carefuly.
   The meaning of each parameter can be found on the processor datasheet (Teensy LC)
   section covering register TSIx_GENCS 
   
   Generally, Lower current, higher number of scans, and higher prescaler
   increase sensitivity, but the trade-off is longer measurement
   time and decreased range.   
   */

#define REFCHRG	4 /* T32=4 (8 microamps) */
#define EXTCHRG	3 /* T32=3 0 to 15 - current to use, value is 2*(current+1) */
#define PRESCALE 2 /* T32=5 prescaler, 0 to 7 - value is 2^(prescaler+1) */
#define NRSCANS 9 /* T32=9  number of times to scan/sample, 0 to 31, value is nscan+1 */


/* Parameters for the software filters on the readings
   We use two averages: one very heavy (deep) average that gives us the baseline (zero) value
   One short average that gives us the current reading
   
   Subtracting them, should give you a cleaner value

   The downside is that, if the pads are touched for a very long time, the deep average
   aproximates the short average.
   You may tweak the depth of the heavy average of call callibrateCapacitivePads()
   if you need a re callibration
*/

#define TSI_SHORT_AVG_MUL 0
#define TSI_SHORT_AVG_DIV 1

#define TSI_DEEP_AVG_MUL 999
#define TSI_DEEP_AVG_DIV 1000

#define TSI_DELAY_BETWEEN_SCANS_MILLIS 100

#define NO_PAD_SCANNING 0xFF

/* Correspondence between Pin Nr and capacitive channel for Teensy LC */
static const uint8_t TLC_pin2tsi[] = {
	//0    1    2    3    4    5    6    7    8    9
	9,  10, 255,   2,   3, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255,  13,   0,   6,   8,   7,
	255, 255,  14,  15, 255, 255, 255
};

class clsTouch {
public:
	clsTouch();
	void init();
	void callibrateCapacitivePads();
	void scanCapacitivePads();
	int* getTouchValues();
	unsigned long getNrScansPerformed() { return nr_scans_performed_;  }

	byte current_pad_scanning_ = NO_PAD_SCANNING;

private:
	int tsi_scan_sync(byte ch);
	void tsi_init_scan_async(byte ch);
	bool tsi_is_scan_complete();
	int tsi_get_scan_result();

	int* output_readings_; // used to output readings to user space program
	int* short_avg_readings_;
	int* deep_avg_readings_;

	byte* tsi_channel_;

	bool tsi_module_enabled_ = false;

	//byte current_pad_scanning_ = NO_PAD_SCANNING;
	unsigned long nr_scans_performed_ = 0;
	byte tsi_async_scan_in_progress_ = false;
};

#endif

