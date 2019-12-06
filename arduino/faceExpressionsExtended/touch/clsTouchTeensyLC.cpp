#include "touch/clsTouchTeensyLC.h"

clsTouch::clsTouch() {
	output_readings = (int*)calloc(NR_CAPACITIVE_PADS, sizeof(int));
	short_avg_readings = (int*)calloc(NR_CAPACITIVE_PADS, sizeof(int));
	deep_avg_readings = (int*)calloc(NR_CAPACITIVE_PADS, sizeof(int));
	tsi_channel = (byte*)calloc(NR_CAPACITIVE_PADS, sizeof(byte));
}

void clsTouch::init() {
	tsi_channel[0] = pin2tsi[CAP_PAD0_PIN];
	tsi_channel[1] = pin2tsi[CAP_PAD1_PIN];
	tsi_channel[2] = pin2tsi[CAP_PAD2_PIN];
	tsi_channel[3] = pin2tsi[CAP_PAD3_PIN];

	for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
		if (tsi_channel[b] == 255) {
			Serial.printf("Invalid channel for pad %d\n", b);
			return;
		}
	}

	*portConfigRegister(CAP_PAD0_PIN) = PORT_PCR_MUX(0);
	*portConfigRegister(CAP_PAD1_PIN) = PORT_PCR_MUX(0);
	*portConfigRegister(CAP_PAD2_PIN) = PORT_PCR_MUX(0);
	*portConfigRegister(CAP_PAD3_PIN) = PORT_PCR_MUX(0);

	SIM_SCGC5 |= SIM_SCGC5_TSI; // enable clock for TSI module (crashes if we don't do this first)

	/* Configure Module
	   To configure the module w emust turn it off first
	*/
	TSI0_GENCS &= ~TSI_GENCS_TSIEN; // module off

	/* Teensy LC: REFCHRG(4) = 8microAmps */
	TSI0_GENCS = TSI_GENCS_REFCHRG(REFCHRG) | TSI_GENCS_EXTCHRG(EXTCHRG) | TSI_GENCS_PS(PRESCALE) | TSI_GENCS_NSCN(NRSCANS) | TSI_GENCS_TSIEN /*| TSI_GENCS_EOSF*/;
	delay(20);

	callibrateCapacitivePads();

	initialized = true;
}

void clsTouch::callibrateCapacitivePads() {
	if (!initialized) return;
	
	int tsi_reading;
	/* first scan to initalize filters */
	for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
		tsi_reading = max(0, tsi_scan_sync(tsi_channel[b])); /* reject negative readings */
		short_avg_readings[b] = tsi_reading;
		deep_avg_readings[b] = tsi_reading;
	}

	/* take sequential scans to get a more stable baseline value for filtering */
	for (byte c = 0; c < 10; c++) {
		for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
			tsi_reading = max(0, tsi_scan_sync(tsi_channel[b])); /* reject negative readings */
			short_avg_readings[b] = (tsi_reading + short_avg_readings[b] * TSI_SHORT_AVG_MUL) / TSI_SHORT_AVG_DIV;
			deep_avg_readings[b] = (tsi_reading + deep_avg_readings[b] * TSI_DEEP_AVG_MUL) / TSI_DEEP_AVG_DIV;
		}
	}
}

int* clsTouch::getTouchValues() {
	if (!initialized) return output_readings;

	for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
		output_readings[b] = max(0, deep_avg_readings[b] - short_avg_readings[b]);
	}
	return output_readings;
}


void clsTouch::scanCapacitivePads() {
	if (!initialized) return;

	if (current_pad_scanning == NO_PAD_SCANNING) {
		tsi_init_scan_async(0);
		current_pad_scanning = 0;
	}
	else if (tsi_is_scan_complete()) {
		int tsi_reading = max(0, tsi_get_scan_result());
		short_avg_readings[current_pad_scanning] = (tsi_reading + short_avg_readings[current_pad_scanning] * TSI_SHORT_AVG_MUL) / TSI_SHORT_AVG_DIV;
		deep_avg_readings[current_pad_scanning] = (tsi_reading + deep_avg_readings[current_pad_scanning] * TSI_DEEP_AVG_MUL) / TSI_DEEP_AVG_DIV;

		current_pad_scanning = ((current_pad_scanning + 1) % NR_CAPACITIVE_PADS);
		tsi_init_scan_async(current_pad_scanning);
	} /* else {
	  do nothing; wait for the scan to complete and check for that on the next function call
	}  */
}

int clsTouch::tsi_scan_sync(byte channel) {
	TSI0_DATA = TSI_DATA_TSICH(channel) | TSI_DATA_SWTS;
	delayMicroseconds(10);
	while (TSI0_GENCS & TSI_GENCS_SCNIP); // wait
	delayMicroseconds(1);
	return TSI0_DATA & 0xFFFF;
}

void clsTouch::tsi_init_scan_async(byte channel) {
	TSI0_DATA = TSI_DATA_TSICH(channel) | TSI_DATA_SWTS;
	delayMicroseconds(10);	
}

inline bool clsTouch::tsi_is_scan_complete() {
	return ( (TSI0_GENCS & TSI_GENCS_SCNIP) > 0 );
}

int clsTouch::tsi_get_scan_result() {
	if (TSI0_GENCS & TSI_GENCS_SCNIP) return -1; // scan is still in progress
	return TSI0_DATA & 0xFFFF;
}






