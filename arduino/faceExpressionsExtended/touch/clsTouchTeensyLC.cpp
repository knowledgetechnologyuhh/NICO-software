#include "touch/clsTouchTeensyLC.h"

clsTouch::clsTouch() {
	output_readings_ = (int*)calloc(NR_CAPACITIVE_PADS, sizeof(int));
	short_avg_readings_ = (int*)calloc(NR_CAPACITIVE_PADS, sizeof(int));
	deep_avg_readings_ = (int*)calloc(NR_CAPACITIVE_PADS, sizeof(int));
	tsi_channel_ = (byte*)calloc(NR_CAPACITIVE_PADS, sizeof(byte));
}

#ifdef __MKL26Z64__
void clsTouch::init() {
	tsi_channel_[0] = TLC_pin2tsi[CAP_PAD0_PIN];
	tsi_channel_[1] = TLC_pin2tsi[CAP_PAD1_PIN];
	tsi_channel_[2] = TLC_pin2tsi[CAP_PAD2_PIN];
	tsi_channel_[3] = TLC_pin2tsi[CAP_PAD3_PIN];

	for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
		if (tsi_channel_[b] == 255) {
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

	tsi_module_enabled_ = true;

	callibrateCapacitivePads();
}

void clsTouch::callibrateCapacitivePads() {
	if (!tsi_module_enabled_) return;
	
	int tsi_reading;
	/* first scan to initalize filters */
	for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
		tsi_reading = tsi_scan_sync(tsi_channel_[b]);
		short_avg_readings_[b] = tsi_reading;
		deep_avg_readings_[b] = tsi_reading;
	}

	/* take sequential scans to get a more stable baseline value for filtering */
	for (byte c = 0; c < 10; c++) {
		for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
			tsi_reading = tsi_scan_sync(tsi_channel_[b]);
			short_avg_readings_[b] = (tsi_reading + short_avg_readings_[b] * TSI_SHORT_AVG_MUL) / TSI_SHORT_AVG_DIV;
			deep_avg_readings_[b] = (tsi_reading + deep_avg_readings_[b] * TSI_DEEP_AVG_MUL) / TSI_DEEP_AVG_DIV;
		}
	}
}

void clsTouch::scanCapacitivePads() {
	if (!tsi_module_enabled_) return;

	if (current_pad_scanning_ == NO_PAD_SCANNING) {
		tsi_init_scan_async(0);
		current_pad_scanning_ = 0;
	}
	else if (tsi_is_scan_complete()) {
		//delay(1);
		int tsi_reading = tsi_get_scan_result();
		short_avg_readings_[current_pad_scanning_] = (tsi_reading + short_avg_readings_[current_pad_scanning_] * TSI_SHORT_AVG_MUL) / TSI_SHORT_AVG_DIV;
		deep_avg_readings_[current_pad_scanning_] = (tsi_reading + deep_avg_readings_[current_pad_scanning_] * TSI_DEEP_AVG_MUL) / TSI_DEEP_AVG_DIV;

		nr_scans_performed_++;

		current_pad_scanning_ = ((current_pad_scanning_ + 1) % NR_CAPACITIVE_PADS);
		tsi_init_scan_async(current_pad_scanning_);
	} /* else {
	  do nothing; wait for the scan to complete and check for that on the next function call
	  }  */
}

#else
#warning Attempting to Compile Touch functions but the selected board is not Teensy LC

void clsTouch::init() { Serial.println("Wrong board model for Touch"); initialized = false; }
void clsTouch::callibrateCapacitivePads() { ; }
void clsTouch::scanCapacitivePads() { ; }

#endif


int* clsTouch::getTouchValues() {
	if (!tsi_module_enabled_) return output_readings_;

	for (byte b = 0; b < NR_CAPACITIVE_PADS; b++) {
		output_readings_[b] = max(-65000, short_avg_readings_[b] - deep_avg_readings_[b]);
	}
	return output_readings_;
}

int clsTouch::tsi_scan_sync(byte channel) {
	TSI0_DATA = TSI_DATA_TSICH(channel) | TSI_DATA_SWTS;
	delayMicroseconds(10);
	while (TSI0_GENCS & TSI_GENCS_SCNIP); // wait for scan to complete
	delayMicroseconds(1);
	return TSI0_DATA & 0xFFFF;
}

void clsTouch::tsi_init_scan_async(byte channel) {
	TSI0_DATA = TSI_DATA_TSICH(channel) | TSI_DATA_SWTS;
	delayMicroseconds(10);	
}

inline bool clsTouch::tsi_is_scan_complete() {
	return ( (TSI0_GENCS & TSI_GENCS_SCNIP) == 0 );
}

int clsTouch::tsi_get_scan_result() {
	if (TSI0_GENCS & TSI_GENCS_SCNIP) return -1; // scan is still in progress
	return TSI0_DATA & 0xFFFF;
}






