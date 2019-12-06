#include "Arduino.h"
#include "TouchSampling.h"

short tsi_error_cnt = 0;

int short_avg[4];
int deep_avg[4];

/*void tsi0_isr(void){

	// handle errors
	if (TSI0_GENCS & TSI_GENCS_OVRF) {
		TSI0_GENCS |= TSI_GENCS_OVRF;
		tsi_error_cnt++;
		tsi_init(false);
		return;
	}

	// clear end of Scan flag 
	TSI0_GENCS |= TSI_GENCS_EOSF;
	int read1, read2;

	read1 = max(0, *TSI0_CNTR_ch1); // reject negative readings
	short_avg[0] = (read1 + short_avg[0] * SHORT_AVG_MUL) / SHORT_AVG_DIV;
	deep_avg[0] = (read1 + deep_avg[0] * DEEP_AVG_MUL) / DEEP_AVG_DIV;

	

	main_board->i_capacitive1_reading = (short)(short_avg1 - deep_avg1);
	main_board->i_capacitive2_reading = (short)(short_avg2 - deep_avg2);
}
*/
void tsi_init(boolean reset_zeros) {
	byte ch1 = pin2tsi[A8];
	byte ch2 = pin2tsi[A9];

	if (ch1 == 255 || ch2 == 255) {
		Serial.printf("Invalid pin assignment");
	}

	*portConfigRegister(A9) = PORT_PCR_MUX(0);
	*portConfigRegister(A8) = PORT_PCR_MUX(0);

	SIM_SCGC5 |= SIM_SCGC5_TSI;

	/* Configure TSI Module */
	TSI0_GENCS &= ~TSI_GENCS_TSIEN; /* turn off TSI module */
	//TSI0_SCANC = TSI_SCANC_REFCHRG(3) | TSI_SCANC_EXTCHRG(CURRENT); /* scan settings: REFCHRG(3)=8microAmps; */
	//TSI0_GENCS = TSI_GENCS_NSCN(NSCAN) | TSI_GENCS_PS(PRESCALE) | TSI_GENCS_TSIEN;

	/* Teensy LC: REFCHRG(4) = 8microAmps */
	TSI0_GENCS = TSI_GENCS_REFCHRG(4) | TSI_GENCS_EXTCHRG(CURRENT) | TSI_GENCS_PS(PRESCALE) | TSI_GENCS_NSCN(NSCAN) | TSI_GENCS_TSIEN /*| TSI_GENCS_EOSF*/;

	if (reset_zeros) {
		delay(20);
		/* Calibrate the base reading (zero) */

		/* first scan to initalize filters */		
		int read1;

		read1 = max(0, tsi_user_initiated_scan(ch1)); /* reject negative readings */
		short_avg[0] = read1;
		deep_avg[0] = read1;


		/* filter sequential scans to get a more stable zero value */
		for (byte b = 0; b < 10; b++) {
			read1 = max(0, tsi_user_initiated_scan(ch1)); /* reject negative readings */
			short_avg[0] = (read1 + short_avg[0] * SHORT_AVG_MUL) / SHORT_AVG_DIV;
			deep_avg[0] = (read1 + deep_avg[0] * DEEP_AVG_MUL) / DEEP_AVG_DIV;			
		}		
	}

	//TSI0_GENCS &= ~TSI_GENCS_TSIEN; /* Disable TSI module */

	/* re configure TSI module for continuous scan */
	////T3.2 TSI0_SCANC |= TSI_SCANC_SMOD(SCAN_MODULUS);//TSI_SCANC_REFCHRG(3) | TSI_SCANC_EXTCHRG(2) | TSI_SCANC_SMOD(100) | TSI_SCANC_AMCLKS(0) | TSI_SCANC_AMPSC(0);
	////T3.2 TSI0_GENCS |= TSI_GENCS_STM | TSI_GENCS_ESOR | TSI_GENCS_TSIIE | TSI_GENCS_TSIEN;//TSI_GENCS_NSCN(15) | TSI_GENCS_PS(4) | TSI_GENCS_STM | TSI_GENCS_ESOR | TSI_GENCS_TSIIE | TSI_GENCS_ERIE | TSI_GENCS_TSIEN;

	/* STM = hardware trigger scan, ESOR=End of Scan interrupt selection, TSIIEN=interrutpt enable, TSIEN = TSI enable */
	//TSI0_GENCS |= TSI_GENCS_STM | TSI_GENCS_ESOR | TSI_GENCS_TSIIEN | TSI_GENCS_TSIEN;//TSI_GENCS_NSCN(15) | TSI_GENCS_PS(4) | TSI_GENCS_STM | TSI_GENCS_ESOR | TSI_GENCS_TSIIE | TSI_GENCS_ERIE | TSI_GENCS_TSIEN;

	//NVIC_SET_PRIORITY(IRQ_TSI, 128);
	//NVIC_ENABLE_IRQ(IRQ_TSI);
}

int tsi_user_initiated_scan(byte channel) {
	TSI0_DATA = TSI_DATA_TSICH(channel) | TSI_DATA_SWTS;
	delayMicroseconds(10);
	while (TSI0_GENCS & TSI_GENCS_SCNIP); // wait
	delayMicroseconds(1);
	return TSI0_DATA & 0xFFFF;
}

void tsi_stop() {
	NVIC_DISABLE_IRQ(IRQ_TSI);
}