#include "TouchSampling.h"


extern int short_avg[4];
extern int deep_avg[4];

// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(9600);

	delay(3000);

	tsi_init(true);
}

// Add the main program code into the continuous loop() function
void loop()
{
	int read1 = max(0, tsi_user_initiated_scan(pin2tsi[A8])); /* reject negative readings */
	short_avg[0] = (read1 + short_avg[0] * SHORT_AVG_MUL) / SHORT_AVG_DIV;
	deep_avg[0] = (read1 + deep_avg[0] * DEEP_AVG_MUL) / DEEP_AVG_DIV;

	Serial.printf("Short %d, Deep %d, normalized %d\r\n", short_avg[0], deep_avg[0], short_avg[0] - deep_avg[0]);

	

}
