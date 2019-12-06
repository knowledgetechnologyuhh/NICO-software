/***************************************************
This is a library for our I2C LED Backpacks

Designed specifically to work with the Adafruit LED Matrix backpacks
----> http://www.adafruit.com/products/872
----> http://www.adafruit.com/products/871
----> http://www.adafruit.com/products/870

These displays use I2C to communicate, 2 pins are required to
interface. There are multiple selectable I2C addresses. For backpacks
with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
with 3 Address Select pins: 0x70 thru 0x77

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution
****************************************************/

/****
  4 Dec 2018 - Contribution by Seed Robotics (www.seedrobotics.com)
  under the original NICo license.

  Added commands: 
	- "capr" to query the readings of capacitive touch channels.
			returns a sequence of numbers with the capacitive reading (as text;
			needs to be parsed with somethign like sscanf("%d %d %d %d"...) to
			convert to INTs (2 byte values);
	- "capc" to force recallibration (re-zero'ing) of capacitive pad readings

  Capacitive touch channels are enabled via the custom Seed Robotics board
  which is based on the Teensy LC board.

  Pads are connected to processor pins 2, 3, 22 and 23.

  Compiled with Arduino 1.0.6 and Teensyduino 1.45.
  You need to download those to compile the sample with touch support.
  Newer versions of Arduno/Teensiduino /should/ also compile properly.

  CAPACITIVE SENSITIVITY: the Teensyduino library function touchRead does not 
  work properly with the NICo head pads. This is because the chip can be set up 
  for different sensitivity but touchRead sets it up to a fixed
  (lower) sensitivity.
  Furtermore touchRead is syncronous which blocks execution while read is in progres..
  Therefore, we use our own custom functions that set up the processor registers
  to the appropriate sensitivity and implement functions that allow asyncronous operation.
  
  --
  If do not want Touch support, #undefine TOUCH_SUPPORT
  If you attempt to compile Touch support and the board is not a Teensy LC
  it will also terminate with error.
**/

// For touch support
// comment the define below to disable or compile without touch
#define TOUCH_SUPPORT

#include <SPI.h>
#include "touch/clsTouchTeensyLC.h"
#include <SoftwareSerial.h>

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_8x8matrix left = Adafruit_8x8matrix();
Adafruit_8x8matrix right = Adafruit_8x8matrix();
Adafruit_8x16matrix mouth = Adafruit_8x16matrix();

// For neopixel Jewels
// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#if (defined(TOUCH_SUPPORT) && !defined(__MKL26Z64__) )
#error Touch support can only be compiled for a Teensy LC processor board. Either set your board correctly or comment '#define TOUCH_SUPPORT' so that the sketch compiles without touch.
#else
clsTouch touch_pads;
#endif


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define NEOPXL_PIN            6

// How many NeoPixels are attached to the Arduino?
#define NEOPXL_NUMPIXELS      14

// When we setup the NeoPixel library, we tell it how many neo_pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel neo_pixels = Adafruit_NeoPixel(NEOPXL_NUMPIXELS, NEOPXL_PIN, NEO_RGBW + NEO_KHZ800);


#define E_lowByte(w) ((uint8_t) ((w) & 0xff))
#define E_highByte(w) ((uint8_t) ((w) >> 8) & 0xff)
#define E_highestByte(w) ((uint8_t) ((w) >> 16) & 0xff)

void setup() {

	Serial.begin(115200);
	Serial.setTimeout(4);
	//Serial.println("NICO Face Expressions");
	//Serial.println("Type in a face expressions: ");

	delay(3000);

	int brightness = 11;

	left.begin(0x70);  // pass in the address
	left.setBrightness(brightness);

	right.begin(0x71);  // pass in the address
	right.setBrightness(brightness);

	mouth.begin(0x72);  // pass in the address

	mouth.setBrightness(brightness + 4);

	//happiness();

	// initialize the NeoPixel Jewels
	neo_pixels.begin(); // This initializes the NeoPixel library.

	for (int i = 0; i < NEOPXL_NUMPIXELS; i++) {

		// neo_pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
		neo_pixels.setPixelColor(i, neo_pixels.Color(0, 0, 0)); // Moderately bright green color.

		neo_pixels.show(); // This sends the updated pixel color to the hardware.

		delay(5); // Delay for a period of time (in milliseconds).

	}

#ifdef TOUCH_SUPPORT
	touch_pads.init();
#endif
}

static const uint8_t PROGMEM
up_bmp[] =
{ B00000000,
B00000000,
B00001000,
B00010000,
B00100000,
B01000000,
B01000000,
B01000000,
B01000000,
B01000000,
B01000000,
B00100000,
B00010000,
B00001000,
B00000000,
B00000000
},
neutral_bmp[] =
{ B00000000,
B00000000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00000000,
B00000000
},
down_bmp[] =
{ B00000000,
B00000000,
B10000000,
B01000000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B00100000,
B01000000,
B10000000,
B00000000,
B00000000
},
open_bmp[] =
{ B00000000,
B00000000,
B00000000,
B11111000,
B10001000,
B10001000,
B10001000,

B10001000,
B10001000,
B10001000,
B10001000,
B11111000,
B00000000,
B00000000,
B00000000
},
small_open_bmp[] =
{ B00000000,
B00000000,
B00000000,
B00000000,
B11111000,
B10001000,
B10001000,
B10001000,
B10001000,
B10001000,
B11111000,
B00000000,
B00000000,
B00000000,
B00000000
},
wide_open_bmp[] =
{ B00000000,
B11111100,
B10000100,
B10000100,
B10000100,

B10000100,
B10000100,
B10000100,
B10000100,
B10000100,
B10000100,
B10000100,
B10000100,
B11111100,
B00000000
};

void lineBrows(int x1, int x2) {

	left.clear();
	left.drawLine(x1, 0, x2, 8, LED_ON);
	left.writeDisplay();  // write the changes we just made to the display

	right.clear();
	right.drawLine(x2, 0, x1, 8, LED_ON);
	right.writeDisplay();  // write the changes we just made to the display

}



void showMouthBitmap(const uint8_t *mouthM)
{
	//Serial.print("in mouth bitmap");
	mouth.clear();
	// for (int ze=0; ze<16; ze++)
	//      {
	//        Serial.print(ze);
	//        Serial.print(" ");
	//        Serial.print(mouthM[ze],HEX);
	//        Serial.println();
	//      }
	mouth.drawBitmap((int16_t)0, (int16_t)0, mouthM, 8, 16, LED_ON);

	mouth.writeDisplay();

}

// Show mouth display from non constant PROGMEM array
// The library from adafruit did overload the function
// resulting in using the wrong function if converting this to a const .......
// Erik
void showMouthBitmap_nC(uint8_t *mouthM)
{
	//Serial.print("in mouth bitmap");
	//mouth.clear();
	// for (int ze=0; ze<16; ze++)
	//      {
	//        Serial.print(ze);
	//        Serial.print(" ");
	//        Serial.print(mouthM[ze],HEX);
	//        Serial.println();
	//      }
	mouth.drawBitmap((int16_t)0, (int16_t)0, mouthM, 8, 16, LED_ON);
	mouth.writeDisplay();

}

void showMouthOpen(int size)
{
	mouth.clear();
	mouth.drawRoundRect(0, 3, size, size + 5, 2, LED_ON);
	mouth.writeDisplay();
}

void happiness()
{
	showMouthBitmap(up_bmp);
	lineBrows(3, 4);
}

void sadness()
{
	showMouthBitmap(down_bmp);
	lineBrows(2, 5);
}

void anger()
{
	showMouthOpen(6);
	lineBrows(7, 1);
}

void fear()
{
	//mouth.clear();
	//showMouthBitmap(small_open_bmp);
	//mouth.drawRoundRect(0, 4, 6, 10, 2, LED_ON);
	//mouth.writeDisplay();

	showMouthBitmap(wide_open_bmp);

	lineBrows(1, 3);

	//left.clear();
	//left.drawCircle(0, 4, 7, LED_ON);
	//left.writeDisplay();  // write the changes we just made to the display

	//right.clear();
	//right.drawCircle(0, 4, 7, LED_ON);
	//right.writeDisplay();  // write the changes we just made to the display
}

void surprise()
{
	mouth.clear();
	mouth.drawRoundRect(0, 4, 6, 10, 2, LED_ON);
	mouth.writeDisplay();

	left.clear();
	left.drawCircle(8, 4, 5, LED_ON);
	left.writeDisplay();  // write the changes we just made to the display

	right.clear();
	right.drawCircle(8, 4, 5, LED_ON);
	right.writeDisplay();  // write the changes we just made to the display
}

void disgust()
{
	showMouthBitmap(open_bmp);

	//mouth.clear();
	//mouth.drawLine(0,6, 0,9, LED_ON);
	//mouth.writeDisplay();  // write the changes we just made to the display

	left.clear();
	left.drawLine(7, 0, 7, 3, LED_ON);
	left.writeDisplay();  // write the changes we just made to the display

	right.clear();
	right.drawLine(7, 4, 7, 7, LED_ON);
	right.writeDisplay();  // write the changes we just made to the display

						   //lineBrows(3, 5);
}

void neutral()
{
	showMouthBitmap(neutral_bmp);
	lineBrows(4, 4);
}

// Converting HEX-String into long
// from pico in arduino forum
// https://forum.arduino.cc/index.php?topic=123486.0

long x2l(char *s)
{
	long x = 0;
	for (;;) {
		char c = *s;
		if (c >= '0' && c <= '9') {
			x *= 16;
			x += c - '0';
		}
		else if (c >= 'A' && c <= 'F') {
			x *= 16;
			x += (c - 'A') + 10;
		}
		else break;
		s++;
	}
	return x;
}




void loop() {
	int* scan_results;
	static unsigned long last_print = 0;

	// protocol-mode
	// 2 - Full data communication between Arduino and controller
	// 1 - Reduced communication, only handshake
	// 0 - Only one way communication
	int prot_mode;
	prot_mode = 2;

	touch_pads.scanCapacitivePads();

	if (millis() - last_print > 1000) {
		scan_results = touch_pads.getTouchValues();
		Serial.printf("Touch values: %d %d %d %d\n", scan_results[0], scan_results[1], scan_results[2], scan_results[3]);
		last_print = millis();
	}
	
	if (Serial.available()) {
		String str = Serial.readString();

		if (str == "happiness") {
			Serial.println("Showing happiness");
			happiness();

		}
		else if (str == "sadness") {
			Serial.println("Showing sadness");
			sadness();

		}
		else if (str == "anger") {
			Serial.println("Showing anger");
			anger();

		}
		else if (str == "neutral") {
			Serial.println("Showing neutral");
			neutral();

		}
		else if (str == "disgust") {
			Serial.println("Showing disgust");
			disgust();

		}
		else if (str == "surprise") {
			Serial.println("Showing surprise");
			surprise();

		}
		else if (str == "fear") {
			Serial.println("Showing fear");
			fear();

		}
		else if (str == "clear") {
			Serial.println("Clearing LCDs");

			mouth.clear();
			mouth.writeDisplay();

			left.clear();
			left.writeDisplay();

			right.clear();
			right.writeDisplay();

			// Chage communication mode,
			// the lower, the faster  
		}
		else if (str.substring(0, 3) == "mod") {

			int new_mode = str.substring(3, 4)[0];
			prot_mode = new_mode;
			Serial.print("Change Comm mode to  ");
			Serial.print(str.substring(3, 4));
			Serial.println(" ");


		}
		else if (str.substring(0, 3) == "che") {
			String co = str.substring(3, NEOPXL_NUMPIXELS * 6 + 3);

			for (int i = 0; i < NEOPXL_NUMPIXELS; i++) {

				String oneCo = co.substring(i * 6, i * 6 + 6);

				//Serial.println(oneCo);

				char charBuf[50];
				oneCo.toCharArray(charBuf, 50);

				long pix = x2l(charBuf);

				Serial.println(pix, HEX);

				// neo_pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
				neo_pixels.setPixelColor(i, neo_pixels.Color(E_highByte(pix), E_highestByte(pix), E_lowByte(pix)));

				neo_pixels.show(); // This sends the updated pixel color to the hardware.

				delay(5); // Delay for a period of time (in milliseconds).

			}



		}
		else if (str.substring(0, 3) == "raw") {

			String display = str.substring(3, 4);

			//Serial.println("Display");

			//Serial.println(display);

			int numcol = 0;
			// if mouth display
			if (display == "m") {
				numcol = 16;
			}
			// else eye display
			else {
				numcol = 8;
			}

			String co = str.substring(4, numcol * 2 + 4);

			//Write Dummy pattern in Matrix to recognize it in case of error
			uint8_t Amatrix[] = { B00000000,
				B11111100,
				B10000100,
				B10000100,
				B10000100,

				B10000100,
				B10000100,
				B10000100,
				B10000100,
				B10000100,
				B10000100,
				B10000100,
				B10000100,
				B11111100,
				B00000000
			};


			for (int i = 0; i < numcol * 2; i += 2) {

				String oneCo = co.substring(i, i + 2);

				char charBuf[8];

				oneCo.toCharArray(charBuf, 10);

				//Serial.print("char by ");
				//Serial.print(i);
				//Serial.print(" ");
				//Serial.println(charBuf);


				long pix = x2l(charBuf);

				//Serial.print(pix, HEX);

				// Set this on the Array
				Amatrix[i / 2] = (uint8_t)pix;
				//Amatrix[i] = B00000000;


			}
			if (display == "m") {
				//happiness();
				mouth.clear();
				if (prot_mode>1) Serial.print("Displaying on mouth:");

				for (int ze = 0; ze<numcol; ze++)
				{
					if (prot_mode>1) Serial.print(ze);
					if (prot_mode>1) Serial.print(" ");
					if (prot_mode>1) Serial.print(Amatrix[ze], HEX);

				}
				if (prot_mode>0) Serial.println(" ");
				showMouthBitmap_nC(Amatrix);
			}

			if ((display == "r") || (display == "l")) {
				//happiness();
				if (prot_mode>1) Serial.print("Displaying on eyebrow ");
				if (prot_mode>1) Serial.print(display);
				if (prot_mode>1) Serial.print(" ");

				for (int ze = 0; ze<numcol; ze++)
				{
					if (prot_mode>1) Serial.print(ze);
					if (prot_mode>1) Serial.print(" ");
					if (prot_mode>1) Serial.print(Amatrix[ze], HEX);

				}
				if (prot_mode>0) Serial.println(" ");
				if (display == "r")
				{
					right.clear();
					right.drawBitmap((int16_t)0, (int16_t)0, Amatrix, 8, 8, LED_ON);

					right.writeDisplay();
				}
				else
				{
					left.clear();
					left.drawBitmap((int16_t)0, (int16_t)0, Amatrix, 8, 8, LED_ON);

					left.writeDisplay();
				}

				//showMouthBitmap_nC(Amatrix);
			}


		}

		else {
			Serial.println("Unknown command. Will not show anything");
			Serial.println(str);
		}
	}


}
