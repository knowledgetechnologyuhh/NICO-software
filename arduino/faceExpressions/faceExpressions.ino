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
 - "caprt" to query the readings of capacitive touch channels in text format
 returns a sequence of numbers with the capacitive reading (as text;
 can be parsed with somethign like sscanf("%d %d %d %d"...) to
 convert to INTs (2 byte values);

 - "caprr" to query the readings of capacitive touch in raw format
 returns a sequence of bytes with the capacitive readings.
 see the command below for detailed documentation on the output format.

 - "capca" to force recallibration (re-zero'ing) of capacitive pad readings

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
 If do not want Touch support, #undefine COMPILE_TOUCH_SUPPORT
 If you attempt to compile Touch support and the board is not a Teensy LC
 it will also terminate with error.
 **/

 // For touch support
 // comment the define below to disable or compile without touch
#define COMPILE_TOUCH_SUPPORT

#include "touch/clsTouchTeensyLC.h"
#include <SoftwareSerial.h>

#include <SPI.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_8x8matrix left = Adafruit_8x8matrix();
Adafruit_8x8matrix right = Adafruit_8x8matrix();
Adafruit_8x16matrix mouth = Adafruit_8x16matrix();

#if (defined(COMPILE_TOUCH_SUPPORT) && !defined(__MKL26Z64__) )
#error Touch support can only be compiled for a Teensy LC processor board. Either set your board correctly or comment '#define TOUCH_SUPPORT' so that the sketch compiles without touch.
#else
clsTouch touch_pads;
#endif

void setup() {
  
  Serial.begin(9600);
  //Serial.println("NICO Face Expressions");
  //Serial.println("Type in a face expressions: ");

  int brightness=4;
  
  left.begin(0x70);  // pass in the address
  left.setBrightness(brightness); 
  
  right.begin(0x71);  // pass in the address
  right.setBrightness(brightness);

  mouth.begin(0x72);  // pass in the address

  mouth.setBrightness(brightness+4);

#ifdef COMPILE_TOUCH_SUPPORT
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
    B00000000},
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
    B00000000 },
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
    B00000000 },
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
    left.drawLine(x1,0, x2,8, LED_ON);
    left.writeDisplay();  // write the changes we just made to the display

    right.clear();
    right.drawLine(x2,0, x1,8, LED_ON);
    right.writeDisplay();  // write the changes we just made to the display  

}

void showMouthBitmap(const uint8_t *mouthM)
{
    mouth.clear();
    mouth.drawBitmap((int16_t) 0, (int16_t) 0, mouthM, 8, 16, LED_ON);
    mouth.writeDisplay();
  
}

void showMouthOpen(int size)
{
    mouth.clear();
    mouth.drawRoundRect(0, 3, size, size+5, 2, LED_ON);
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
    left.drawLine(7,0, 7,3, LED_ON);
    left.writeDisplay();  // write the changes we just made to the display

    right.clear();
    right.drawLine(7,4, 7,7, LED_ON);
    right.writeDisplay();  // write the changes we just made to the display  

    //lineBrows(3, 5);  
}

void neutral()
{
    showMouthBitmap(neutral_bmp); 
    lineBrows(4, 4);
}





void loop() {

#ifdef COMPILE_TOUCH_SUPPORT
	touch_pads.scanCapacitivePads();
#endif

    if (Serial.available()) {
      String str = Serial.readString();

      if (str == "happiness") {
          Serial.println("Showing happiness");
          happiness();

      } else if (str == "sadness") {
          Serial.println("Showing sadness");
          sadness();

      } else if (str == "anger") {
          Serial.println("Showing anger");
          anger();
  
      } else if (str == "neutral") {
          Serial.println("Showing neutral");
          neutral();

      } else if (str == "disgust") {
          Serial.println("Showing disgust");
          disgust();

      } else if (str == "surprise") {
          Serial.println("Showing surprise");
          surprise();
          
      } else if (str == "fear") {
          Serial.println("Showing fear");
          fear();

       } else if (str == "clear") {
          Serial.println("Clearing LCDs");

          mouth.clear();
          mouth.writeDisplay();

          left.clear();
          left.writeDisplay();
      
          right.clear();
          right.writeDisplay();



      } 
	   
#ifdef COMPILE_TOUCH_SUPPORT
	   else if (str == "caprt") { // capacitive readings exported as text. Separated by space, terminated by NEWLINE

		   int *readings = touch_pads.getTouchValues();
		   for (byte b = 0; b < (NR_CAPACITIVE_PADS - 1); b++) {
			   Serial.printf("%d ", readings[b]);
		   }
		   Serial.println(readings[NR_CAPACITIVE_PADS - 1]);
		   Serial.send_now(); // force USB stack to flush immediately. Function is available on Teensy boards

	   }
	   else if (str == "caprr") {  // capacitive readings exported as RAW. Provides a more efficient way to transfer data.
								   // Byte 0=Nr capacitive pads
								   // Byte 1=size of each reading (in nr of bytes). This to account for systems where an INT can be 2 or 4 bytes...
								   // the values in a sequence
								   // Last byte=checksum is calculated in the same way as Dynamixel 1: add all data bytes and then negate the result
		   int *readings = touch_pads.getTouchValues();

		   byte checksum = 0;
		   for (byte b = 0; b < (NR_CAPACITIVE_PADS * sizeof(int)); b++) {
			   checksum = (byte)(checksum + ((byte*)readings)[b]);
		   }
		   checksum = (byte)(~checksum);

		   Serial.write(NR_CAPACITIVE_PADS);
		   Serial.write(sizeof(int));
		   Serial.write((byte *)readings, NR_CAPACITIVE_PADS * sizeof(int));
		   Serial.write(checksum);
		   Serial.send_now(); // force USB stack to flush immediately. Function is available on Teensy boards
	   }
	   else if (str == "capca") { // capacitive sensor calibration: forces a re callibration of the sensors (note: sensors are automatically callibrated in initialization)
		   touch_pads.callibrateCapacitivePads();
		   Serial.println("Capacitive pads callibrated");
		   Serial.send_now(); // force USB stack to flush immediately. Function is available on Teensy boards
	   }
#endif	   
	   
	   else {
          Serial.println("Unknown command. Will not show anything");
      }
  }


}
