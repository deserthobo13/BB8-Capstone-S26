#include <FatReader.h>
#include <SdReader.h>
#include "lightDefinitions.h"
#include "soundDefinitions.h"
#include "FastLED.h"

#include <avr/pgmspace.h>
#include <WaveHC.h>
#include <Wavemainpage.h>
#include <WavePinDefs.h>
#include <WaveUtil.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//SoftwareSerial mySerial(14, 15); // RX, TX

int incomingByte = 0;

uint8_t dirLevel; // indent level for file/dir names    (for prettyprinting)




void sdErrorCheck(void)
{
  if (!card.errorCode()) return;
  PgmPrint("\r\nSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  PgmPrint(", ");
  Serial.println(card.errorData(), HEX);
  while(1);
}

void setup() {
  FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
  Serial.begin(9600);
 // myserial.begin(9600);
  
 // putstring("Free RAM: ");       // This can help with debugging, running out of RAM is bad
 // putstring_nl(freeRam());      // if this is under 150 bytes it may spell trouble!
  
  // Set the output pins for the DAC control. This pins are defined in the library
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
 
  // pin13 LED
  pinMode(13, OUTPUT);

  //  if (!card.init(true)) { //play with 4 MHz spi if 8MHz isn't working for you
  if (!card.init()) {         //play with 8 MHz spi (default faster!)  
    putstring_nl("Card init. failed!");  // Something went wrong, lets print out why
    sdErrorCheck();
    while(1);                            // then 'halt' - do nothing!
  }
  
  // enable optimize read - some cards may timeout. Disable if you're having problems
  card.partialBlockRead(true);
 
// Now we will look for a FAT partition!
  uint8_t part;
  for (part = 0; part < 5; part++) {     // we have up to 5 slots to look in
    if (vol.init(card, part)) 
      break;                             // we found one, lets bail
  }
  if (part == 5) {                       // if we ended up not finding one  :(
    putstring_nl("No valid FAT partition!");
    sdErrorCheck();      // Something went wrong, lets print out why
    while(1);                            // then 'halt' - do nothing!
  }
  
  // Lets tell the user about what we found
  putstring("Using partition ");
  Serial.print(part, DEC);
  putstring(", type is FAT");
  Serial.println(vol.fatType(),DEC);     // FAT16 or FAT32?
  
  // Try to open the root directory
  if (!root.openRoot(vol)) {
    Serial.println("Can't open root dir!"); // Something went wrong,
    while(1);                             // then 'halt' - do nothing!
  }
  
  // Whew! We got past the tough parts.
  Serial.println("Ready!");
  dirLevel = 0;
}

void loop() {
  eyeStatic();
  /*if (Serial.available() > 0) {
      incomingByte = Serial.read();
        if (incomingByte == "a") {
          //
        }
          else if (incomingByte == "b") {
          //
          }
          else if (incomingByte == "c") {
          //
          }
          else if (incomingByte == "d") {
          //
          }
          else if (incomingByte == "e") {
          //
          }
        }
        else {
          allOn();
        }*/

    giddy();
    delay(180000);
   // happyStatic();
    //delay(180000);
    yes();
    Serial.print("Playing Yes...");
    delay(300);
    //delay(180000);
    yahoo();
    delay(180000);
    //waggle();
    //delay(180000);
    //upset();
    //delay(180000);
    //readyy();
    //delay(180000);
    //proximity();
    //delay(180000);
    //headturn();
    //delay(180000);
    /*newFirmware();
    delay(180000);
    poke();
    delay(180000);
    firmware();
    delay(180000);
    yahoo();
    delay(180000);
    */    FastLED.show();
  }
