#include <FatReader.h>
#include <SdReader.h>
#include <avr/pgmspace.h>
#include "WaveUtil.h"
#include "WaveHC.h"
#include <EEPROM.h>
#include "Adafruit_TLC5947.h"

// How many boards do you have chained?  This is for Adafruit LED board
#define NUM_TLC5974 1

//define these pins for the WaveHC
#define data   30
#define clock   32
#define latch   34

/**
 * Wave board setup
 */
SdReader card;    // This object holds the information for the card
FatVolume vol;    // This holds the information for the partition on the card
FatReader root;   // This holds the information for the filesystem on the card
FatReader f;      // This holds the information for the file we're play
WaveHC wave;      // This is the only wave (audio) object, since we will only play one at a time

#define DEBOUNCE 100  // button debouncer ...ISN'T USED!!

const int buttonPin = 17;    // the number of the pushbutton pin Unused for now
const int ledPin = 18;      // the number of the LED pin Unused for now

int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
int sound = 0;
long lights = 0;


/**
 * NOTE: I need to clean this up a bit since I believe all the lights (except the Holo) are 
 * being powered of the Adafruit board I am using.  It was much easier to me.  
 * The Holo is connected (with a resistor) to the output of the Amp board as per Ed's wiring.
 * This enables the Holo to flash when sounds are played.
 */

// the pin that the LED is attached to  THIS SECTION NEEDS TO BE UPDATED....
int eye = 24;
int psi = 26;
int holo = 28;
int sys1 = 9;//this will be the Adafruit board

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by


long lastDebounceTime = 0;  // the last time the output pin was toggled..ISN'T USED!!
long debounceDelay = 10;    // the debounce time; increase if the output flickers...ISN'T USED!!

char posarray[7];
int playaudio = 0;
int  i;

/**
 * Adafruit board setup
 * https://www.adafruit.com/products/1429
 */
Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5974, clock, data, latch);




/**
   SETUP
*/
void setup() {
  // set up serial port
  Serial.begin(38400);//default serial port used for debug
  putstring_nl("WaveHC with 6 buttons");
  Serial1.begin(19200); //this is where the xbee is attached to the Mega

  tlc.begin();
  lightDome(); //call method to light the leds on the Adafruit board

  putstring("Free RAM: ");       // This can help with debugging, running out of RAM is bad
  Serial.println(freeRam());      // if this is under 150 bytes it may spell trouble!

  // Set the output pins for the DAC control. This pins are defined in the library
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // pin13 LED
  pinMode(13, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  pinMode(eye, OUTPUT);
  pinMode(psi, OUTPUT);
  pinMode(holo, OUTPUT);
  pinMode(sys1, OUTPUT);

  digitalWrite(eye, HIGH);
  digitalWrite(psi, HIGH);
  digitalWrite(holo, HIGH);
  digitalWrite(sys1, HIGH);

  digitalWrite(17, HIGH);
  digitalWrite(18, LOW);
  digitalWrite(19, HIGH);

  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);


/**
 * This is for SD Card in Wave
 */
  //  if (!card.init(true)) { //play with 4 MHz spi if 8MHz isn't working for you
  if (!card.init()) {         //play with 8 MHz spi (default faster!)
    putstring_nl("Card init. failed!");  // Something went wrong, lets print out why
    sdErrorCheck();
    while (1);                           // then 'halt' - do nothing!
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
    while (1);                           // then 'halt' - do nothing!
  }

  // Lets tell the user about what we found
  putstring("Using partition ");
  Serial.print(part, DEC);
  putstring(", type is FAT");
  Serial.println(vol.fatType(), DEC);    // FAT16 or FAT32?

  // Try to open the root directory
  if (!root.openRoot(vol)) {
    putstring_nl("Can't open root dir!"); // Something went wrong,
    while (1);                            // then 'halt' - do nothing!
  }

  // Whew! We got past the tough parts.
  putstring_nl("Ready!");
  playcomplete("103.WAV");

}//end setup

/**
   MAIN LOOP
*/
void loop() {

/**
 * Read the XBEE for the array sent from the controller
 */
  if (Serial1.read() == ',') {

    while (Serial1.available() >= 6)  {
      for (i = 0; i < 7; i ++) {
        posarray[i] = Serial1.read(); //build array from incoming xbee data
      }
    }
  }

  sound = posarray[5]; //position 5 is the audio number for audio file to play..

  if (lights > 500) {

    // set the brightness of pin 9:
    analogWrite(psi, brightness);

    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness == 0 || brightness == 255) {
      fadeAmount = -fadeAmount ;
    }
    // wait for 30 milliseconds to see the dimming effect

  }

  delay(10);

  lights++;

  if (lights > 100) {
    digitalWrite(sys1, HIGH);
  }

  if (lights > 110) {
    digitalWrite(sys1, LOW);
  }

  if (lights > 115) {
    digitalWrite(sys1, HIGH);
  }

  if (lights > 130) {
    digitalWrite(sys1, LOW);
  }


  if (lights > 200) {
    digitalWrite(sys1, LOW);
  }

  if (lights > 500) {
    digitalWrite(sys1, HIGH);
  }
  if (lights > 505) {
    digitalWrite(sys1, LOW);
  }
  if (lights > 515) {
    digitalWrite(sys1, HIGH);
  }
  if (lights > 520) {
    digitalWrite(sys1, LOW);
  }

  if (lights > 540) {
    digitalWrite(sys1, HIGH);
  }

  if (lights > 800) {
    digitalWrite(sys1, LOW);
  }
  if (lights > 900) {
    digitalWrite(sys1, HIGH);
  }
  if (lights > 800) {
    digitalWrite(sys1, LOW);
  }

  if (lights > 1000) {
    digitalWrite(sys1, HIGH);
  }

  if (lights > 1200) {
    digitalWrite(sys1, LOW);
  }
  if (lights > 1500) {
    lights = 0;
  }


  //Determine which sound to play
  if (sound == 10) {
    playcomplete("1.WAV");
  }
  else if (sound == 1) {
    playcomplete("2.WAV");
  }

  else if (sound == 2) {
    playcomplete("3.WAV");
  }

  else if (sound == 3) {
    playcomplete("4.WAV");
  }
  else if (sound == 4) {
    playcomplete("5.WAV");
  }

  else if (sound == 5) {
    playcomplete("6.WAV");
  }

  else if (sound == 6) {
    playcomplete("7.WAV");
  }
  else if (sound == 7) {
    playcomplete("8.WAV");
  }

  else if (sound == 8) {
    playcomplete("9.WAV");
  }
  else {
    //play nothing!
  }

  sound == 0; //play nothing next time through loop...

  //could make a routine to cycle through all sounds..

}

/**
 * This is specifically for the Adafruit board I am using
 */
void lightDome()
{
  //led pin / brightness
  tlc.setPWM(0,4095);
  tlc.setPWM(1,4095);
  tlc.setPWM(2,4095);
  tlc.setPWM(3,4095);
  tlc.setPWM(4,4095);
  tlc.setPWM(5,4095);
  tlc.setPWM(6,4095);
  tlc.setPWM(7,4095);
  tlc.setPWM(8,4095);
  
  tlc.setPWM(16,4095);
  tlc.setPWM(17,4095);
  tlc.setPWM(18,4095);
  tlc.setPWM(19,4095);
  tlc.setPWM(20,4095);
  tlc.setPWM(21,4095);
  tlc.setPWM(22,4095);
  tlc.setPWM(23,4095);
  
  tlc.write(); //always call this to ensure the board gets the info

}


/**
   Plays a full file from beginning to end with no pause
*/
void playcomplete(char *name) {
  // call our helper to find and play this name
  playfile(name);
  while (wave.isplaying) {
    // do nothing while its playing
  }
  // now its done playing
}

/**
   Plays a file
*/
void playfile(char *name) {
  Serial.print(name);
  // see if the wave object is currently doing something
  if (wave.isplaying) {// already playing something, so stop it!
    wave.stop(); // stop it
  }
  // look in the root directory and open the file
  if (!f.open(root, name)) {
    putstring("Couldn't open file "); Serial.print(name); return;
  }
  // OK read the file and turn it into a wave object
  if (!wave.create(f)) {
    putstring_nl("Not a valid WAV"); return;
  }

  // ok time to play! start playback
  wave.play();
}

/**
 * this handy function will return the number of bytes currently free in RAM, great for debugging!
 */
int freeRam(void)
{
  extern int  __bss_end;
  extern int  *__brkval;
  int free_memory;
  if ((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}

/**
 * SD Card Error Checking routine
 */
void sdErrorCheck(void)
{
  if (!card.errorCode()) return;
  putstring("\n\rSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  putstring(", ");
  Serial.println(card.errorData(), HEX);
  while (1);
}
