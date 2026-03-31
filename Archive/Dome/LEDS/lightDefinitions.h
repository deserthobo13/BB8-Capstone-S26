#ifndef lightDefinitions_h
#define lightDefinitions_h

#include "FastLED.h"

#define NUM_LEDS 12
#define DATA_PIN 11
#define CLOCK_PIN 13

CRGB leds[NUM_LEDS];

void allOn() {
  leds[0] = CRGB::Blue;
  leds[1] = CRGB::White;
  leds[2] = CRGB::Red;
  leds[3] = CRGB::White;
  leds[4] = CRGB::White;
  leds[5] = CRGB::White;
  leds[6] = CRGB::White;
  leds[7] = CRGB::White;
  leds[8] = CRGB::White;
  leds[9] = CRGB::White;
  leds[10] = CRGB::White;
  leds[11] = CRGB::White;
  leds[12] = CRGB::White;
  FastLED.setBrightness(50);
  FastLED.show();
}

void psiStatic() {
  leds[0] = CRGB::White;
}

void psiBlinkS1() {
  leds[0] = CRGB::White;
  FastLED.show();
  delay(500); //change value acordingly
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500); //change value acordingly
}

void hpStatic() {
  leds[1] = CRGB::Blue;
}

void eyeStatic() {
  leds[2] = CRGB::Red;
}

void logicOneStatic() {
  leds[3] = CRGB::White;
  leds[4] = CRGB::White;
  leds[5] = CRGB::White;
}

void logicTwoStatic() {
  leds[6] = CRGB::Blue;
  leds[7] = CRGB::Blue;
  leds[8] = CRGB::Blue;
}

void logicThreeStatic() {
  leds[9] = CRGB::Blue;
  leds[10] = CRGB::Blue;
  leds[11] = CRGB::Blue;
  leds[12] = CRGB::Blue;
}

//AUDIO LIGHT DEFINITIONS
void psiAngry() {
}

void psiGiddy() {
}

void psiHappyStatic() {
}

void psiYahoo() {
}

void psiYes() {
}


#endif
