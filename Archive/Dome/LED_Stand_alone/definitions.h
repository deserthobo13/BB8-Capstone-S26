#ifndef definitions_h
#define definitions_h

#include "FastLED.h"

#define NUM_LEDS 12
CRGB leds[NUM_LEDS];


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

void allLogics() {
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
}

#endif
