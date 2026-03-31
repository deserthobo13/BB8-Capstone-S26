#include "FastLED.h"
#define NUM_LEDS 14 //Set Number Here
CRGB leds[NUM_LEDS];
void setup() {
    FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
  }

void loop() {
    leds[0] = CRGB::RED; FastLED.show(); delay(20);
  }
