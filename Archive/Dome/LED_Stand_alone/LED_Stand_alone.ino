#include "definitions.h"

void setup() {
  FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
  Serial.begin(9600);  
}

void loop() {
  psiStatic();
  hpStatic();
  eyeStatic();
  logicOneStatic();

  
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
  FastLED.show();
}
