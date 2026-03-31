const int VERT = 0; // 1 525/526 1021 -- Analog Pin 0
const int HORIZ = 1; // 1 499/498 1021 -- Analog Pin 1
const int SEL = 3; //Digital Pin 30

// Also connect the joystick VCC to Arduino 5V, and joystick GND to Arduino GND.

// This sketch outputs serial data at 9600 baud (open Serial Monitor to view).
void setup()
{
  // make the SEL line an input
  pinMode(SEL, INPUT);
  //End Test LEDS


  // turn on the pull-up resistor for the SEL line (see http://arduino.cc/en/Tutorial/DigitalPins)
  //digitalWrite(SEL,HIGH);

  // set up serial port for output
  Serial.begin(9600);
}

void loop() 
{
  int vertical, horizontal, select;
    
  vertical = analogRead(VERT); // will be 0-1023
  horizontal = analogRead(HORIZ); // will be 0-1023
  select = digitalRead(SEL); // will be HIGH (1) if not pressed, and LOW (0) if pressed

  Serial.Write(',');
  Serial.Write(vertical);
  Serial.Write('|');
  Serial.Write(horizontal);

}  