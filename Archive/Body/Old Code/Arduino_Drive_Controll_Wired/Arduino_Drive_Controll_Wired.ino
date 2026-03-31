
#include <Wire.h>
#include <ArduinoNunchuk.h>

ArduinoNunchuk nunchuk = ArduinoNunchuk();

unsigned short left = 9;
unsigned short right = 10;
unsigned short rotate = 11;

signed int dutyCycle = 0; 
unsigned int highus = 1600;
unsigned int highus1 = 1600;
unsigned int lowus = 1000;

void setup()
{
  pinMode(left, OUTPUT); 
  digitalWrite(left, LOW);
  Serial.begin(19200);
  nunchuk.init();
}

void loop()
{
  nunchuk.update();

  Serial.print(nunchuk.analogX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.analogY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.zButton, DEC);
  Serial.print(' ');
  Serial.println(nunchuk.cButton, DEC);
  delay(100);

  if (nunchuk.analogX > 122) {
    Serial.print("RIGHT");
  }
  else if (nunchuk.analogX < 120) {
    Serial.print("LEFT");  
  }
  else if (nunchuk.analogY > 135) {
    Serial.print("FORWARD");
    digitalWrite(left, HIGH);
    delayMicroseconds(highus);
    digitalWrite(left, LOW);
    delayMicroseconds(lowus);
  }
  else if (nunchuk.analogY < 134 ) {
    Serial.print("BACKWARDS");
  }
  else if (nunchuk.zButton > 0) {
    Serial.print("ZPRESSED");  
  }
  else if (nunchuk.cButton < 0) {
    Serial.print("CPRESSED");  
  }
}
