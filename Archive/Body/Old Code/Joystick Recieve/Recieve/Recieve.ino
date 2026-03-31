void setup()
{
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  
}

void loop() 
{
  
if (VERTVAL < 500) {
    digitalWrite(13, HIGH);
  }

  else if (VERTVAL > 500) {
    digitalWrite(14, HIGH);
  }

  else if (HORIZVAL < 500) {
    digitalWrite(15, HIGH);
  }

  else if (HORIZVAL >500) {
    digitalWrite(16, HIGH);
  }

}  