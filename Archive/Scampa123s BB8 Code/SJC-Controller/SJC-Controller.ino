//Remember to remove XBEE from Arduino Pro Mini before trying to upload sketch!


//define the button pins
int button1 = 3;
int button2 = 4;
int button3 = 5;
int button4 = 6;
int button5 = 8;
int buttonTop1 = 13;
int buttonTop2 = 12;
int buttonTop3 = 11;
int buttonJoyStick = 10;

//maintain the state of the buttons
int button1state = 0;
int button2state = 0;
int button3state = 0;
int button4state = 0;
int button5state = 0;
int buttonTop1state = 0;
int buttonTop2state = 0;
int buttonTop3state = 0;
int buttonJoyStickstate = 0;

//this is a variable to hold the current audio "button" sent via XBEE
int audio;


void setup()
{
 //Create Serial Object (19200 Baud)..all XBEEs are running at 19200 baud
 Serial.begin(19200);


//using internal pullup resistors..which means all buttons ar HIGH until pressed
 pinMode(button1, INPUT_PULLUP);
 pinMode(button2, INPUT_PULLUP);
 pinMode(button3, INPUT_PULLUP);
 pinMode(button4, INPUT_PULLUP);
 pinMode(button5, INPUT_PULLUP);
 pinMode(buttonTop1, INPUT_PULLUP);
 pinMode(buttonTop2, INPUT_PULLUP);
 pinMode(buttonTop3, INPUT_PULLUP);
 pinMode(buttonJoyStick, INPUT_PULLUP);

}

void loop()
{


//joystick and psp thumbsticks
//you need to calibrate this code to set the "untouched" state of the joysticks
//since this code is a loop and continually sends signals to the receivers
int val1 = analogRead(0);
if (val1 >= 75 && val1 <= 90)//determine if bottom psp is untouched!
{
  val1 = 90; //no movement of psp/servo
}
else
{
  val1 = map(val1, 0, 1023, 0, 180); //bottom psp //dome tilt...bottom psp
}

int val2 = analogRead(4);

if(val2 >= 430 && val2 <= 520)
{
  val2 = 90;
}
else{
  val2 = map(val2, 0, 1023, 0, 180); //bottom psp //dome spin ...bottom psp
}

int val3 = analogRead(2);
if(val3 == 90 )
{
  val3 = 90;
}
else
{
  val3 = map(val3, 0, 1023, 0, 180);  //joystick //pend tilt..joystick
}

int val4 = analogRead(3);
if (val4 == 88)
{
  val4 = 90;
}
else
{
  val4 = map(val4, 0, 1023, 0, 50); //joystick //drive...joystick
}

//int val5 = map(analogRead(4), 0, 1023, 0, 180); //top psp slider //unused at moment
int val5 = 90;

//tactile switch buttons and joystick "press" button
button1state = digitalRead(button1);
digitalWrite(button1, HIGH); //reset to HIGH if pressed
button2state = digitalRead(button2);
digitalWrite(button2, HIGH); //reset to HIGH if pressed
button3state = digitalRead(button3);
digitalWrite(button3, HIGH); //reset to HIGH if pressed
button4state = digitalRead(button4);
digitalWrite(button4, HIGH); //reset to HIGH if pressed
button5state = digitalRead(button5);
digitalWrite(button5, HIGH); //reset to HIGH if pressed
buttonTop1state = digitalRead(buttonTop1);
digitalWrite(buttonTop1, HIGH); //reset to HIGH if pressed
buttonTop2state = digitalRead(buttonTop2);
digitalWrite(buttonTop2, HIGH); //reset to HIGH if pressed
buttonTop3state = digitalRead(buttonTop3);
digitalWrite(buttonTop3, HIGH); //reset to HIGH if pressed
buttonJoyStickstate = digitalRead(buttonJoyStick);
digitalWrite(buttonJoyStick, HIGH); //reset to HIGH if pressed


//Determine which button is pushed
if (button1state == LOW) {
   audio=10; //set the audio file to tell the Dome to play..not 0! 0 means play nothing
} 

else if (button2state == LOW) {
   audio=1;
} 
else if (button3state == LOW) {
   audio=2;
} 
else if (button4state == LOW) {
   audio=3;
} 
else if (button5state == LOW) {
   audio=4;
} 
else if (buttonTop1state == LOW) {
   audio=5;
} 
else if (buttonTop2state == LOW) {
   audio=6;
} 
else if (buttonTop3state == LOW) {
   audio=7;
} 
else if (buttonJoyStickstate == LOW) {
   audio=8;
} 
else{
  audio = 0;//send 0 to play nothing..check this in dome code
}

Serial.write(',');//char to look for to capture data
Serial.write(val1); //dome tilt...bottom psp
Serial.write(val2); //dome spin ...bottom psp
Serial.write(val3);  //pend tilt..joystick
Serial.write(val4); //drive...joystick
Serial.write(val5); //unused
Serial.write(audio);//sends the audio to play

delay(25);
  
  
}
