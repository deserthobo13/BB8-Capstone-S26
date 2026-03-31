
#include <VarSpeedServo.h>


int pos1;
int pos2;
int pos3;
int pos4;
int pos5;
int mappos1;
int mappos2;
int mappos3;
int mappos4;
int mappos5;
int map2pos1;
int map2pos3;
int audiorcv;
int posarray[8];
int i;


//Define Pins for Servos
int servo1Pin = 9;//aluminum bracket tilt
int servo2Pin = 11;//dome rotate
int servo3Pin = 10; //plastic servo bracket tilt
int servo4Pin = 6;//drive
int servo5Pin = 5;//unused..I assume it was the bottom weight rotation




//Create Servo Object
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo servo5;//unused

void setup()
{
//Start Serial
Serial.begin(19200);

 
 //Attaches the Servo to our object
 servo1.attach(servo1Pin);
 servo2.attach(servo2Pin);
 servo3.attach(servo3Pin);
 servo4.attach(servo4Pin);
 servo5.attach(servo5Pin); //unused
 

 //servo4.slowmove(25, 255); //not sure why this was ever here, it will make the servo move from the start
 
}


void loop() {
  //Serial.println("debug");
if(Serial.read() == ','){
while(Serial.available() >= 6)  {
 //Serial.println("Serial is avail");
    for(i = 0; i < 7; i ++) {         
        posarray[i] = Serial.read();    
}
    }

    
//map(value, start of source range, end of source range, start of target range,end of target range)
//map(posarray[0], 0, 180, 180, 30); returns if posArray is 0->89 return 180, else if 90-on returns 29
pos1 = map(posarray[0], 0, 180, 180, 30); //Dome Tilt

if (posarray[1] == 90){//if I receive 90 from the controll then no joystick is being touched
  mappos2 = 92;//set continuous rotation servo to stopped position..for mine 92...
}
else{
  pos2 = map(posarray[1], 0, 180, 0, 155); //Dome Spin
  mappos2 = constrain(pos2, 0, 180); //dome spin?
}

pos3 = map(posarray[2], 0, 180, 40, 140); //Pend Tilt
pos4 = map(posarray[3], 0, 50, 0, 180); //Drive
pos5 = map(posarray[4], 0, 180, 0, 180); //Body Spin

//????? why further constrain if used map...
//Constrain will return the value if it is between the two numbers else the 
//it returns either the lower number or the highest
mappos1 = constrain(pos1, 40, 130);//dome tilt?
mappos3 = constrain(pos3, 40, 150); //pend tilt?
mappos4 = constrain(pos4, 0, 180); //Drive
//mappos5 = constrain(pos5, 0, 180); //Body Weight Spin..unused

audiorcv = posarray[5];
 
servo1.slowmove(mappos3, 20); //pend tilt..pin 9..ALUMINUM Servo
servo2.slowmove(mappos2, 20); //dome spin?..pin 11..DOME SPIN
servo3.slowmove(mappos1, 20); //dome tilt?..pin 10..PLASTIC BRACKET SERVO
servo4.slowmove(mappos4, 80); //Drive...pin 6..SPHERE SPIN

//not connected..i believe it was the bottom rotational weight spin
//servo5.slowmove(mappos5, 255); //body spin?? .. pin 5...??

}//serial.read


delay(10);


    




}
