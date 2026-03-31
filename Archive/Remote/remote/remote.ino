int PinNullBtn = 0;
int PinOneBtn = 1;
int PinTwoBtn = 2;
int PinThreeBtn = 3;
int PinFourBtn = 4;
int PinFiveBtn = 5;
int PinSixBtn = 6;
int PinSevenBtn = 7;
int PinEightBtn = 8;
int PinNineBtn = 9;

void setup() {
  Serial.begin(9600);
  pinMode(PinNullBtn, INPUT);
  pinMode(PinTwoBtn, INPUT);
  pinMode(PinOneBtn, INPUT);
  pinMode(PinThreeBtn, INPUT);
  pinMode(PinFourBtn, INPUT);
  pinMode(PinFiveBtn, INPUT);
  pinMode(PinSixBtn, INPUT);
  pinMode(PinSevenBtn, INPUT);
  pinMode(PinEightBtn, INPUT);
  pinMode(PinNineBtn, INPUT);
}

void loop() {
  
  if (digitalRead(PinNullBtn)= "HIGH") {
    Serial.println("a"); 
  } 
 
  else if (digitalRead(PinOneBtn) = "HIGH") { 
    Serial.println("b");
  }

  else if (digitalRead(PinTwoBtn) = "HIGH") { 
    Serial.println("c");
  }
 
  else if (digitalRead(PinThreeBtn) = "HIGH") { 
    Serial.println("d");
  }

  else if (digitalRead(PinFourBtn) = "HIGH") { 
    Serial.println("e");
  }

  else if ( digitalRead(PinFiveBtn = "HIGH" ) { 
    Serial.println("f");
  }

  else if ( digitalRead(PinSixBtn = "HIGH" ) { 
    Serial.println("g");
  }

  else if ( PinSevenBtn = "HIGH") { 
    Serial.println("h");
  }
           
  else if (PinEightBtn = "HIGH") { 
    Serial.println("i") ;
  }

  else if (PinNineBtn = "HIGH") { 
    Serial.println("j") ;
  }
}
