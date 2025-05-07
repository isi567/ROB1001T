const int buttonPin = 5;  // the pin that the pushbutton is attached to
const int GreenLED = 4; 
const int RedLED = 3;   // the pin that the LED is attached to
int buttonState = 0;

float duration, distance;

void setup() {
  //setup inputs
  pinMode(buttonPin, INPUT);
  pinMode(GreenLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  Serial.begin( 115200 );
  Serial.println( "RS" );    
}

void loop() {
  digitalWrite(GreenLED, LOW);
  digitalWrite(RedLED, LOW);
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    Serial.write("\non");
  }
  else {
    Serial.write("\noff");
  }
    
}
